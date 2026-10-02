"""Talk to a Roomba through its built-in Wi-Fi radio - no add-on hardware.

Wi-Fi Roombas run a small MQTT server (TLS, port 8883) on your home network.
This module:

* finds robots on the network (UDP 5678 "irobotmcs" discovery),
* fetches the robot's local password - either straight from the robot
  (hold HOME while docked, older models) or once via your iRobot account,
* connects, follows the robot's state, and reads its position
  (``pose`` in the state reports on older models, or by polling the
  "RRTP" position request on newer firmware),
* sends mission commands (start / pause / resume / stop / dock).

Wi-Fi robots navigate themselves; there is no way to steer them over Wi-Fi.
The mapper therefore *watches* where the robot goes and builds the map from
that.

Protocol details come from community reverse-engineering (dorita980,
roombapy, roomba-v4 projects) - iRobot does not publish them - and newer
firmware (the "V4" protocol used by the Combo Essential, RVG-Y1) is still
being worked out. ``probe()`` reports what your particular robot supports.
"""

import json
import math
import os
import socket
import ssl
import threading
import time
import urllib.parse
import urllib.request
import uuid

from .mqtt_lite import MQTTClient, MQTTError, roomba_tls_context

DISCOVERY_PORT = 5678
MQTT_PORT = 8883

# -- models -------------------------------------------------------------------

# Matched against the SKU prefix reported by discovery / the cloud. Radii in
# metres: body_radius is the robot's footprint, clean_radius how far from its
# centre it cleans (the side brush reaches the edge of the body).
PROFILES = [
    ("Y01", {
        "key": "combo-essential",
        "name": "Roomba Combo Essential (RVG-Y1)",
        "mop": True, "body_radius": 0.17, "clean_radius": 0.17, "protocol": "v4",
        "notes": "V4 firmware. Local MQTT support varies by firmware; run `probe` to check. "
                 "Navigation is gyroscope-based, so poses drift more than on camera/LiDAR models.",
    }),
    ("Q01", {
        "key": "vac-essential",
        "name": "Roomba Vac Essential (RVG-Y1)",
        "mop": False, "body_radius": 0.17, "clean_radius": 0.17, "protocol": "v4",
        "notes": "Same platform as the Combo Essential without the mop.",
    }),
    ("X", {
        "key": "v4-matter",
        "name": "Roomba Combo (Matter generation)",
        "mop": True, "body_radius": 0.17, "clean_radius": 0.17, "protocol": "v4",
        "notes": "Reported to close the local MQTT port; may not be usable without the cloud.",
    }),
    ("R9", {"key": "900", "name": "Roomba 900 series", "mop": False,
            "body_radius": 0.17, "clean_radius": 0.17, "protocol": "classic",
            "notes": "Reports its position in state updates."}),
    ("", {"key": "classic", "name": "Wi-Fi Roomba", "mop": False,
          "body_radius": 0.17, "clean_radius": 0.17, "protocol": "classic",
          "notes": "Position via RRTP (i/s/j series) or state pose (900 series)."}),
]
MODEL_ALIASES = {"rvg-y1": "combo-essential", "combo": "combo-essential",
                 "combo-essential": "combo-essential", "vac-essential": "vac-essential"}


def profile_for(sku=None, key=None):
    if key:
        key = MODEL_ALIASES.get(key.lower(), key.lower())
        for _, prof in PROFILES:
            if prof["key"] == key:
                return dict(prof)
        raise ValueError(f"unknown model {key!r}")
    sku = (sku or "").upper()
    for prefix, prof in PROFILES:
        if sku.startswith(prefix):
            return dict(prof)
    return dict(PROFILES[-1][1])


# -- discovery ------------------------------------------------------------------


def parse_discovery(data):
    """Decode a discovery reply (JSON, sometimes with a 2-byte length prefix)."""
    start = data.find(b"{")
    if start < 0:
        raise ValueError("not a discovery reply")
    info = json.loads(data[start:].decode("utf-8", "replace"))
    hostname = info.get("hostname", "")
    blid = hostname.split("-", 1)[1] if "-" in hostname else None
    return {
        "ip": info.get("ip"),
        "blid": blid,
        "name": info.get("robotname"),
        "sku": info.get("sku"),
        "firmware": info.get("sw"),
        "mac": info.get("mac"),
        "protocol_version": str(info.get("ver", "")),
        "matter": info.get("matter"),
        "raw": info,
    }


def discover(timeout=3.0, target="255.255.255.255", port=DISCOVERY_PORT):
    """Broadcast for robots; returns a list of parsed discovery replies."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.settimeout(0.3)
    found = {}
    try:
        deadline = time.time() + timeout
        next_send = 0.0
        while time.time() < deadline:
            if time.time() >= next_send:
                s.sendto(b"irobotmcs", (target, port))
                next_send = time.time() + 1.0
            try:
                data, addr = s.recvfrom(4096)
            except socket.timeout:
                continue
            if data == b"irobotmcs":
                continue  # our own broadcast echoed back
            try:
                info = parse_discovery(data)
            except ValueError:
                continue
            info["ip"] = info["ip"] or addr[0]
            found[info["blid"] or addr[0]] = info
    finally:
        s.close()
    for info in found.values():
        info["profile"] = profile_for(info.get("sku"))
    return list(found.values())


# -- credentials ----------------------------------------------------------------

PASSWORD_REQUEST = bytes.fromhex("f005efcc3b2900")


def get_password_local(ip, port=MQTT_PORT, timeout=10.0, ssl_context=None):
    """Ask the robot itself for its password (older firmware).

    Put the robot on its dock and hold HOME for about 2 seconds until it
    plays a tone (or the Wi-Fi light flashes), then call this within a
    minute.
    """
    ctx = ssl_context or roomba_tls_context()
    with socket.create_connection((ip, port), timeout=timeout) as raw:
        with ctx.wrap_socket(raw, server_hostname=None) as sock:
            sock.sendall(PASSWORD_REQUEST)
            data = b""
            deadline = time.time() + timeout
            while time.time() < deadline:
                try:
                    chunk = sock.recv(1024)
                except socket.timeout:
                    break
                if not chunk:
                    break
                data += chunk
                if len(data) > 7 and len(data) >= 2 + data[1]:
                    break
    if len(data) <= 7:
        raise RuntimeError("robot did not send a password - hold HOME on the docked robot "
                           "until it beeps, then try again within a minute")
    return data[7:].split(b"\x00", 1)[0].decode("utf-8", "replace")


DISCOVERY_URL = "https://disc-prod.iot.irobotapi.com/v1/discover/endpoints?country_code={cc}"
APP_ID = "ANDROID-C7FB240E-DF34-42D7-AE4E-A8C17079A294"


def _http_json(url, data=None, headers=None, opener=None):
    opener = opener or urllib.request.urlopen
    hdrs = {"User-Agent": "roomba-mapper", "Accept": "application/json"}
    hdrs.update(headers or {})
    body = None
    if isinstance(data, dict) and hdrs.get("Content-Type") == "application/json":
        body = json.dumps(data).encode()
    elif data is not None:
        body = urllib.parse.urlencode(data).encode()
        hdrs.setdefault("Content-Type", "application/x-www-form-urlencoded")
    req = urllib.request.Request(url, data=body, headers=hdrs)
    with opener(req, timeout=20) as res:
        return json.loads(res.read().decode())


def get_credentials_cloud(email, password, country="US", opener=None):
    """Log in to your iRobot account once to read each robot's local password.

    Your account password is only sent to iRobot's own login servers and is
    not stored. Returns [{"blid", "password", "name", "sku", "firmware", ...}].
    """
    disc = _http_json(DISCOVERY_URL.format(cc=urllib.parse.quote(country)), opener=opener)
    gigya = disc["gigya"]
    deployment = disc["deployments"][disc["current_deployment"]]
    login = _http_json(
        f"https://accounts.{gigya['datacenter_domain']}/accounts.login",
        {"apiKey": gigya["api_key"], "loginID": email, "password": password,
         "targetEnv": "mobile", "format": "json"},
        opener=opener)
    if login.get("errorCode"):
        raise RuntimeError(f"iRobot login failed: {login.get('errorMessage', login['errorCode'])}")
    body = {
        "app_id": APP_ID,
        "assume_robot_ownership": "0",
        "gigya": {"signature": login["UIDSignature"], "timestamp": login["signatureTimestamp"],
                  "uid": login["UID"]},
    }
    res = _http_json(f"{deployment['httpBase']}/v2/login", body,
                     headers={"Content-Type": "application/json"}, opener=opener)
    robots = []
    for blid, info in (res.get("robots") or {}).items():
        robots.append({
            "blid": blid,
            "password": info.get("password"),
            "name": info.get("name"),
            "sku": info.get("sku"),
            "firmware": info.get("softwareVer"),
            "deployment": info.get("svcDeplId"),
            "profile": profile_for(info.get("sku")),
        })
    return robots


# -- state ----------------------------------------------------------------------

RETURN_PHASES = {"hmUsrDock", "hmPostMsn", "hmMidMsn"}


def activity_from(reported):
    """Collapse cleanMissionStatus into one simple word for the GUI."""
    cms = reported.get("cleanMissionStatus") or {}
    phase, cycle = cms.get("phase"), cms.get("cycle")
    if phase == "run":
        return "cleaning"
    if phase in RETURN_PHASES:
        return "returning"
    if phase == "stuck":
        return "stuck"
    if phase == "evac":
        return "emptying"
    if phase in ("pause",) or (phase == "stop" and cycle not in (None, "none")):
        return "paused"
    if phase == "charge":
        return "docked"
    if phase is None:
        return "unknown"
    return "idle"


def pose_from_reported(reported, units="mm"):
    """Classic state pose -> (x_m, y_m, theta_rad) or None."""
    pose = reported.get("pose")
    if not isinstance(pose, dict) or not isinstance(pose.get("point"), dict):
        return None
    try:
        scale = {"mm": 0.001, "cm": 0.01, "m": 1.0}[units]
        x = float(pose["point"]["x"]) * scale
        y = float(pose["point"]["y"]) * scale
        th = math.radians(float(pose.get("theta", 0)))
    except (KeyError, TypeError, ValueError):
        return None
    return x, y, th


def pose_from_rrtp(msg):
    """RRTP 'data' reply -> (x_m, y_m, theta_rad, pmap_id) or None."""
    if msg.get("reportType") != "current":
        return None
    for entry in msg.get("data") or []:
        coords = entry.get("coords")
        xyt = None
        if isinstance(coords, list) and coords and isinstance(coords[0], dict):
            if coords[0].get("type") == "unknown":
                return None
            xyt = coords[0].get("xyt")
        elif isinstance(coords, list) and len(coords) == 3:
            xyt = coords
        if xyt and len(xyt) == 3:
            try:
                return float(xyt[0]), float(xyt[1]), float(xyt[2]), entry.get("pmap_id")
            except (TypeError, ValueError):
                return None
    return None


def resolve_path(data, path):
    """Follow a dotted path ('a.b.0.c') through dicts and lists; None if absent."""
    cur = data
    for part in path.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        elif isinstance(cur, list) and part.isdigit() and int(part) < len(cur):
            cur = cur[int(part)]
        else:
            return None
    return cur


def pose_from_value(value, units="mm", angle="deg"):
    """Interpret a position value in any of the shapes robots use.

    Accepts {"point": {"x", "y"}, "theta"}, {"x", "y", "theta"/"t"/"heading"},
    or [x, y] / [x, y, theta]. Returns (x_m, y_m, theta_rad) or None.
    """
    scale = {"mm": 0.001, "cm": 0.01, "m": 1.0}[units]
    th = 0.0
    try:
        if isinstance(value, dict):
            pt = value.get("point") if isinstance(value.get("point"), dict) else value
            x, y = float(pt["x"]), float(pt["y"])
            for key in ("theta", "t", "heading", "angle", "yaw"):
                if key in value:
                    th = float(value[key])
                    break
        elif isinstance(value, (list, tuple)) and len(value) in (2, 3):
            x, y = float(value[0]), float(value[1])
            th = float(value[2]) if len(value) == 3 else 0.0
        else:
            return None
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (x, y, th)):
        return None
    return x * scale, y * scale, (math.radians(th) if angle == "deg" else th)


POSITION_HINTS = ("pose", "pos", "point", "coord", "xyt", "loc", "theta", "heading", "x", "y")
NOT_POSITION = ("batpct", "rssi", "snr", "noise", "time", "ts", "mssnm", "sqft", "expiretm",
                "nmssn", "rechrgm", "mssnstrttm", "bbrun", "bbchg", "bbpwr", "bbsys", "bbmssn",
                "tz", "utctime", "lastcommand", "version", "seq", "ver")


def _leaves(obj, prefix=""):
    if isinstance(obj, dict):
        for k, v in obj.items():
            yield from _leaves(v, f"{prefix}.{k}" if prefix else str(k))
    elif isinstance(obj, list) and len(obj) <= 16:
        for i, v in enumerate(obj):
            yield from _leaves(v, f"{prefix}.{i}" if prefix else str(i))
    elif isinstance(obj, (int, float)) and not isinstance(obj, bool):
        yield prefix, float(obj)


def position_candidates(messages, limit=15):
    """Find fields that look like a position in captured messages.

    messages: list of (topic, payload_dict). A position changes often while
    the robot drives and goes up *and* down (unlike counters and clocks), and
    usually has a telling name. Returns [{"path", "topic", "changes", "sample",
    "score"}] best first.
    """
    series = {}
    for topic, payload in messages:
        for path, val in _leaves(payload):
            series.setdefault((topic, path), []).append(val)
    out = []
    for (topic, path), vals in series.items():
        last = path.lower().split(".")[-1]
        if last in NOT_POSITION or any(abs(v) > 1e8 for v in vals):
            continue
        changes = sum(1 for a, b in zip(vals, vals[1:]) if a != b)
        if changes < 2:
            continue
        ups = sum(1 for a, b in zip(vals, vals[1:]) if b > a)
        downs = sum(1 for a, b in zip(vals, vals[1:]) if b < a)
        both_ways = min(ups, downs) > 0
        named = any(h in part for part in path.lower().split(".") for h in POSITION_HINTS
                    if len(h) > 1 or part == h)
        score = changes * (3 if both_ways else 1) * (4 if named else 1)
        out.append({"path": path, "topic": topic, "changes": changes,
                    "sample": vals[-5:], "score": score})
    out.sort(key=lambda c: -c["score"])
    return out[:limit]


SENSITIVE_KEYS = ("ssid", "bssid", "mac", "addr", "pass", "wlcfg", "netinfo", "gw", "dns",
                  "mask", "cloudenv", "svccnct", "country", "sku")


def redact(obj):
    """Copy of a message with network details removed, safe to share."""
    if isinstance(obj, dict):
        return {k: ("<redacted>" if any(s in k.lower() for s in SENSITIVE_KEYS) else redact(v))
                for k, v in obj.items()}
    if isinstance(obj, list):
        return [redact(v) for v in obj]
    return obj


def deep_merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst


# -- the live connection ------------------------------------------------------------


class WifiRoomba:
    """Live link to one robot. Callbacks fire on a background thread:

    on_state(reported_dict)            merged state report after each update
    on_pose(x, y, theta, source)       metres / radians in the robot's frame
    on_link(status_text, connected)    connection changes
    """

    def __init__(self, ip, blid, password, port=MQTT_PORT, tls=True, profile=None,
                 pose_source="auto", pose_units="mm", rrtp_interval=1.0, ssl_context=None,
                 pose_path=None, pose_angle="deg", rrtp_topic="req", rrtp_con_type="local"):
        self.ip, self.blid, self.password = ip, blid, password
        self.port, self.tls = port, tls
        self.profile = profile or profile_for()
        self.name = self.profile["name"]
        self.pose_source = pose_source
        self.pose_units = pose_units
        self.rrtp_interval = rrtp_interval
        self.ssl_context = ssl_context
        self.reported = {}
        self.on_state = None
        self.on_pose = None
        self.on_link = None
        self.on_raw = None           # callable(topic, payload_bytes), for diagnostics
        self.pose_path = pose_path   # read the position from this field of the state reports
        self.pose_angle = pose_angle
        self.rrtp_topic = rrtp_topic
        self.rrtp_con_type = rrtp_con_type
        self.rrtp_supported = None   # None = not yet known
        self.shadow_pose_seen = False
        self.last_pose = None
        self._client = None
        self._stop = threading.Event()
        self._rrtp_pending = {}
        self._rrtp_misses = 0
        self._lock = threading.Lock()

    # -- lifecycle --

    def start(self):
        threading.Thread(target=self._run, name="roomba-wifi", daemon=True).start()

    def close(self):
        self._stop.set()
        if self._client:
            self._client.close()

    @property
    def connected(self):
        return bool(self._client and self._client.connected)

    def _emit_link(self, text, ok):
        if self.on_link:
            self.on_link(text, ok)

    def _run(self):
        backoff = 5.0
        while not self._stop.is_set():
            try:
                self._connect_once()
                backoff = 5.0
                self._poll_loop()
            except (OSError, MQTTError, ssl.SSLError) as exc:
                msg = str(exc) or exc.__class__.__name__
                if isinstance(exc, ConnectionRefusedError):
                    msg = (f"robot refused a connection on port {self.port} - it may not offer local "
                           "control, or the iRobot app is connected (only one connection is allowed)")
                self._emit_link(f"Robot offline: {msg}. Retrying in {int(backoff)} s", False)
            if self._client:
                self._client.close()
                self._client = None
            if self._stop.wait(backoff):
                break
            backoff = min(backoff * 2, 120.0)

    def _connect_once(self):
        self._wake()
        client = MQTTClient(self.ip, self.port, self.blid, self.blid, self.password,
                            tls=self.tls, ssl_context=self.ssl_context)
        client.on_message = self._on_message
        disconnected = threading.Event()
        client.on_disconnect = lambda exc: disconnected.set()
        client.connect()
        client.subscribe("#", f"$aws/things/{self.blid}/#")
        self._client = client
        self._disconnected = disconnected
        self._emit_link(f"Connected to {self.name} at {self.ip}", True)
        self.publish("$aws/things/%s/shadow/get" % self.blid, "")

    def _wake(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.sendto(b"irobotmcs", (self.ip, DISCOVERY_PORT))
        except OSError:
            pass

    def _poll_loop(self):
        """Request RRTP positions while the robot is moving, until disconnected."""
        while not self._stop.is_set() and not self._disconnected.is_set():
            active = activity_from(self.reported) in ("cleaning", "returning", "unknown")
            want_rrtp = self.pose_source in ("auto", "rrtp") and active
            if self.pose_source == "auto" and self.shadow_pose_seen and not self.rrtp_supported:
                want_rrtp = False  # state reports already carry the pose
            interval = self.rrtp_interval if self.rrtp_supported is not False else 30.0
            if want_rrtp:
                self._request_rrtp()
            self._disconnected.wait(interval)
        if self._disconnected.is_set() and not self._stop.is_set():
            raise ConnectionError("connection to robot lost")

    def _request_rrtp(self, topic=None, con_type=None):
        req_id = uuid.uuid4().hex[:12]
        now = time.time()
        with self._lock:
            # count requests that were never answered
            expired = [k for k, t in self._rrtp_pending.items() if now - t > 3.0]
            for k in expired:
                del self._rrtp_pending[k]
                self._rrtp_misses += 1
            if self._rrtp_misses >= 5 and not self.rrtp_supported:
                self.rrtp_supported = False
            self._rrtp_pending[req_id] = now
        self.publish(topic or self.rrtp_topic,
                     json.dumps({"reqId": req_id, "reqType": "current",
                                 "conType": con_type or self.rrtp_con_type}))
        return req_id

    def publish(self, topic, payload):
        if self._client and self._client.connected:
            self._client.publish(topic, payload)

    # -- incoming --

    def _on_message(self, topic, payload):
        if self.on_raw:
            try:
                self.on_raw(topic, payload)
            except Exception:
                pass
        try:
            msg = json.loads(payload.decode("utf-8", "replace"))
        except ValueError:
            return
        if not isinstance(msg, dict):
            return
        if "reportType" in msg:
            self._on_rrtp(msg)
            return
        if isinstance(msg.get("state"), dict) and "reportType" in (msg["state"].get("reported") or {}):
            self._on_rrtp(msg["state"]["reported"])
            return
        reported = (msg.get("state") or {}).get("reported") if "state" in msg else msg
        if not isinstance(reported, dict):
            return
        with self._lock:
            deep_merge(self.reported, reported)
            snapshot = json.loads(json.dumps(self.reported))
        if self.on_state:
            self.on_state(snapshot)
        if self.pose_path:
            value = resolve_path(reported, self.pose_path)
            pose = pose_from_value(value, self.pose_units, self.pose_angle) if value is not None else None
            if pose:
                self.shadow_pose_seen = True
                self._emit_pose(*pose, "state:" + self.pose_path)
        elif self.pose_source in ("auto", "state", "shadow") and "pose" in reported:
            pose = pose_from_reported(reported, self.pose_units)
            if pose:
                self.shadow_pose_seen = True
                self._emit_pose(*pose, "state")

    def _on_rrtp(self, msg):
        with self._lock:
            self._rrtp_pending.pop(msg.get("reqId"), None)
        pose = pose_from_rrtp(msg)
        if msg.get("reportType") == "current":
            self.rrtp_supported = True
            self._rrtp_misses = 0
        if pose and self.pose_source in ("auto", "rrtp"):
            x, y, th, pmap = pose
            if pmap:
                self.reported.setdefault("_rrtp", {})["pmap_id"] = pmap
            self._emit_pose(x, y, th, "rrtp")

    def _emit_pose(self, x, y, th, source):
        self.last_pose = (x, y, th, source, time.time())
        if self.on_pose:
            self.on_pose(x, y, th, source)

    # -- commands --

    def command(self, verb, params=None):
        if verb not in ("start", "clean", "pause", "resume", "stop", "dock", "find", "evac"):
            raise ValueError(f"unsupported command {verb!r}")
        if not self.connected:
            raise RuntimeError("robot is not connected")
        body = {"command": verb, "time": int(time.time()), "initiator": "localApp"}
        if params:
            body["params"] = params
        self.publish("cmd", json.dumps(body))

    def capabilities(self):
        return {
            "connected": self.connected,
            "pose_from_state": self.shadow_pose_seen,
            "rrtp": self.rrtp_supported,
            "has_mop": self.profile.get("mop", False),
            "can_drive": False,
        }


def mop_params(mode, wetness=2):
    """Start parameters for vacuum-only or vacuum+mop (Combo models).

    Values follow the community-documented V4 command format; robots that
    don't understand them ignore the params and use their app settings.
    """
    if mode == "vacuum":
        return {"operatingMode": 2}
    if mode == "mop":
        w = max(1, min(3, int(wetness)))
        return {"operatingMode": 6, "padWetness": {"disposable": w, "reusable": w}}
    return None


V4_SHADOWS = ("ro-currentstate", "ro-stats", "ro-configinfo", "ro-services", "rw-settings")


def suggest_pose_path(candidates):
    """Turn the best candidate leaf (e.g. 'cleanMissionStatus.pos.point.x') into a --pose-path."""
    for c in candidates:
        parts = c["path"].split(".")
        if parts[-1] in ("x", "y", "0", "1"):
            parts = parts[:-1]
            if parts and parts[-1] == "point":
                parts = parts[:-1]
            if parts:
                return ".".join(parts)
    return None


def probe(ip, blid=None, password=None, port=MQTT_PORT, tls=True, listen=20.0, log=print,
          dump_path=None):
    """Check what a robot supports. Prints findings and returns them as a dict.

    With dump_path, every message the robot sends is saved there (one JSON per
    line, network details removed) so unknown position fields can be found.
    """
    report = {"ip": ip}
    log(f"1. Discovery (UDP {DISCOVERY_PORT})")
    try:
        found = discover(timeout=3.0, target=ip)
        info = found[0] if found else None
    except OSError as exc:
        info = None
        log(f"   error: {exc}")
    if info:
        prof = info["profile"]
        report["discovery"] = info["raw"]
        log(f"   {info['name']!r} sku={info['sku']} firmware={info['firmware']} "
            f"protocol ver={info['protocol_version'] or '?'} -> {prof['name']}")
        log(f"   note: {prof['notes']}")
        blid = blid or info["blid"]
    else:
        log("   no reply (robot asleep, different subnet, or discovery disabled)")
    log(f"2. Local MQTT port {port}")
    try:
        with socket.create_connection((ip, port), timeout=5):
            pass
        report["port_open"] = True
        log("   open")
    except OSError as exc:
        report["port_open"] = False
        log(f"   closed ({exc}). This robot does not offer local control on its radio.")
        return report
    if not (blid and password):
        log("   (give --blid and --password to test the connection itself)")
        return report

    link = WifiRoomba(ip, blid, password, port=port, tls=tls)
    events = {"state": 0, "pose_state": 0, "pose_rrtp": 0}
    link.on_state = lambda r: events.__setitem__("state", events["state"] + 1)

    def on_pose(x, y, th, src):
        events["pose_rrtp" if src == "rrtp" else "pose_state"] += 1
    link.on_pose = on_pose

    captured = []           # (topic, dict)
    topics = {}
    pending = {}            # reqId -> variant description
    answered = set()
    dump = open(dump_path, "w", encoding="utf-8") if dump_path else None

    def on_raw(topic, payload):
        topics[topic] = topics.get(topic, 0) + 1
        text = payload.decode("utf-8", "replace")
        for req_id, variant in list(pending.items()):
            if req_id in text:
                answered.add(variant)
        try:
            msg = json.loads(text)
        except ValueError:
            msg = {"_raw": text[:2000]}
        if isinstance(msg, dict):
            body = (msg.get("state") or {}).get("reported") if isinstance(msg.get("state"), dict) else msg
            captured.append((topic, body if isinstance(body, dict) else msg))
        if dump:
            dump.write(json.dumps({"t": round(time.time(), 2), "topic": topic, "msg": redact(msg)}) + "\n")
    link.on_raw = on_raw

    log("3. Logging in")
    try:
        link._connect_once()
    except (OSError, MQTTError, ssl.SSLError) as exc:
        report["login"] = str(exc)
        log(f"   failed: {exc}")
        if dump:
            dump.close()
        return report
    report["login"] = "ok"
    for name in V4_SHADOWS:  # ask newer firmware for its named state documents too
        link.publish(f"$aws/things/{blid}/shadow/name/{name}/get", "")
    variants = [("req", "local"), ("req", "remote"), (f"$aws/things/{blid}/req", "local")]
    variant_flags = {f"topic={t} conType={c}": (t, c) for t, c in variants}
    log(f"   ok. Listening {int(listen)} s. For a useful result the robot should be cleaning, "
        "away from the dock, the whole time...")
    end, k = time.time() + listen, 0
    while time.time() < end:
        topic, con = variants[k % len(variants)]
        req_id = link._request_rrtp(topic=topic, con_type=con)
        pending[req_id] = f"topic={topic} conType={con}"
        k += 1
        time.sleep(1.0)
    time.sleep(1.5)  # late replies
    link.close()
    if dump:
        dump.close()

    candidates = position_candidates(captured)
    suggestion = suggest_pose_path(candidates)
    report.update(events, rrtp=link.rrtp_supported, activity=activity_from(link.reported),
                  topics=topics, rrtp_variants_answered=sorted(answered),
                  candidates=candidates, suggested_pose_path=suggestion)
    log(f"   state reports: {events['state']}, activity: {report['activity']}")
    log("   topics seen: " + (", ".join(f"{t} ({n})" for t, n in sorted(topics.items())) or "none"))
    log(f"   position in state reports: {events['pose_state']}, RRTP positions: {events['pose_rrtp']}")
    if answered:
        log("   position requests answered for: " + "; ".join(sorted(answered)))
    else:
        log(f"   position requests (RRTP) unanswered in all {len(variants)} variants tried")
    if candidates:
        log("   fields that changed while listening (most position-like first):")
        for c in candidates[:8]:
            log(f"     {c['path']:45} {c['changes']:3} changes, latest {c['sample'][-3:]}  [{c['topic']}]")
    else:
        log("   no numeric field changed while listening")
    if dump_path:
        log(f"   every message was saved to {os.path.abspath(dump_path)} (network details removed)")

    if events["pose_rrtp"] and "topic=req conType=local" not in answered:
        topic, con = variant_flags[sorted(answered)[0]]
        flags = (f" --rrtp-topic '{topic}'" if topic != "req" else "") + \
                (f" --rrtp-contype {con}" if con != "local" else "")
        report["suggested_flags"] = flags.strip()
        log(f"   => automatic mapping will work. Start the mapper with:  serve{flags}")
    elif events["pose_state"] or events["pose_rrtp"]:
        log("   => automatic mapping will work with this robot.")
    elif suggestion:
        log(f"   => possible position field found. Try:  serve --pose-path {suggestion}")
        log("      (add --pose-units cm or m if the map comes out 10x too small/large, "
            "--pose-angle rad if turns look wrong)")
    elif report["activity"] in ("docked", "idle", "unknown"):
        if link.rrtp_supported:
            log("   => the robot answers position requests (RRTP); it reports a position once it moves.")
        log("   The robot wasn't moving - start a clean, then probe again to confirm positions.")
    else:
        log("   => no position found while cleaning. Send the saved file (check it first) so the "
            "format can be worked out; meanwhile draw the map by hand.")
    return report
