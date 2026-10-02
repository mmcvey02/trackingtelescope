"""Read your own robot's maps and cleaning history from your iRobot account.

Newer Roombas (such as the Combo Essential, RVG-Y1) keep their position to
themselves on the local network but upload maps to iRobot after each clean.
This module logs in the same way the iRobot app does and asks iRobot's
servers for that data:

    login  ->  temporary Amazon (AWS) credentials issued by iRobot
           ->  signed requests to iRobot's REST API (AWS Signature V4)
           ->  /v1/p2maps (maps)  and  /v1/<robot>/missionhistory (runs)

The API is private and undocumented; endpoints and formats come from the
community project roombapy-prime and may change with app updates. Your
password goes only to iRobot's login service, and no tokens are saved.
"""

import datetime
import gzip
import hashlib
import hmac
import io
import json
import os
import tarfile
import urllib.error
import urllib.parse
import urllib.request

DISCOVERY_URL = "https://disc-prod.iot.irobotapi.com/v1/discover/endpoints?country_code={cc}"
APP_ID = "ANDROID-C7FB240E-DF34-42D7-AE4E-A8C17079A294"
AWS_USER_AGENT = "aws-sdk-iOS/2.27.6 iOS/18.0.1 en_US"
APP_USER_AGENT = "iRobot/7.16.2.140449 CFNetwork/1568.100.1.2.1 Darwin/24.0.0"

# keys removed from anything written to disk
SECRET_KEYS = ("password", "token", "signature", "secret", "credentials", "accesskey",
               "sessiontoken", "cognito", "email", "uid", "loginid", "map_url", "url",
               "ssid", "mac", "addr", "wlcfg", "netinfo", "phone", "address")


class CloudError(Exception):
    pass


# -- AWS Signature Version 4 (standard library) -------------------------------------


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def _hmac(key, msg):
    return hmac.new(key, msg.encode(), hashlib.sha256).digest()


def sigv4_sign(method, url, access_key, secret_key, region, service, body=b"",
               headers=None, session_token=None, now=None):
    """Return request headers carrying an AWS SigV4 signature.

    `headers` (lower-case names) are included in the signature together with
    host and x-amz-date. A session token is attached unsigned, as the
    iRobot app does.
    """
    now = now or datetime.datetime.now(datetime.timezone.utc)
    amz_date = now.strftime("%Y%m%dT%H%M%SZ")
    date_stamp = amz_date[:8]
    parsed = urllib.parse.urlsplit(url)
    path = parsed.path or "/"
    canonical_uri = urllib.parse.quote(path, safe="/~")
    query = urllib.parse.parse_qsl(parsed.query, keep_blank_values=True)
    canonical_qs = "&".join(
        f"{urllib.parse.quote(k, safe='~')}={urllib.parse.quote(v, safe='~')}"
        for k, v in sorted(query))
    signed = {k.lower(): str(v).strip() for k, v in (headers or {}).items()}
    signed["host"] = parsed.netloc
    signed["x-amz-date"] = amz_date
    names = sorted(signed)
    canonical_headers = "".join(f"{k}:{signed[k]}\n" for k in names)
    signed_headers = ";".join(names)
    canonical_request = "\n".join([method.upper(), canonical_uri, canonical_qs,
                                   canonical_headers, signed_headers, _sha256(body or b"")])
    scope = f"{date_stamp}/{region}/{service}/aws4_request"
    string_to_sign = "\n".join(["AWS4-HMAC-SHA256", amz_date, scope,
                                _sha256(canonical_request.encode())])
    key = _hmac(("AWS4" + secret_key).encode(), date_stamp)
    for part in (region, service, "aws4_request"):
        key = _hmac(key, part)
    signature = hmac.new(key, string_to_sign.encode(), hashlib.sha256).hexdigest()
    out = dict(signed)
    out["authorization"] = (f"AWS4-HMAC-SHA256 Credential={access_key}/{scope}, "
                            f"SignedHeaders={signed_headers}, Signature={signature}")
    if session_token:
        out["x-amz-security-token"] = session_token
    out.pop("host")  # urllib adds it
    return out


# -- HTTP ------------------------------------------------------------------------------


def _request(method, url, headers=None, body=None, opener=None, timeout=30):
    """Returns (status, body_bytes); HTTP errors are returned, not raised."""
    opener = opener or urllib.request.urlopen
    req = urllib.request.Request(url, data=body, headers=headers or {}, method=method)
    try:
        with opener(req, timeout=timeout) as res:
            return res.status, res.read()
    except urllib.error.HTTPError as err:
        return err.code, err.read() or b""


def _json_or_none(data):
    try:
        return json.loads(data.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None


def redact(obj):
    if isinstance(obj, dict):
        return {k: ("<redacted>" if any(s in str(k).lower() for s in SECRET_KEYS) else redact(v))
                for k, v in obj.items()}
    if isinstance(obj, list):
        return [redact(v) for v in obj]
    return obj


# -- session -------------------------------------------------------------------------------


class IRobotCloud:
    def __init__(self, email, password, country="US", opener=None):
        self.email, self._password, self.country = email, password, country
        self.opener = opener
        self.disc = None
        self.robots = {}
        self.creds = None
        self.region = None

    def _json(self, method, url, data=None, form=False, headers=None):
        # same headers as `get-password --cloud`, which is known to log in fine
        hdrs = {"Accept": "application/json", "User-Agent": "roomba-mapper"}
        hdrs.update(headers or {})
        body = None
        if data is not None:
            if form:
                body = urllib.parse.urlencode(data).encode()
                hdrs["Content-Type"] = "application/x-www-form-urlencoded"
            else:
                body = json.dumps(data).encode()
                hdrs["Content-Type"] = "application/json"
        status, raw = _request(method, url, hdrs, body, self.opener)
        parsed = _json_or_none(raw)
        if status >= 400 or parsed is None:
            raise CloudError(f"{url.split('?')[0]} answered HTTP {status}")
        return parsed

    def login(self):
        self.disc = self._json("GET", DISCOVERY_URL.format(cc=urllib.parse.quote(self.country)))
        gigya = self.disc["gigya"]
        dep = self.disc["deployments"][self.disc["current_deployment"]]
        g = self._json("POST", f"https://accounts.{gigya['datacenter_domain']}/accounts.login",
                       {"apiKey": gigya["api_key"], "loginID": self.email, "password": self._password,
                        "targetEnv": "mobile", "format": "json"}, form=True)
        if g.get("errorCode"):
            raise CloudError(f"iRobot login failed: {g.get('errorDetails') or g.get('errorMessage')}")
        res = self._json("POST", f"{dep['httpBase']}/v2/login", {
            "app_id": APP_ID,
            "assume_robot_ownership": "0",
            "gigya": {"signature": g["UIDSignature"], "timestamp": g["signatureTimestamp"],
                      "uid": g["UID"]},
        })
        self._password = None
        self.robots = res.get("robots") or {}
        self.creds = res.get("credentials")
        if not self.creds or "AccessKeyId" not in self.creds:
            raise CloudError("login worked but iRobot did not issue API credentials")
        cognito = self.creds.get("CognitoId") or ""
        self.region = cognito.split(":")[0] if ":" in cognito else dep.get("awsRegion", "us-east-1")
        return self.robots

    def base_for(self, blid):
        """REST host for the deployment the robot belongs to."""
        deps = self.disc["deployments"]
        svc = (self.robots.get(blid) or {}).get("svcDeplId")
        dep = deps.get(svc) or deps[self.disc["current_deployment"]]
        return dep.get("httpBaseAuth") or dep["httpBase"].replace("unauth", "auth")

    def get(self, blid, path, query=None):
        """Signed GET; returns (status, parsed_json_or_bytes)."""
        url = self.base_for(blid) + path
        if query:
            url += "?" + urllib.parse.urlencode(query)
        headers = sigv4_sign("GET", url, self.creds["AccessKeyId"], self.creds["SecretKey"],
                             self.region, "execute-api",
                             headers={"accept": "application/json",
                                      "content-type": "application/json",
                                      "user-agent": AWS_USER_AGENT},
                             session_token=self.creds.get("SessionToken"))
        status, raw = _request("GET", url, headers, None, self.opener)
        parsed = _json_or_none(raw)
        return status, (parsed if parsed is not None else raw)

    def download(self, url):
        """Plain GET for pre-signed download links (they carry their own auth)."""
        status, raw = _request("GET", url, {"User-Agent": APP_USER_AGENT}, None, self.opener)
        if status >= 400:
            raise CloudError(f"download answered HTTP {status}")
        return raw


# -- map bundles ---------------------------------------------------------------------------


def unpack_bundle(data):
    """A map download is a tar.gz of GeoJSON files -> {name: parsed_json}."""
    out = {}
    try:
        tar = tarfile.open(fileobj=io.BytesIO(data), mode="r:*")
    except tarfile.TarError:
        try:
            return {"map": json.loads(gzip.decompress(data))}
        except (OSError, ValueError):
            parsed = _json_or_none(data)
            return {"map": parsed} if parsed is not None else {}
    with tar:
        for member in tar.getmembers():
            if not member.isfile():
                continue
            name = member.name.rsplit("/", 1)[-1]
            name = name.rsplit(".", 1)[0] if "." in name else name
            raw = tar.extractfile(member).read()
            parsed = _json_or_none(raw)
            if parsed is not None:
                out[name] = parsed
    return out


def _coords(geom):
    if not isinstance(geom, dict):
        return
    if geom.get("type") == "GeometryCollection":
        for g in geom.get("geometries", []):
            yield from _coords(g)
        return

    def walk(c):
        if isinstance(c, (list, tuple)) and len(c) >= 2 and all(isinstance(v, (int, float)) for v in c[:2]):
            yield float(c[0]), float(c[1])
        elif isinstance(c, (list, tuple)):
            for item in c:
                yield from walk(item)
    yield from walk(geom.get("coordinates"))


def summarize_geojson(doc):
    """Feature count, geometry types, property names and coordinate range of a GeoJSON doc."""
    feats = doc.get("features") if isinstance(doc, dict) else None
    if feats is None and isinstance(doc, dict) and doc.get("type") == "Feature":
        feats = [doc]
    if not isinstance(feats, list):
        return None
    feats = [f for f in feats if isinstance(f, dict) and "geometry" in f]
    if not feats:
        return None
    types, props = {}, set()
    xs, ys, points = [], [], 0
    for f in feats:
        geom = f.get("geometry") or {}
        gtype = str(geom.get("type"))
        types[gtype] = types.get(gtype, 0) + 1
        props.update((f.get("properties") or {}).keys())
        for x, y in _coords(geom):
            xs.append(x)
            ys.append(y)
            points += 1
    summary = {"features": len(feats), "geometry": types, "properties": sorted(props), "points": points}
    if xs:
        summary["x_range"] = [round(min(xs), 3), round(max(xs), 3)]
        summary["y_range"] = [round(min(ys), 3), round(max(ys), 3)]
    return summary


# -- the probe -----------------------------------------------------------------------------


def _save(out_dir, name, obj):
    path = os.path.join(out_dir, name)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(redact(obj), fh, indent=1)
    return path


def _describe(body):
    if isinstance(body, list):
        keys = sorted(body[0].keys()) if body and isinstance(body[0], dict) else []
        return f"list of {len(body)}" + (f", fields: {', '.join(keys[:14])}" if keys else "")
    if isinstance(body, dict):
        return "fields: " + ", ".join(sorted(body.keys())[:14])
    return f"{len(body)} bytes" if isinstance(body, (bytes, bytearray)) else str(body)[:80]


def cloud_probe(email, password, country="US", blid=None, out_dir="roomba_cloud", log=print,
                opener=None, max_maps=3):
    """Ask iRobot's servers what they hold for one robot; saves everything (redacted)."""
    os.makedirs(out_dir, exist_ok=True)
    report = {"endpoints": {}, "maps": [], "features": {}}
    cloud = IRobotCloud(email, password, country, opener)
    log("1. Logging in to your iRobot account")
    robots = cloud.login()
    for b, info in robots.items():
        log(f"   robot {info.get('name')!r} sku={info.get('sku')} service={info.get('svcDeplId')}"
            f" firmware={info.get('softwareVer')}")
    if not robots:
        raise CloudError("the account has no robots")
    if blid not in robots:
        blid = next(iter(robots))
    report["robot"] = {"sku": robots[blid].get("sku"), "service": robots[blid].get("svcDeplId")}
    log(f"   ok - using API host {urllib.parse.urlsplit(cloud.base_for(blid)).netloc} "
        f"(region {cloud.region})")

    def call(label, path, query=None, save_as=None):
        status, body = cloud.get(blid, path, query)
        ok = status < 400
        report["endpoints"][label] = status
        log(f"   {label:28} HTTP {status}  " + (_describe(body) if ok else ""))
        if ok and save_as and not isinstance(body, (bytes, bytearray)):
            _save(out_dir, save_as, body)
        return body if ok else None

    log("2. Maps")
    maps = call("maps (p2maps)", "/v1/p2maps", {"robotId": blid, "visible": "true"}, "p2maps.json")
    if isinstance(maps, dict):
        maps = maps.get("items") or maps.get("p2maps") or maps.get("maps") or []
    for m in (maps or [])[:max_maps]:
        map_id, ver = m.get("p2map_id"), m.get("active_p2mapv_id")
        if not map_id:
            continue
        log(f"   map {map_id!s:.12}  name={m.get('name')!r} state={m.get('state')} version={ver!s:.12}")
        call("  map details", f"/v1/p2maps/{map_id}", save_as=f"map_{map_id}.json")
        if not ver:
            continue
        call("  map version", f"/v1/p2maps/{map_id}/versions/{ver}", save_as=f"map_{map_id}_version.json")
        link = call("  map download link", f"/v1/p2maps/{map_id}/versions/{ver}/geojson",
                    {"response_type": "link"})
        url = (link or {}).get("map_url") if isinstance(link, dict) else None
        if not url:
            continue
        try:
            files = unpack_bundle(cloud.download(url))
        except CloudError as exc:
            log(f"   download failed: {exc}")
            continue
        bundle_dir = os.path.join(out_dir, f"map_{map_id}")
        os.makedirs(bundle_dir, exist_ok=True)
        summary = {}
        for name, doc in files.items():
            _save(bundle_dir, name + ".json", doc)
            s = summarize_geojson(doc)
            if s:
                summary[name] = s
                report["features"][name] = report["features"].get(name, 0) + s["features"]
        report["maps"].append({"id": map_id, "files": sorted(files), "summary": summary})
        log(f"   downloaded {len(files)} map files to {bundle_dir}:")
        for name in sorted(files):
            s = summary.get(name)
            if s:
                rng = f" x {s.get('x_range')} y {s.get('y_range')}" if "x_range" in s else ""
                log(f"     {name:16} {s['features']:4} features {s['geometry']}{rng}")
            else:
                log(f"     {name:16} (metadata)")

    log("3. Cleaning history")
    hist = call("mission history", f"/v1/{blid}/missionhistory",
                {"app_id": APP_ID, "filterType": "omit_quickly_canceled_not_scheduled",
                 "supportedDoneCodes": "dndEnd,returnHomeEnd", "maxReports": "10"},
                "missionhistory.json")
    if hist is None:
        call("mission history (simple)", f"/v1/{blid}/missionhistory", None, "missionhistory_simple.json")

    log("4. Older map service (for comparison)")
    call("pmaps (classic)", f"/v1/{blid}/pmaps", {"visible": "true", "activeDetails": "2"}, "pmaps.json")
    call("robot record", "/v1/robots", {"robot_id": blid}, "robot.json")

    has_paths = any(k in report["features"] for k in ("trajectories", "coverage"))
    log("")
    if has_paths:
        log("=> The cloud holds the robot's paths/coverage. The mapper can import these after each clean.")
    elif report["maps"]:
        log("=> Maps download, but without paths or coverage. Room outlines and the dock can still be imported.")
    elif any(s < 400 for s in report["endpoints"].values()):
        log("=> Logged in and some data is available, but no map was returned for this robot.")
    else:
        log("=> iRobot refused every request for this robot's data.")
    log(f"   Everything received was saved (passwords, tokens and links removed) in {os.path.abspath(out_dir)}")
    _save(out_dir, "summary.json", report)
    return report
