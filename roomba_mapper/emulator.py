"""Pretend to be a Wi-Fi Roomba on the network.

Wraps the simulator in the same protocol a real robot speaks - MQTT over
TLS with BLID/password login, state reports on the shadow topic, RRTP
position replies on ``data``, commands on ``cmd``, UDP discovery and the
"hold HOME" password handshake - so the whole Wi-Fi code path can be run
and tested without hardware:

    python -m roomba_mapper emulate          # terminal 1
    python -m roomba_mapper serve --robot wifi --ip 127.0.0.1 \
        --blid EMU0001 --password emulator    # terminal 2
"""

import json
import math
import os
import socket
import ssl
import subprocess
import tempfile
import threading
import time

from .mqtt_lite import MiniBroker
from .simulator import SimRoomba
from .wifi import PASSWORD_REQUEST, deep_merge


def self_signed_context(workdir=None):
    """Server TLS context with a throwaway self-signed certificate (needs openssl)."""
    workdir = workdir or tempfile.mkdtemp(prefix="roomba-emu-")
    key, cert = os.path.join(workdir, "key.pem"), os.path.join(workdir, "cert.pem")
    if not os.path.exists(cert):
        subprocess.run(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "30",
             "-subj", "/CN=roomba-emulator", "-keyout", key, "-out", cert],
            check=True, capture_output=True)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert, key)
    return ctx


class RoombaEmulator:
    def __init__(self, host="127.0.0.1", port=0, blid="EMU0001", password="emulator",
                 sku="Y011040", pose_mode="rrtp", tls=True, time_scale=10.0, robot=None,
                 discovery_port=None, tick=0.1, rrtp_topic="req", pose_field="cleanMissionStatus.pos"):
        """pose_mode: 'rrtp' (answer position requests on `rrtp_topic`), 'state'
        (pose in state reports, like a Roomba 980), 'field' (position under an
        unusual state field, `pose_field`, to exercise discovery of unknown
        formats) or 'none' (no position at all)."""
        self.blid, self.password, self.sku = blid, password, sku
        self.pose_mode = pose_mode
        self.rrtp_topic = rrtp_topic
        self.pose_field = pose_field
        self.robot = robot or SimRoomba()
        self.time_scale = time_scale
        self.tick = tick
        self.commands = []
        self.ssl_context = self_signed_context() if tls else None
        self.broker = MiniBroker(host, port, username=blid, password=password,
                                 ssl_context=self.ssl_context)
        self.broker.on_publish = self._on_publish
        self.port = self.broker.port
        self.host = host
        self.password_mode = False  # "HOME button held": next raw TLS client gets the password
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._udp = None
        if discovery_port is not None:
            self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp.bind((host, discovery_port))
            self.discovery_port = self._udp.getsockname()[1]
            threading.Thread(target=self._discovery_loop, daemon=True).start()
        threading.Thread(target=self._run, daemon=True).start()

    # -- protocol --

    def discovery_info(self):
        return {"ver": "4" if self.sku.startswith(("Y", "Q", "X")) else "3",
                "hostname": f"Roomba-{self.blid}", "robotname": "Emulated Roomba",
                "ip": self.host, "mac": "00:00:00:00:00:00", "sw": "emulator+1.0",
                "sku": self.sku, "nc": 0, "proto": "mqtt", "cap": {"pose": 1}}

    def _discovery_loop(self):
        while not self._stop.is_set():
            try:
                data, addr = self._udp.recvfrom(1024)
            except OSError:
                return
            if data.strip() == b"irobotmcs":
                self._udp.sendto(json.dumps(self.discovery_info()).encode(), addr)

    def _on_publish(self, topic, payload):
        try:
            msg = json.loads(payload.decode() or "{}")
        except ValueError:
            return
        if topic == "cmd":
            self.commands.append(msg)
            with self._lock:
                self.robot.command(msg.get("command"), msg.get("params"))
        elif topic == self.rrtp_topic and self.pose_mode == "rrtp":
            with self._lock:
                moving = self.robot.phase not in ("charge",)
                x, y, th = self.robot.reported_pose()
            coords = ([{"type": "current", "xyt": [round(x, 3), round(y, 3), round(th, 3)],
                        "ts": int(time.time())}] if moving else [{"type": "unknown"}])
            self.broker.send("data", json.dumps({
                "reportType": "current", "reqId": msg.get("reqId"), "ver": "1.0.0",
                "data": [{"pmap_id": "simPmap1", "pmapv_id": "v1", "coords": coords}]}))
        elif topic.endswith("/shadow/get"):
            self._publish_state()

    def _publish_state(self, extra=None):
        with self._lock:
            reported = self.robot.reported()
        if extra:
            deep_merge(reported, extra)
        self.broker.send(f"$aws/things/{self.blid}/shadow/update",
                         json.dumps({"state": {"reported": reported}}))

    def _run(self):
        last_version = -1
        since_pose = 0.0
        while not self._stop.is_set():
            with self._lock:
                self.robot.step(self.tick)
                version = self.robot.version
                moving = self.robot.phase not in ("charge", "stop")
            if version != last_version:
                last_version = version
                self._publish_state()
            since_pose += self.tick
            if self.pose_mode in ("state", "field") and moving and since_pose >= 1.0:
                since_pose = 0.0
                with self._lock:
                    x, y, th = self.robot.reported_pose()
                pose = {"theta": int(round(math.degrees(th))),
                        "point": {"x": int(x * 1000), "y": int(y * 1000)}}
                if self.pose_mode == "state":
                    self._publish_state({"pose": pose})
                else:
                    extra = {"signal": {"rssi": -45 - int(time.time() * 7) % 9}}
                    node = extra
                    parts = self.pose_field.split(".")
                    for part in parts[:-1]:
                        node = node.setdefault(part, {})
                    node[parts[-1]] = pose
                    self._publish_state(extra)
            self._stop.wait(self.tick / self.time_scale)

    # -- password handshake (separate listener, as on a real robot in "HOME held" mode) --

    def serve_password_once(self, timeout=10.0):
        """Answer one password request on a fresh TLS port; returns that port."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind((self.host, 0))
        srv.listen(1)
        srv.settimeout(timeout)
        port = srv.getsockname()[1]

        def handle():
            try:
                conn, _ = srv.accept()
                if self.ssl_context:
                    conn = self.ssl_context.wrap_socket(conn, server_side=True)
                data = conn.recv(64)
                if data == PASSWORD_REQUEST:
                    pw = self.password.encode() + b"\x00"
                    conn.sendall(bytes([0xF0, 5 + len(pw)]) + PASSWORD_REQUEST[2:] + pw)
                conn.close()
            except OSError:
                pass
            finally:
                srv.close()
        threading.Thread(target=handle, daemon=True).start()
        return port

    def close(self):
        self._stop.set()
        self.broker.close()
        if self._udp:
            self._udp.close()
