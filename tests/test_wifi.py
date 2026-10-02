import io
import json
import math
import os
import tempfile
import shutil
import threading
import time
import unittest

from roomba_mapper import wifi
from roomba_mapper.emulator import RoombaEmulator
from roomba_mapper.mqtt_lite import MiniBroker, MQTTClient, MQTTError, topic_matches

HAVE_OPENSSL = shutil.which("openssl") is not None


def wait_for(cond, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


class MQTTTests(unittest.TestCase):
    def test_pubsub_auth_and_single_connection(self):
        broker = MiniBroker(username="blid", password="pw")
        got = []
        broker.on_publish = lambda t, p: (got.append((t, p)), broker.send("data", b"echo:" + p))
        c = MQTTClient("127.0.0.1", broker.port, "blid", "blid", "pw", tls=False, keepalive=2)
        msgs = []
        c.on_message = lambda t, p: msgs.append((t, p))
        try:
            c.connect()
            c.subscribe("#")
            time.sleep(0.2)
            c.publish("req", "hi")
            c.publish("cmd", b"x", qos=1)
            self.assertTrue(wait_for(lambda: len(msgs) == 2))
            self.assertEqual(got, [("req", b"hi"), ("cmd", b"x")])
            with self.assertRaises(MQTTError):
                MQTTClient("127.0.0.1", broker.port, "z", "blid", "pw", tls=False).connect()
            time.sleep(2.5)  # survives keep-alive pings
            self.assertTrue(c.connected)
        finally:
            c.close()
            broker.close()

    def test_bad_password(self):
        broker = MiniBroker(username="blid", password="pw")
        try:
            with self.assertRaisesRegex(MQTTError, "bad username or password"):
                MQTTClient("127.0.0.1", broker.port, "blid", "blid", "nope", tls=False).connect()
        finally:
            broker.close()

    def test_topic_matching(self):
        self.assertTrue(topic_matches("#", "a/b"))
        self.assertTrue(topic_matches("$aws/things/X/#", "$aws/things/X/shadow/update"))
        self.assertTrue(topic_matches("a/+/c", "a/b/c"))
        self.assertFalse(topic_matches("a/+", "a/b/c"))


class ParsingTests(unittest.TestCase):
    def test_discovery_with_length_prefix(self):
        body = json.dumps({"hostname": "Roomba-ABC123", "robotname": "Rosie", "sku": "Y011040",
                           "sw": "x+1", "ver": "4", "ip": "192.168.1.9"}).encode()
        info = wifi.parse_discovery(len(body).to_bytes(2, "big") + body)
        self.assertEqual(info["blid"], "ABC123")
        self.assertEqual(info["protocol_version"], "4")
        self.assertEqual(wifi.profile_for(info["sku"])["key"], "combo-essential")

    def test_profiles(self):
        self.assertEqual(wifi.profile_for(key="RVG-Y1")["key"], "combo-essential")
        self.assertEqual(wifi.profile_for("Y014020")["key"], "combo-essential")  # a real unit's SKU
        self.assertFalse(wifi.profile_for("Y014020")["mode_select"])
        self.assertTrue(wifi.profile_for("Y011040")["mop"])
        self.assertFalse(wifi.profile_for("Q012020")["mop"])
        self.assertEqual(wifi.profile_for("R980020")["key"], "900")
        self.assertEqual(wifi.profile_for("i755020")["key"], "classic")
        with self.assertRaises(ValueError):
            wifi.profile_for(key="toaster")

    def test_pose_parsing(self):
        x, y, th = wifi.pose_from_reported({"pose": {"theta": 90, "point": {"x": 1500, "y": -250}}})
        self.assertAlmostEqual(x, 1.5)
        self.assertAlmostEqual(y, -0.25)
        self.assertAlmostEqual(th, 1.5708, places=3)
        x, _, _ = wifi.pose_from_reported({"pose": {"theta": 0, "point": {"x": 150, "y": 0}}}, "cm")
        self.assertAlmostEqual(x, 1.5)
        self.assertIsNone(wifi.pose_from_reported({"pose": "?"}))

    def test_rrtp_parsing(self):
        a = {"reportType": "current", "data": [{"pmap_id": "P", "coords": [
            {"type": "current", "xyt": [1.2, -0.5, 3.0], "ts": 1}]}]}
        self.assertEqual(wifi.pose_from_rrtp(a), (1.2, -0.5, 3.0, "P"))
        b = {"reportType": "current", "data": [{"pmap_id": "P", "coords": [0.1, 0.2, 0.3], "ts": 1}]}
        self.assertEqual(wifi.pose_from_rrtp(b), (0.1, 0.2, 0.3, "P"))
        c = {"reportType": "current", "data": [{"coords": [{"type": "unknown"}]}]}
        self.assertIsNone(wifi.pose_from_rrtp(c))

    def test_activity(self):
        a = lambda phase, cycle="clean": wifi.activity_from(
            {"cleanMissionStatus": {"phase": phase, "cycle": cycle}})
        self.assertEqual(a("run"), "cleaning")
        self.assertEqual(a("hmPostMsn"), "returning")
        self.assertEqual(a("charge", "none"), "docked")
        self.assertEqual(a("stop"), "paused")
        self.assertEqual(a("stop", "none"), "idle")
        self.assertEqual(a("stuck"), "stuck")
        self.assertEqual(wifi.activity_from({}), "unknown")

    def test_pose_from_any_shape(self):
        self.assertEqual(wifi.resolve_path({"a": {"b": [5, {"c": 7}]}}, "a.b.1.c"), 7)
        self.assertIsNone(wifi.resolve_path({"a": 1}, "a.b"))
        x, y, th = wifi.pose_from_value({"point": {"x": 1000, "y": 2000}, "theta": 180})
        self.assertEqual((x, y), (1.0, 2.0))
        self.assertAlmostEqual(th, math.pi)
        x, y, th = wifi.pose_from_value({"x": 1.5, "y": -2, "t": 0.5}, units="m", angle="rad")
        self.assertEqual((x, y, th), (1.5, -2.0, 0.5))
        self.assertEqual(wifi.pose_from_value([10, 20], units="cm")[:2], (0.1, 0.2))
        self.assertIsNone(wifi.pose_from_value("north"))
        self.assertIsNone(wifi.pose_from_value({"x": float("nan"), "y": 0}))

    def test_position_candidates_rank_position_first(self):
        msgs = []
        for k in range(30):
            msgs.append(("$aws/things/X/shadow/update", {
                "batPct": 100 - k // 3,
                "cleanMissionStatus": {"mssnM": k // 10, "where": {"x": int(1000 * math.sin(k / 3)),
                                                                   "y": int(800 * math.cos(k / 4))}},
                "counter": k,
                "signal": {"rssi": -40 - k % 5},
            }))
        cands = wifi.position_candidates(msgs)
        self.assertEqual({c["path"] for c in cands[:2]},
                         {"cleanMissionStatus.where.x", "cleanMissionStatus.where.y"})
        self.assertFalse(any(c["path"] in ("batPct", "signal.rssi", "cleanMissionStatus.mssnM")
                             for c in cands))
        self.assertEqual(wifi.suggest_pose_path(cands), "cleanMissionStatus.where")

    def test_redact(self):
        out = wifi.redact({"netinfo": {"addr": 1}, "wlcfg": {"ssid": "home"}, "mac": "aa",
                           "batPct": 50, "list": [{"bssid": "x"}]})
        self.assertEqual(out["netinfo"], "<redacted>")
        self.assertEqual(out["wlcfg"], "<redacted>")
        self.assertEqual(out["mac"], "<redacted>")
        self.assertEqual(out["batPct"], 50)
        self.assertEqual(out["list"][0]["bssid"], "<redacted>")

    def test_mop_params(self):
        self.assertEqual(wifi.mop_params("vacuum"), {"operatingMode": 2})
        self.assertEqual(wifi.mop_params("mop", 9)["padWetness"], {"disposable": 3, "reusable": 3})
        self.assertIsNone(wifi.mop_params(None))

    def test_cloud_credentials(self):
        calls = []

        class Resp(io.BytesIO):
            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

        def opener(req, timeout=None):
            calls.append(req.full_url)
            if "discover/endpoints" in req.full_url:
                body = {"gigya": {"api_key": "KEY", "datacenter_domain": "us1.gigya.com"},
                        "current_deployment": "v011",
                        "deployments": {"v011": {"httpBase": "https://unauth.example"}}}
            elif "accounts.login" in req.full_url:
                self.assertIn(b"apiKey=KEY", req.data)
                body = {"UID": "u", "UIDSignature": "sig", "signatureTimestamp": "1"}
            else:
                sent = json.loads(req.data)
                self.assertEqual(sent["gigya"], {"signature": "sig", "timestamp": "1", "uid": "u"})
                body = {"robots": {"BLID1": {"name": "Rosie", "password": ":1:secret", "sku": "Y011040",
                                             "softwareVer": "v4", "svcDeplId": "v011"}}}
            return Resp(json.dumps(body).encode())

        robots = wifi.get_credentials_cloud("me@example.com", "pw", opener=opener)
        self.assertEqual(robots[0]["blid"], "BLID1")
        self.assertEqual(robots[0]["password"], ":1:secret")
        self.assertEqual(robots[0]["profile"]["key"], "combo-essential")
        self.assertEqual(len(calls), 3)


@unittest.skipUnless(HAVE_OPENSSL, "needs openssl to make a test certificate")
class EmulatorTests(unittest.TestCase):
    def connect(self, emu, **kw):
        link = wifi.WifiRoomba("127.0.0.1", emu.blid, emu.password, port=emu.port,
                               profile=wifi.profile_for(emu.sku), rrtp_interval=0.2, **kw)
        self.poses, self.states, self.links = [], [], []
        link.on_pose = lambda x, y, t, s: self.poses.append((x, y, s))
        link.on_state = lambda r: self.states.append(wifi.activity_from(r))
        link.on_link = lambda text, ok: self.links.append((text, ok))
        link.start()
        self.assertTrue(wait_for(lambda: link.connected))
        return link

    def test_rrtp_robot_end_to_end(self):
        emu = RoombaEmulator(pose_mode="rrtp", time_scale=40, discovery_port=0)
        try:
            found = wifi.discover(1.0, "127.0.0.1", emu.discovery_port)
            self.assertEqual(found[0]["blid"], emu.blid)
            self.assertEqual(found[0]["profile"]["key"], "combo-essential")
            pw_port = emu.serve_password_once()
            self.assertEqual(wifi.get_password_local("127.0.0.1", pw_port), emu.password)

            link = self.connect(emu)
            self.assertTrue(wait_for(lambda: "docked" in self.states))
            link.command("start", wifi.mop_params("mop"))
            self.assertTrue(wait_for(lambda: len(self.poses) >= 5))
            self.assertTrue(all(src == "rrtp" for _, _, src in self.poses))
            self.assertTrue(link.capabilities()["rrtp"])
            link.command("dock")
            self.assertTrue(wait_for(lambda: "returning" in self.states))
            self.assertEqual([c["command"] for c in emu.commands], ["start", "dock"])
            self.assertEqual(emu.commands[0]["params"]["operatingMode"], 6)
            link.close()
        finally:
            emu.close()

    def test_state_pose_robot(self):
        emu = RoombaEmulator(pose_mode="state", time_scale=40, sku="R980020")
        try:
            link = self.connect(emu, pose_source="auto")
            link.command("start")
            self.assertTrue(wait_for(lambda: len(self.poses) >= 3))
            self.assertTrue(all(src == "state" for _, _, src in self.poses))
            self.assertTrue(link.capabilities()["pose_from_state"])
            link.close()
        finally:
            emu.close()

    def test_wrong_password_is_reported(self):
        emu = RoombaEmulator(pose_mode="none", time_scale=1)
        try:
            link = wifi.WifiRoomba("127.0.0.1", emu.blid, "wrong", port=emu.port)
            msgs = []
            link.on_link = lambda text, ok: msgs.append(text)
            link.start()
            self.assertTrue(wait_for(lambda: msgs))
            self.assertIn("bad username or password", msgs[0])
            link.close()
        finally:
            emu.close()

    def test_probe_reports_capabilities(self):
        emu = RoombaEmulator(pose_mode="rrtp", time_scale=40, discovery_port=None)
        try:
            emu.robot.command("start")
            lines = []
            report = wifi.probe("127.0.0.1", emu.blid, emu.password, port=emu.port, listen=2.0,
                                log=lines.append)
            self.assertTrue(report["port_open"])
            self.assertEqual(report["login"], "ok")
            self.assertGreater(report["pose_rrtp"], 0)
            self.assertTrue(any("automatic mapping will work" in line for line in lines))
        finally:
            emu.close()

    def test_probe_finds_unknown_position_field_and_mapper_uses_it(self):
        emu = RoombaEmulator(pose_mode="field", time_scale=3, pose_field="cleanMissionStatus.loc")
        try:
            emu.robot.command("start")
            with tempfile.TemporaryDirectory() as d:
                dump = os.path.join(d, "dump.jsonl")
                lines = []
                report = wifi.probe("127.0.0.1", emu.blid, emu.password, port=emu.port, listen=5,
                                    log=lines.append, dump_path=dump)
                with open(dump) as fh:
                    first = json.loads(fh.readline())
                self.assertIn("topic", first)
            self.assertEqual(report["suggested_pose_path"], "cleanMissionStatus.loc")
            self.assertTrue(any("--pose-path cleanMissionStatus.loc" in line for line in lines))
            link = self.connect(emu, pose_path="cleanMissionStatus.loc")
            self.assertTrue(wait_for(lambda: len(self.poses) >= 3))
            self.assertEqual(self.poses[-1][2], "state:cleanMissionStatus.loc")
            link.close()
        finally:
            emu.close()

    def test_probe_finds_position_requests_on_other_topic(self):
        topic = "$aws/things/EMU0001/req"
        emu = RoombaEmulator(pose_mode="rrtp", time_scale=3, rrtp_topic=topic)
        try:
            emu.robot.command("start")
            report = wifi.probe("127.0.0.1", emu.blid, emu.password, port=emu.port, listen=4,
                                log=lambda _l: None)
            self.assertEqual(report["rrtp_variants_answered"], [f"topic={topic} conType=local"])
            self.assertIn("--rrtp-topic", report["suggested_flags"])
            link = self.connect(emu, rrtp_topic=topic)
            self.assertTrue(wait_for(lambda: len(self.poses) >= 3))
            link.close()
        finally:
            emu.close()

    def test_echoing_robot_without_positions_like_combo_essential(self):
        # Behaviour seen on a real RVG-Y1 (firmware congo+1.1.22): every publish is echoed
        # back, state reports arrive, no pose field, position requests go unanswered.
        emu = RoombaEmulator(pose_mode="none", time_scale=3, echo=True)
        try:
            emu.robot.command("start")
            lines = []
            report = wifi.probe("127.0.0.1", emu.blid, emu.password, port=emu.port, listen=4,
                                log=lines.append)
            self.assertEqual(report["rrtp_variants_answered"], [])
            self.assertIsNone(report["suggested_pose_path"])
            self.assertFalse(report["rrtp"])
            self.assertTrue(any("echoing" in line for line in lines))
            self.assertTrue(any("no position found while cleaning" in line for line in lines))

            link = self.connect(emu)
            link.command("pause")
            link._request_rrtp()
            time.sleep(0.5)
            for key in ("reqId", "reqType", "command", "initiator"):
                self.assertNotIn(key, link.reported)
            self.assertEqual(self.poses, [])
            link.close()
        finally:
            emu.close()

    def test_closed_port(self):
        lines = []
        import socket
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        report = wifi.probe("127.0.0.1", port=port, listen=0, log=lines.append)
        self.assertFalse(report["port_open"])


if __name__ == "__main__":
    unittest.main()
