import json
import math
import os
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

from roomba_mapper import geometry as geo
from roomba_mapper.controller import MapController
from roomba_mapper.server import make_server
from roomba_mapper.simulator import SimRoomba
from roomba_mapper.store import MapStore

try:
    from helpers import DirectSimLink
except ImportError:  # run as a package
    from tests.helpers import DirectSimLink


def true_floor_area(robot):
    home = robot.home
    return sum(geo.polygon_area(p) for p in home.floor) - sum(geo.polygon_area(p) for p in home.obstacles)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, "map.json")
        self.clock = [1_000_000.0]

    def tearDown(self):
        self.dir.cleanup()

    def make(self, seed=3):
        robot = SimRoomba(seed=seed, drift_deg=1.0)
        link = DirectSimLink(robot)
        ctrl = MapController(MapStore(self.path), link, clock=lambda: self.clock[0])
        return ctrl, link, robot

    def learn(self, ctrl, link, max_runs=5):
        for _ in range(max_runs):
            link.run_mission(self.clock)
            if ctrl.map.locked:
                return
        self.fail("map never locked")

    def test_learns_locks_and_matches_the_home(self):
        ctrl, link, robot = self.make()
        self.learn(ctrl, link)
        floors = ctrl.map.polygons("floor")
        self.assertEqual(len(floors), 1)
        area = ctrl.map.floor_area()
        self.assertAlmostEqual(area, true_floor_area(robot), delta=0.15 * true_floor_area(robot))
        # the free-standing kitchen island is found as an obstacle
        island_centre = (3.1, -0.2)
        self.assertTrue(any(geo.point_in_polygon(*island_centre, p) for p in ctrl.map.polygons("obstacle")))
        self.assertIn("locked", ctrl.notice)
        self.assertGreater(ctrl.snapshot()["stats"]["percent"], 85)

    def test_locked_map_is_reference_and_corrects_drift(self):
        ctrl, link, robot = self.make()
        self.learn(ctrl, link)
        plan_before = json.dumps(ctrl.map.elements)
        self.clock[0] += 48 * 3600  # everything is due again
        self.assertEqual(ctrl.snapshot()["stats"]["percent"], 0.0)
        drift = (0.25, -0.15, math.radians(6))
        link.run_mission(self.clock, drift=drift)
        run = ctrl.map.runs[-1]
        self.assertEqual(json.dumps(ctrl.map.elements), plan_before)  # reference not reshaped
        self.assertGreater(run["align_score"], 0.9)
        for px, py in [(0.5, 0.5), (4.0, 2.0), (4.5, 5.0)]:
            rx, ry = geo.transform(px, py, drift)
            mx, my = geo.transform(rx, ry, run["offset"])
            self.assertLess(math.hypot(mx - px, my - py), 0.2)
        self.assertGreater(ctrl.snapshot()["stats"]["percent"], 80)

    def test_map_persists_across_restart(self):
        ctrl, link, _ = self.make()
        self.learn(ctrl, link)
        ctrl.add_shape("nogo", [[1, 0], [1.5, 0], [1.5, 0.5], [1, 0.5]], name="Pet bowls")
        ctrl.save(force=True)
        ctrl2, _, _ = self.make()
        self.assertTrue(ctrl2.map.locked)
        self.assertEqual(ctrl2.map.elements, ctrl.map.elements)
        self.assertIn("reference locked", ctrl2.notice)

    def test_manual_map_without_any_runs(self):
        ctrl, link, _ = self.make()
        with self.assertRaises(ValueError):
            ctrl.set_locked(True)
        ctrl.add_shape("floor", [[-0.2, -1.5], [4.8, -1.5], [4.8, 2.5], [-0.2, 2.5]], name="Living room")
        ctrl.set_locked(True)
        link.run_mission(self.clock)
        self.assertEqual([el["source"] for el in ctrl.map.elements], ["manual"])
        self.assertGreater(ctrl.snapshot()["stats"]["percent"], 50)

    def test_unlock_relearns(self):
        ctrl, link, _ = self.make()
        self.learn(ctrl, link)
        ctrl.set_locked(False)
        ctrl.settings["auto_lock"] = False
        link.run_mission(self.clock)
        self.assertFalse(ctrl.map.locked)
        self.assertIn("growth", ctrl.map.runs[-1])

    def test_pose_only_robot_runs_are_detected(self):
        ctrl, link, robot = self.make()
        link.on_state = lambda reported: None  # robot that only reports positions
        robot.command("start")
        for _ in range(200):
            robot.step(1.0)
            self.clock[0] += 1.0
            ctrl._on_pose(*robot.reported_pose(), "rrtp")
        self.assertIsNotNone(ctrl.run)
        self.clock[0] += 200
        with ctrl.lock:
            ctrl._end_run("no position updates")
        self.assertIsNone(ctrl.run)
        self.assertTrue(ctrl.map.has_floor_plan())

    def test_auto_start(self):
        ctrl, link, robot = self.make()
        self.learn(ctrl, link)
        ctrl.update_settings({"auto_start_percent": 30, "auto_start_min_hours": 6})
        self.assertFalse(ctrl.check_auto_start())       # just cleaned
        self.clock[0] += 25 * 3600                       # cleaning has gone stale
        robot.battery = 50.0
        ctrl._on_state(robot.reported())
        self.assertFalse(ctrl.check_auto_start())       # not with a low battery
        robot.battery = 100.0
        ctrl._on_state(robot.reported())
        self.assertTrue(ctrl.check_auto_start())
        self.assertEqual(link.sent[-1][0], "start")

    def test_commands_and_mop_mode(self):
        ctrl, link, _ = self.make()
        ctrl.command("clean", "mop")
        self.assertEqual(link.sent[-1], ("start", {"operatingMode": 6,
                                                   "padWetness": {"disposable": 2, "reusable": 2}}))
        ctrl.command("dock")
        with self.assertRaises(ValueError):
            ctrl.command("selfdestruct")

    def test_pmap_change_warns(self):
        ctrl, link, robot = self.make()
        self.learn(ctrl, link)
        ctrl._on_state(dict(robot.reported(), pmaps=[{"otherMap": "v9"}]))
        self.assertIn("rebuilt its own internal map", ctrl.notice)

    def test_settings_validation(self):
        ctrl, _, _ = self.make()
        for bad in ({"stale_hours": -1}, {"mop_wetness": 7}, {"nope": 1}, {"stale_hours": float("nan")}):
            with self.assertRaises(ValueError):
                ctrl.update_settings(bad)
        ctrl.update_settings({"view_rotation": 90, "view_mirror": True})
        self.assertEqual(ctrl.settings["view_rotation"], 90.0)


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.robot = SimRoomba(seed=3)
        self.link = DirectSimLink(self.robot)
        self.ctrl = MapController(MapStore(os.path.join(self.dir.name, "m.json")), self.link)
        self.server = make_server(self.ctrl, "127.0.0.1", 0, pin="4321")
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.dir.cleanup()

    def request(self, path, body=None, pin="4321"):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.base + path, data=data, headers={"X-Pin": pin})
        if data:
            req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=10) as res:
                raw = res.read()
                return res.status, (json.loads(raw) if raw[:1] in b"{[" else raw)
        except urllib.error.HTTPError as err:
            return err.code, json.loads(err.read() or b"{}")

    def test_serves_gui(self):
        status, body = self.request("/")
        self.assertEqual(status, 200)
        self.assertIn(b"Roomba Map", body)
        for f in ("/app.js", "/style.css", "/manifest.webmanifest", "/icon.svg"):
            self.assertEqual(self.request(f)[0], 200, f)
        self.assertEqual(self.request("/../server.py")[0], 404)

    def test_pin(self):
        self.assertEqual(self.request("/api/state", pin="0")[0], 401)
        self.assertEqual(self.request("/api/export", pin="")[0], 401)
        self.assertEqual(self.request("/api/command", {"command": "clean"}, pin="")[0], 401)
        self.assertEqual(self.link.sent, [])
        self.assertEqual(self.request("/api/info", pin="")[0], 200)

    def test_state_sends_heavy_parts_only_when_changed(self):
        _, s = self.request("/api/state")
        self.assertIn("elements", s)
        self.assertIn("coverage", s)
        _, s2 = self.request(f"/api/state?map={s['map_version']}&cov={s['cov_version']}")
        self.assertNotIn("elements", s2)
        self.assertNotIn("coverage", s2)

    def test_shape_editing_flow(self):
        status, s = self.request("/api/shapes", {"kind": "floor", "points": [[0, 0], [4, 0], [4, 3], [0, 3]],
                                                 "name": "Kitchen"})
        self.assertEqual(status, 200)
        new_id = s["created_id"]
        self.assertEqual(s["elements"][0]["name"], "Kitchen")
        status, s = self.request("/api/shapes/update", {"id": new_id, "points": [[0, 0], [5, 0], [5, 3], [0, 3]]})
        self.assertEqual(status, 200)
        self.assertEqual(s["elements"][0]["points"][1], [5.0, 0.0])
        self.assertEqual(self.request("/api/map", {"action": "lock"})[1]["locked"], True)
        status, geo_json = self.request("/api/export")
        self.assertEqual(status, 200)
        self.assertEqual(geo_json["features"][0]["properties"]["kind"], "floor")
        self.assertEqual(len(geo_json["features"][0]["geometry"]["coordinates"][0]), 5)
        status, s = self.request("/api/shapes/delete", {"id": new_id})
        self.assertEqual(s["elements"], [])

    def test_commands(self):
        self.assertEqual(self.request("/api/command", {"command": "clean", "mode": "vacuum"})[0], 200)
        self.assertEqual(self.link.sent[-1], ("start", {"operatingMode": 2}))

    def test_bad_input(self):
        cases = [
            ("/api/command", {"command": "fly"}),
            ("/api/shapes", {"kind": "floor", "points": [[0, 0]]}),
            ("/api/shapes", {"kind": "lava", "points": [[0, 0], [1, 0], [0, 1]]}),
            ("/api/shapes/update", {"id": "x"}),
            ("/api/shapes/delete", {"id": 999}),
            ("/api/map", {"action": "explode"}),
            ("/api/map", {"action": "lock"}),  # nothing to lock yet
            ("/api/settings", {"stale_hours": None}),
        ]
        for path, body in cases:
            self.assertEqual(self.request(path, body)[0], 400, (path, body))
        self.assertEqual(self.request("/api/nothing", {})[0], 404)


if __name__ == "__main__":
    unittest.main()
