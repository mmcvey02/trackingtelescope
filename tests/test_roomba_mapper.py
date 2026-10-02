import json
import os
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

from roomba_mapper import planner
from roomba_mapper.controller import Controller
from roomba_mapper.map_store import CLEAN, DIRTY, NOGO, OBSTACLE, UNKNOWN, GridMap, MapStore
from roomba_mapper.robot import SimulatedRobot
from roomba_mapper.server import make_server


class FakeClock:
    def __init__(self, t=1_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def run_until_idle(ctrl, limit=10_000):
    for _ in range(limit):
        if not ctrl.step() and ctrl.mode in ("idle", "manual"):
            return
    raise AssertionError("controller never went idle")


class GridMapTests(unittest.TestCase):
    def test_round_trip(self):
        g = GridMap(5, 4, cell_cm=25, dock=(1, 2))
        g.mark_cleaned(2, 2, now=10)
        g.set(3, 1, OBSTACLE)
        g.set(4, 3, NOGO)
        g2 = GridMap.from_dict(json.loads(json.dumps(g.to_dict())))
        self.assertEqual(g2.cells, g.cells)
        self.assertEqual(g2.dock, (1, 2))
        self.assertEqual(g2.cleaned_at[g2._idx(2, 2)], 10)
        self.assertEqual(g2.clean_count[g2._idx(2, 2)], 1)

    def test_stale_cells_need_cleaning_again(self):
        g = GridMap(3, 3)
        g.mark_cleaned(1, 1, now=0)
        self.assertFalse(g.needs_cleaning(1, 1, now=100, stale_after=3600))
        self.assertTrue(g.needs_cleaning(1, 1, now=4000, stale_after=3600))
        self.assertFalse(g.needs_cleaning(1, 1, now=10**9, stale_after=0))

    def test_reset_pass_keeps_layout(self):
        g = GridMap(3, 3)
        g.mark_cleaned(0, 0)
        g.set(1, 1, OBSTACLE)
        g.reset_pass()
        self.assertEqual(g.get(0, 0), DIRTY)
        self.assertEqual(g.get(1, 1), OBSTACLE)

    def test_resize_preserves_overlap(self):
        g = GridMap(4, 4, dock=(3, 3))
        g.set(1, 1, OBSTACLE)
        g.resize(2, 6)
        self.assertEqual((g.width, g.height), (2, 6))
        self.assertEqual(g.get(1, 1), OBSTACLE)
        self.assertEqual(g.dock, (1, 3))

    def test_rejects_bad_sizes(self):
        with self.assertRaises(ValueError):
            GridMap(1, 5)

    def test_store_is_atomic_and_round_trips(self):
        with tempfile.TemporaryDirectory() as d:
            store = MapStore(os.path.join(d, "m.json"))
            self.assertIsNone(store.load())
            store.save({"map": GridMap(3, 3).to_dict()})
            self.assertEqual(store.load()["map"]["width"], 3)
            self.assertEqual([f for f in os.listdir(d) if f.endswith(".tmp")], [])


class PlannerTests(unittest.TestCase):
    def test_path_around_wall(self):
        g = GridMap(5, 5)
        for y in range(4):
            g.set(2, y, OBSTACLE)
        path = planner.path_to(g, (0, 0), (4, 0))
        self.assertEqual(path[-1], (4, 0))
        self.assertIn((2, 4), path)
        for a, b in zip([(0, 0)] + path, path):
            planner.direction_between(a, b)  # every step is adjacent

    def test_unreachable(self):
        g = GridMap(3, 3)
        g.set(1, 0, OBSTACLE)
        g.set(0, 1, OBSTACLE)
        self.assertIsNone(planner.path_to(g, (0, 0), (2, 2)))

    def test_prefers_straight_ahead(self):
        g = GridMap(5, 5)
        g.mark_cleaned(2, 2)
        path = planner.coverage_path(g, (2, 2), "S", now=0)
        self.assertEqual(path, [(2, 3)])


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, "map.json")
        self.clock = FakeClock()

    def tearDown(self):
        self.dir.cleanup()

    def make(self, robot=None, size=(8, 6)):
        robot = robot or SimulatedRobot(size[0], size[1], obstacles={(3, 2), (3, 3), (6, 1)})
        return Controller(MapStore(self.path), robot, default_size=size, tick_s=0, clock=self.clock)

    def test_auto_cleans_everything_reachable_and_docks(self):
        robot = SimulatedRobot(8, 6, obstacles={(3, 2), (3, 3), (6, 1)})
        ctrl = self.make(robot)
        ctrl.set_mode("auto")
        run_until_idle(ctrl)
        g = ctrl.grid
        self.assertEqual(ctrl.mode, "idle")
        self.assertEqual(ctrl.pose, g.dock)
        self.assertTrue(robot.docked)
        for x, y in robot.obstacles:
            self.assertEqual(g.get(x, y), OBSTACLE)
        for y in range(6):
            for x in range(8):
                if (x, y) not in robot.obstacles:
                    self.assertEqual(g.get(x, y), CLEAN, (x, y))
        self.assertEqual(ctrl.snapshot()["stats"]["percent"], 100.0)

    def test_map_persists_and_second_run_skips_clean_cells(self):
        ctrl = self.make()
        ctrl.set_mode("auto")
        run_until_idle(ctrl)
        ctrl.save(force=True)

        robot = SimulatedRobot(8, 6, obstacles={(3, 2), (3, 3), (6, 1)})
        moves = []
        original = robot.move
        robot.move = lambda d: moves.append(d) or original(d)
        ctrl2 = self.make(robot)
        self.assertEqual(ctrl2.grid.get(3, 2), OBSTACLE)
        ctrl2.set_mode("auto")
        run_until_idle(ctrl2)
        self.assertEqual(moves, [])  # nothing left to clean

        # once the cleaning goes stale the robot cleans again
        self.clock.t += 25 * 3600
        ctrl2.set_mode("auto")
        run_until_idle(ctrl2)
        self.assertGreater(len(moves), 30)

    def test_manual_drive_and_goto(self):
        ctrl = self.make()
        ctrl.drive("S")
        ctrl.step()
        self.assertEqual(ctrl.pose, (0, 1))
        self.assertEqual(ctrl.grid.get(0, 1), CLEAN)
        self.assertEqual(ctrl.mode, "manual")
        ctrl.goto(5, 4)
        run_until_idle(ctrl)
        self.assertEqual(ctrl.pose, (5, 4))
        self.assertIn("Arrived", ctrl.status)

    def test_manual_drive_without_vacuum_marks_floor_dirty(self):
        ctrl = self.make()
        ctrl.update_settings({"clean_in_manual": False})
        ctrl.drive("E")
        ctrl.step()
        self.assertEqual(ctrl.grid.get(1, 0), DIRTY)

    def test_bump_marks_obstacle(self):
        ctrl = self.make(SimulatedRobot(8, 6, obstacles={(1, 0)}))
        ctrl.drive("E")
        ctrl.step()
        self.assertEqual(ctrl.pose, (0, 0))
        self.assertEqual(ctrl.grid.get(1, 0), OBSTACLE)

    def test_nogo_is_respected(self):
        ctrl = self.make(SimulatedRobot(8, 6))
        cells = [(x, y) for x in range(4, 8) for y in range(6)]
        ctrl.set_cells(cells, "nogo")
        ctrl.set_mode("auto")
        run_until_idle(ctrl)
        for x, y in cells:
            self.assertEqual(ctrl.grid.get(x, y), NOGO)
        ctrl.set_pose(3, 0)
        ctrl.drive("E")  # can't drive into a no-go zone
        self.assertEqual(ctrl.path, [])
        self.assertIn("no-go", ctrl.status)

    def test_cannot_wall_in_robot_or_dock(self):
        ctrl = self.make()
        self.assertEqual(ctrl.set_cells([(0, 0)], "obstacle"), 0)
        self.assertEqual(ctrl.grid.get(0, 0), UNKNOWN)

    def test_user_edit_mid_route_triggers_replan(self):
        ctrl = self.make(SimulatedRobot(8, 6))
        ctrl.goto(5, 0)
        ctrl.step()
        ctrl.set_cells([(3, 0)], "obstacle")
        run_until_idle(ctrl)
        self.assertEqual(ctrl.grid.get(3, 0), OBSTACLE)

    def test_set_pose_and_dock(self):
        ctrl = self.make()
        ctrl.set_dock(2, 2)
        ctrl.set_pose(4, 4, "N")
        self.assertEqual((ctrl.pose, ctrl.heading), ((4, 4), "N"))
        ctrl.set_mode("returning")
        run_until_idle(ctrl)
        self.assertEqual(ctrl.pose, (2, 2))

    def test_resize(self):
        ctrl = self.make()
        ctrl.set_pose(7, 5)
        ctrl.resize(4, 4)
        self.assertEqual((ctrl.grid.width, ctrl.grid.height), (4, 4))
        self.assertTrue(ctrl.grid.in_bounds(*ctrl.pose))

    def test_robot_error_stops_safely(self):
        class Broken(SimulatedRobot):
            def move(self, direction):
                raise IOError("serial cable unplugged")

        ctrl = self.make(Broken(8, 6))
        ctrl.tick_s = 0
        ctrl.start()
        try:
            ctrl.set_mode("auto")
            for _ in range(100):
                if ctrl.mode == "idle":
                    break
                threading.Event().wait(0.02)
            self.assertEqual(ctrl.mode, "idle")
            self.assertIn("serial cable unplugged", ctrl.status)
        finally:
            ctrl.shutdown()


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        store = MapStore(os.path.join(self.dir.name, "map.json"))
        self.ctrl = Controller(store, SimulatedRobot(6, 5), default_size=(6, 5), tick_s=0)
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
            with urllib.request.urlopen(req, timeout=5) as res:
                return res.status, res.read()
        except urllib.error.HTTPError as err:
            return err.code, err.read()

    def test_serves_gui(self):
        status, body = self.request("/")
        self.assertEqual(status, 200)
        self.assertIn(b"Roomba Map", body)
        self.assertEqual(self.request("/app.js")[0], 200)
        self.assertEqual(self.request("/../server.py")[0], 404)

    def test_pin_required(self):
        self.assertEqual(self.request("/api/state", pin="0000")[0], 401)
        self.assertEqual(self.request("/api/mode", {"mode": "auto"}, pin="")[0], 401)
        self.assertEqual(self.ctrl.mode, "idle")

    def test_api_round_trip(self):
        status, body = self.request("/api/state")
        self.assertEqual(status, 200)
        state = json.loads(body)
        self.assertEqual(len(state["cells"]), 30)

        status, body = self.request("/api/cells", {"cells": [[2, 2], [3, 2]], "state": "obstacle"})
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["cells"][2 * 6 + 2], "3")

        self.assertEqual(self.request("/api/drive", {"direction": "S"})[0], 200)
        self.assertEqual(self.ctrl.mode, "manual")
        self.assertEqual(self.request("/api/mode", {"mode": "auto"})[0], 200)
        self.assertEqual(self.ctrl.mode, "auto")

    def test_bad_input(self):
        self.assertEqual(self.request("/api/mode", {"mode": "fly"})[0], 400)
        self.assertEqual(self.request("/api/goto", {"x": "a", "y": 1})[0], 400)
        self.assertEqual(self.request("/api/cells", {"cells": "nope", "state": "clean"})[0], 400)
        self.assertEqual(self.request("/api/settings", {"stale_hours": None})[0], 400)
        self.assertEqual(self.request("/api/resize", {"width": 1, "height": 5})[0], 400)
        self.assertEqual(self.request("/api/nothing", {})[0], 404)


if __name__ == "__main__":
    unittest.main()
