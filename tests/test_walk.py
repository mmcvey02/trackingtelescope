import json
import math
import os
import shutil
import ssl
import subprocess
import tempfile
import threading
import unittest
import urllib.request

from roomba_mapper import geometry as geo
from roomba_mapper.controller import MapController
from roomba_mapper.geomap import GeoMap
from roomba_mapper.server import https_context, make_server
from roomba_mapper.simulator import SimRoomba
from roomba_mapper.store import MapStore

try:
    from helpers import DirectSimLink
except ImportError:
    from tests.helpers import DirectSimLink

WALK_JS = os.path.join(os.path.dirname(__file__), "..", "roomba_mapper", "static", "walk.js")
NODE = shutil.which("node")


def run_js(body):
    """Run a snippet against walk.js in node; the snippet must print JSON."""
    script = f"const W = require({json.dumps(os.path.abspath(WALK_JS))});\n{body}"
    out = subprocess.run([NODE, "-e", script], capture_output=True, text=True, timeout=30)
    if out.returncode:
        raise AssertionError(out.stderr)
    return json.loads(out.stdout)


@unittest.skipUnless(NODE, "needs node to test the browser code")
class WalkSensorTests(unittest.TestCase):
    def test_heading_counts_body_turns_however_the_phone_is_held(self):
        res = run_js('''
            const D = Math.PI / 180, out = [];
            const turn = (g, rate, axis) => {
              const h = new W.HeadingTracker();
              for (let i = 0; i < 10; i++) h.addGravity(...g);
              for (let t = 0; t <= 1000; t += 10) {
                const r = { alpha: 0, beta: 0, gamma: 0 }; r[axis] = rate;
                h.addRotation(r.alpha, r.beta, r.gamma, t);
              }
              return Math.round(h.heading / D);
            };
            out.push(turn([0, 0, 9.81], 90, "alpha"));     // flat, left
            out.push(turn([0, 9.81, 0], -90, "gamma"));    // upright, right
            out.push(turn([0, -9.81, 0], 90, "gamma"));    // upright, gravity sign flipped (iOS), left
            out.push(turn([0, 9.81, 0], 60, "beta"));      // tilting the phone, not turning
            console.log(JSON.stringify(out));''')
        self.assertEqual(res, [90, -90, 90, 0])

    def test_corners_found_and_glances_ignored(self):
        res = run_js('''
            const D = Math.PI / 180; let seed = 5;
            const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
            const w = new W.WalkTracker({ stepLen: 0.5 }), corners = [];
            const walk = (n, deg) => { for (let i = 0; i < n; i++) {
              const r = w.step((deg + (rnd() - 0.5) * 20) * D); if (r) corners.push(Math.round(r.angle)); } };
            walk(4, 90); walk(1, 0); walk(1, 90);        // turned left at the dock; a glance mid-wall
            walk(8, 0); walk(4, -90); walk(2, -135); walk(6, 180); walk(3, 90);
            console.log(JSON.stringify({ corners, pts: w.points().map(p => p.map(v => +v.toFixed(1))) }));''')
        self.assertEqual(len(res["corners"]), 5)
        for got, want in zip(res["corners"], [-90, -90, -45, -45, -90]):
            self.assertAlmostEqual(got, want, delta=8)
        self.assertEqual(res["pts"][1], [0.1, 3.0])  # first wall: 6 steps of 0.5 m, glance included

    def test_manual_turns_typed_lengths_and_undo(self):
        res = run_js('''
            const w = new W.WalkTracker({ stepLen: 0.7 });
            for (let i = 0; i < 6; i++) w.step(0);
            w.setLength(4.8);                      // measured 4.8 m: 0.8 m per step
            const t = w.manualTurn(90);
            for (let i = 0; i < 3; i++) w.step(Math.PI / 2);
            const before = w.points().length;
            w.manualTurn(-30);
            w.undoCorner();                         // "that wasn't a corner"
            console.log(JSON.stringify({ learned: t.learned, before, after: w.points().length,
                                         pts: w.points().map(p => p.map(v => +v.toFixed(2))) }));''')
        self.assertAlmostEqual(res["learned"], 0.8)
        self.assertEqual(res["after"], res["before"])
        self.assertEqual(res["pts"], [[0, 0], [4.8, 0], [4.8, 2.1]])

    def test_step_counter(self):
        res = run_js('''
            const walking = new W.StepCounter(), still = new W.StepCounter();
            let seed = 1; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
            for (let t = 0; t < 10000; t += 20) {
              walking.add(0.1, 0.2, 9.81 + 2.5 * Math.max(0, Math.sin(2 * Math.PI * 2 * t / 1000)) + (rnd() - 0.5) * 0.4, t);
              still.add(0.1, 0.2, 9.81 + (rnd() - 0.5) * 0.3, t);
            }
            console.log(JSON.stringify([walking.steps, still.steps]));''')
        self.assertEqual(res, [20, 0])


class WalkOutlineTests(unittest.TestCase):
    """geometry.walk_to_polygon: from the walked path to the room outline."""

    def corners(self, pts):
        out = []
        for k in range(len(pts)):
            a, b, c = pts[k - 1], pts[k], pts[(k + 1) % len(pts)]
            t = math.degrees(math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(b[1] - a[1], b[0] - a[0]))
            out.append(round((t + 180) % 360 - 180))
        return out

    def test_room_walked_from_the_dock(self):
        # 4.8 x 3.4 m room, dock mid-west wall, walked 30 cm from the walls
        path = [(0.13, 0), (0.13, 1.4), (4.33, 1.4), (4.33, -1.4), (0.13, -1.4), (0.13, 0)]
        pts = geo.walk_to_polygon(path, gap=0.3)
        self.assertEqual(sorted({round(x, 2) for x, _ in pts}), [-0.17, 4.63])
        self.assertEqual(sorted({round(y, 2) for _, y in pts}), [-1.7, 1.7])

    def test_drifting_gyro_still_gives_square_corners(self):
        # each turn over-rotated by 4 degrees: 16 degrees of drift by the end
        path, h, (x, y) = [(0.0, 0.0)], 0.0, (0.0, 0.0)
        for length in (4, 3, 4, 3):
            x += math.cos(h) * length
            y += math.sin(h) * length
            path.append((x, y))
            h += math.radians(94)
        pts = geo.walk_to_polygon(path, gap=0)
        self.assertEqual(self.corners(pts), [90, 90, 90, 90])
        self.assertAlmostEqual(abs(geo.polygon_area(pts)), 12.0, delta=0.6)
        self.assertEqual(len({round(x, 2) for x, _ in pts}), 2)  # lined up with the dock

    def test_angled_walls_are_kept(self):
        path = [(0, 0), (4, 0), (4, 2), (3, 3), (0, 3), (0.05, 0.03)]  # 45 degree cut corner
        pts = geo.walk_to_polygon(path, gap=0)
        self.assertEqual(len(pts), 5)
        self.assertEqual(sorted(self.corners(pts)), [45, 45, 90, 90, 90])
        raw = geo.walk_to_polygon([(0, 0), (4, 0.3), (4.2, 3), (0, 3), (0, 0.1)], gap=0, square=False)
        self.assertNotEqual(self.corners(raw), [90, 90, 90, 90])  # squaring switched off

    def test_rooms_at_an_angle_to_the_dock_keep_their_angle(self):
        tf = (0, 0, math.radians(30))
        path = [geo.transform(x, y, tf) for x, y in [(0, 0), (4, 0), (4, 3), (0, 3), (0, 0.05)]]
        pts = geo.walk_to_polygon(path, gap=0)
        self.assertAlmostEqual(math.degrees(geo.dominant_angle(pts)), 30, delta=1)

    def test_furniture_shrinks(self):
        pts = geo.walk_to_polygon([(1.7, 1.7), (3.3, 1.7), (3.3, 3.3), (1.7, 3.3), (1.7, 1.71)],
                                  gap=0.3, kind="obstacle")
        self.assertEqual(sorted({round(x, 1) for x, _ in pts}), [2.0, 3.0])

    def test_api(self):
        with tempfile.TemporaryDirectory() as d:
            ctrl = MapController(MapStore(os.path.join(d, "m.json")), DirectSimLink(SimRoomba(seed=1)))
            el = ctrl.add_walked_shape("floor", [[0, 0], [4, 0], [4, 3], [0, 3], [0, 0.1]], gap=0, name="Hall")
            self.assertEqual(ctrl.map.elements[0]["id"], el)
            self.assertEqual(ctrl.map.elements[0]["name"], "Hall")
            for bad in ([[0, 0], [1, 1]], "x", [[0, 0], [1, 0], [2, 0], [3, 0]]):
                with self.assertRaises(ValueError):
                    ctrl.add_walked_shape("floor", bad)
            with self.assertRaises(ValueError):
                ctrl.add_walked_shape("lava", [[0, 0], [4, 0], [4, 3], [0, 3]])


class UntrackedRobotTests(unittest.TestCase):
    def test_hand_made_map_without_positions_claims_nothing(self):
        m = GeoMap()
        m.add_element("floor", [[0, 0], [4, 0], [4, 3], [0, 3]])
        st = m.stats(0, 0)
        self.assertEqual(st["floor_m2"], 12.0)
        self.assertIsNone(st["percent"])
        self.assertEqual(m.missed_spots(0, 0), [])
        self.assertEqual(m.coverage_raster(0, 0)["w"], 0)


@unittest.skipUnless(shutil.which("openssl"), "needs openssl")
class HttpsTests(unittest.TestCase):
    def test_app_served_over_https(self):
        with tempfile.TemporaryDirectory() as d:
            ctx = https_context(os.path.join(d, "c.pem"), os.path.join(d, "k.pem"))
            self.assertEqual(oct(os.stat(os.path.join(d, "k.pem")).st_mode & 0o777), "0o600")
            ctrl = MapController(MapStore(os.path.join(d, "m.json")), DirectSimLink(SimRoomba(seed=1)))
            server = make_server(ctrl, "127.0.0.1", 0, ssl_context=ctx)
            threading.Thread(target=server.serve_forever, daemon=True).start()
            try:
                client = ssl.create_default_context()
                client.check_hostname = False
                client.verify_mode = ssl.CERT_NONE
                url = f"https://127.0.0.1:{server.server_address[1]}"
                with urllib.request.urlopen(url + "/walk.js", context=client, timeout=10) as res:
                    self.assertIn(b"StepCounter", res.read())
                with urllib.request.urlopen(url + "/api/state", context=client, timeout=10) as res:
                    self.assertIn("stats", json.loads(res.read()))
                # the same certificate is reused next time
                cert = os.path.join(d, "c.pem")
                before = os.stat(cert).st_mtime_ns
                https_context(cert, os.path.join(d, "k.pem"))
                self.assertEqual(before, os.stat(cert).st_mtime_ns)
            finally:
                server.shutdown()
                server.server_close()


if __name__ == "__main__":
    unittest.main()
