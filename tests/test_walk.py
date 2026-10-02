import json
import os
import shutil
import ssl
import subprocess
import tempfile
import threading
import unittest
import urllib.request

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
class WalkGeometryTests(unittest.TestCase):
    def test_room_walked_from_the_dock(self):
        # 4.8 x 3.4 m room, dock mid-west wall, walked 30 cm from the walls
        pts = run_js('''console.log(JSON.stringify(W.shapeFromWalk([0.13, 0],
            [{q:1,dist:1.4},{q:0,dist:4.2},{q:3,dist:2.8},{q:2,dist:4.2},{q:1,dist:1.4}], 0.3, "floor")))''')
        self.assertEqual(pts, [[-0.17, -1.7], [4.63, -1.7], [4.63, 1.7], [-0.17, 1.7]])

    def test_counting_errors_are_shared_out_and_walls_stay_square(self):
        pts = run_js('''console.log(JSON.stringify(W.shapeFromWalk([0, 0],
            [{q:0,dist:4.3},{q:1,dist:3.1},{q:2,dist:3.9},{q:3,dist:2.9}], 0, "floor")))''')
        self.assertEqual(len(pts), 4)
        xs = sorted({p[0] for p in pts})
        ys = sorted({p[1] for p in pts})
        self.assertEqual(len(xs), 2)  # every wall is axis-aligned
        self.assertEqual(len(ys), 2)
        self.assertAlmostEqual(xs[1] - xs[0], 4.1, delta=0.02)  # between 3.9 and 4.3
        self.assertAlmostEqual(ys[1] - ys[0], 3.0, delta=0.02)

    def test_furniture_shrinks_and_l_shapes_work(self):
        res = run_js('''console.log(JSON.stringify([
            W.shapeFromWalk([1.7, 1.7], [{q:0,dist:1.6},{q:1,dist:1.6},{q:2,dist:1.6},{q:3,dist:1.6}], 0.3, "obstacle"),
            W.shapeFromWalk([0, 0], [{q:3,dist:2},{q:0,dist:4},{q:1,dist:4},{q:2,dist:2},{q:3,dist:2},{q:2,dist:2}], 0, "floor"),
            W.polygonArea(W.shapeFromWalk([0, 0], [{q:3,dist:2},{q:0,dist:4},{q:1,dist:4},{q:2,dist:2},{q:3,dist:2},{q:2,dist:2}], 0, "floor"))
        ]))''')
        self.assertEqual(res[0], [[2, 2], [3, 2], [3, 3], [2, 3]])
        self.assertEqual(len(res[1]), 6)
        self.assertAlmostEqual(res[2], 12.0)  # 4x4 minus a 2x2 notch, stored counter-clockwise

    def test_turns(self):
        res = run_js('console.log(JSON.stringify([W.turn(0,"left"), W.turn(0,"right"), W.turn(1,"around")]))')
        self.assertEqual(res, [1, 3, 3])

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
