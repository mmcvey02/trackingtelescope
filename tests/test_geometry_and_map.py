import json
import math
import os
import tempfile
import unittest

from roomba_mapper import geometry as geo
from roomba_mapper.geomap import EXPLORE_RES, GeoMap
from roomba_mapper.store import MapStore


def lawnmower(m, width, height, hole=None, rot=0.0, t=0.0, lane=0.3):
    """Sweep a rectangular room (optionally around a rectangular hole)."""
    rows = int(height / lane)
    for row in range(rows + 1):
        y = 0.17 + row * (height - 0.34) / rows
        xs = [0.17, width - 0.17] if row % 2 == 0 else [width - 0.17, 0.17]
        pts = [(xs[0] + (xs[1] - xs[0]) * k / 60, y) for k in range(61)]
        if hole:
            hx0, hy0, hx1, hy1 = hole
            pts = [p for p in pts if not (hx0 - 0.17 < p[0] < hx1 + 0.17 and hy0 - 0.17 < p[1] < hy1 + 0.17)]
        pts = [geo.transform(x, y, (0, 0, rot)) for x, y in pts]
        for a, b in zip(pts, pts[1:]):
            if math.dist(a, b) < 0.2:
                m.sweep(*a, *b, t, 0.17, 0.17)


class GeometryTests(unittest.TestCase):
    def test_extract_l_shape_and_hole(self):
        cells = {(i, j) for i in range(80) for j in range(60)}
        cells -= {(i, j) for i in range(30, 50) for j in range(20, 30)}
        cells -= {(i, j) for i in range(60, 80) for j in range(40, 60)}
        outers, holes = geo.extract_polygons(cells, 0.05, 0.03, 0.5, 0.05)
        self.assertEqual(len(outers), 1)
        self.assertEqual(len(outers[0]), 6)
        self.assertAlmostEqual(geo.polygon_area(outers[0]), 11.0, places=2)
        self.assertEqual(len(holes), 1)
        self.assertAlmostEqual(geo.polygon_area(holes[0]), 0.5, places=2)

    def test_pinch_points_make_simple_loops(self):
        loops = geo.trace_loops({(0, 0), (1, 1)})
        self.assertEqual(len(loops), 2)
        self.assertTrue(all(len(loop) == 4 for loop in loops))

    def test_orthogonalize_snaps_rotated_room(self):
        th = math.radians(20)
        ragged = [(0, 0), (2, 0.05), (4, -0.04), (4.03, 1.5), (3.98, 3), (2, 3.04), (0.02, 3)]
        pts = [geo.transform(x, y, (0, 0, th)) for x, y in ragged]
        out = geo.orthogonalize(pts)
        self.assertEqual(len(out), 4)
        for k in range(4):
            (x1, y1), (x2, y2) = out[k], out[(k + 1) % 4]
            ang = (math.degrees(math.atan2(y2 - y1, x2 - x1)) - 20) % 90
            self.assertLess(min(ang, 90 - ang), 0.5)

    def test_chamfered_corners_become_square(self):
        pts = [(0.2, 0), (4, 0), (4, 3), (0, 3), (0, 0.2)]  # one cut corner
        out = geo.orthogonalize(pts)
        self.assertIn((0.0, 0.0), [(round(x, 2), round(y, 2)) for x, y in out])

    def test_align_recovers_offset(self):
        room = [(0, 0), (5, 0), (5, 3), (2, 3), (2, 4), (0, 4)]
        inside = lambda x, y: geo.point_in_polygon(x, y, room) and all(
            geo.point_segment_distance((x, y), room[k], room[(k + 1) % len(room)]) > 0.15
            for k in range(len(room)))
        # a robot path: rows that run right up to the walls (centre 0.17 m from them)
        truth = [(0.17 + 0.1 * i, 0.17 + 0.24 * j) for i in range(47) for j in range(16)]
        truth = [p for p in truth if inside(*p)]
        drift = (0.15, -0.1, math.radians(3))
        observed = [geo.transform(x, y, drift) for x, y in truth]
        tf, score = geo.align(observed, inside)
        self.assertGreater(score, 0.95)
        for (tx, ty), (ox, oy) in zip(truth[::40], observed[::40]):
            fx, fy = geo.transform(ox, oy, tf)
            self.assertLess(math.hypot(fx - tx, fy - ty), 0.15)

    def test_align_keeps_identity_when_already_right(self):
        inside = lambda x, y: 0.2 < x < 4.8 and 0.2 < y < 2.8
        pts = [(0.3 + 0.2 * i, 0.3 + 0.2 * j) for i in range(22) for j in range(12)]
        tf, score = geo.align(pts, inside)
        self.assertEqual(tf, (0.0, 0.0, 0.0))
        self.assertEqual(score, 1.0)


class GeoMapTests(unittest.TestCase):
    def test_learns_room_with_obstacle(self):
        m = GeoMap()
        lawnmower(m, 4.0, 3.0, hole=(1.8, 1.0, 2.8, 1.8))
        area = m.rebuild()
        floors = m.polygons("floor")
        obstacles = m.polygons("obstacle")
        self.assertEqual(len(floors), 1)
        self.assertLessEqual(len(floors[0]), 6)
        self.assertAlmostEqual(geo.polygon_area(floors[0]), 12.0, delta=0.8)
        self.assertEqual(len(obstacles), 1)
        self.assertAlmostEqual(geo.polygon_area(obstacles[0]), 0.8, delta=0.4)
        self.assertAlmostEqual(area, 12.0 - 0.8, delta=1.0)

    def test_manual_shapes_survive_rebuild(self):
        m = GeoMap()
        lawnmower(m, 3.0, 3.0)
        m.rebuild()
        nogo = m.add_element("nogo", [[1, 1], [1.5, 1], [1.5, 1.5], [1, 1.5]])
        auto_floor = m.polygons("floor")[0]
        floor_id = next(el["id"] for el in m.elements if el["kind"] == "floor")
        m.update_element(floor_id, points=[[0, 0], [3, 0], [3, 3], [0, 3]])
        m.rebuild()
        kinds = sorted((el["kind"], el["source"]) for el in m.elements)
        self.assertIn(("nogo", "manual"), kinds)
        self.assertIn(("floor", "manual"), kinds)
        self.assertTrue(any(el["id"] == nogo for el in m.elements))
        self.assertNotEqual(auto_floor, m.polygons("floor")[0])

    def test_shape_validation(self):
        m = GeoMap()
        for bad in ([[0, 0], [1, 1]], [[0, 0], [1, 0], [2, 0]], [[0, 0], [1e9, 0], [0, 1]],
                    "nope", [[0, 0], [1, "x"], [0, 1]]):
            with self.assertRaises((ValueError, TypeError)):
                m.add_element("floor", bad)
        with self.assertRaises(ValueError):
            m.add_element("lava", [[0, 0], [1, 0], [0, 1]])
        el = m.add_element("floor", [[0, 0], [0, 1], [1, 0]])  # clockwise input
        self.assertGreater(geo.polygon_area(m.elements[0]["points"]), 0)
        with self.assertRaises(ValueError):
            m.delete_element(el + 99)

    def test_coverage_and_staleness(self):
        m = GeoMap()
        m.add_element("floor", [[0, 0], [2, 0], [2, 1], [0, 1]])
        m.add_element("obstacle", [[1.5, 0], [2, 0], [2, 0.5], [1.5, 0.5]])
        self.assertAlmostEqual(m.stats(0, 0)["floor_m2"], 1.75, delta=0.1)
        m.sweep(0.2, 0.5, 1.0, 0.5, now=100, clean_radius=0.5, body_radius=0.17)
        st = m.stats(200, stale_after=3600)
        self.assertGreater(st["percent"], 50)
        self.assertEqual(m.stats(100 + 7200, stale_after=3600)["percent"], 0.0)
        self.assertTrue(m.missed_spots(200, 3600))
        r = m.coverage_raster(200, 3600)
        self.assertEqual(len(r["data"]), r["w"] * r["h"])
        self.assertIn("2", r["data"])
        self.assertIn("1", r["data"])

    def test_returning_drive_is_not_cleaning(self):
        m = GeoMap()
        m.sweep(0, 0, 1, 0, now=1, clean_radius=0.17, body_radius=0.17, clean=False)
        self.assertFalse(m.coverage)
        self.assertTrue(m.explored)

    def test_round_trip(self):
        m = GeoMap()
        lawnmower(m, 3.0, 2.0)
        m.rebuild()
        m.add_element("nogo", [[1, 1], [1.5, 1], [1.5, 1.5]], name="Cables")
        m.locked = True
        m.pmap_id = "abc"
        m2 = GeoMap.from_dict(json.loads(json.dumps(m.to_dict())))
        self.assertEqual(m2.elements, m.elements)
        self.assertEqual(m2.explored, m.explored)
        self.assertEqual(set(m2.coverage), set(m.coverage))
        self.assertTrue(m2.locked)
        self.assertEqual(m2.pmap_id, "abc")
        new_id = m2.add_element("floor", [[5, 5], [6, 5], [6, 6]])
        self.assertNotIn(new_id, [el["id"] for el in m.elements])

    def test_explored_resolution_is_fine_enough(self):
        self.assertLessEqual(EXPLORE_RES, 0.05)


class StoreTests(unittest.TestCase):
    def test_atomic_save_and_v1_migration(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "m.json")
            store = MapStore(path)
            self.assertIsNone(store.load())
            store.save({"map": GeoMap().to_dict()})
            self.assertEqual(store.load()["format"], 2)
            self.assertEqual([f for f in os.listdir(d) if f.endswith(".tmp")], [])
            with open(path, "w") as fh:
                json.dump({"format": 1, "map": {}}, fh)
            self.assertIsNone(store.load())
            self.assertTrue(os.path.exists(path + ".v1.bak"))


if __name__ == "__main__":
    unittest.main()
