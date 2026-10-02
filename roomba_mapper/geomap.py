"""The geometric map: floor plan polygons, what has been explored, what is clean.

* ``elements`` - polygons in metres (dock = origin): ``floor`` areas,
  ``obstacle`` areas (furniture, kitchen islands, ...) and ``nogo`` zones.
  Each is either ``auto`` (generated from where the robot has driven) or
  ``manual`` (drawn by the user). Regenerating the map only replaces
  ``auto`` elements, so hand edits are never lost.
* ``explored`` - fine raster of every spot the robot body has passed over.
  This is the raw material the floor plan is generated from.
* ``coverage`` - coarser raster of when each spot was last cleaned.
* ``locked`` - once locked the floor plan is the robot's reference: new runs
  are aligned to it and measured against it, and no longer reshape it.
"""

import math

from . import geometry as geo

EXPLORE_RES = 0.05  # m
COVER_RES = 0.10    # m
KINDS = ("floor", "obstacle", "nogo")
MAX_COORD = 200.0
MAX_POINTS = 1000
MAX_ELEMENTS = 500


def _clean_points(points):
    if not isinstance(points, (list, tuple)) or not 3 <= len(points) <= MAX_POINTS:
        raise ValueError(f"a shape needs between 3 and {MAX_POINTS} points")
    out = []
    for p in points:
        if not (isinstance(p, (list, tuple)) and len(p) == 2):
            raise ValueError("points must be [x, y] pairs")
        x, y = float(p[0]), float(p[1])
        if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > MAX_COORD or abs(y) > MAX_COORD:
            raise ValueError("point out of range")
        out.append((round(x, 3), round(y, 3)))
    if abs(geo.polygon_area(out)) < 1e-4:
        raise ValueError("shape has no area")
    if geo.polygon_area(out) < 0:
        out.reverse()
    return out


class GeoMap:
    def __init__(self):
        self.elements = []
        self.explored = set()
        self.coverage = {}
        self.locked = False
        self.pmap_id = None
        self.runs = []
        self.last_trail = []
        self.version = 0       # bumps when geometry changes
        self.cov_version = 0   # bumps when coverage / exploration changes
        self._next_id = 1
        self._mask = None
        self._mask_version = -1

    # -- elements ---------------------------------------------------------

    def _touch(self):
        self.version += 1
        self.cov_version += 1

    def add_element(self, kind, points, source="manual", name=None):
        if kind not in KINDS:
            raise ValueError(f"kind must be one of {KINDS}")
        if len(self.elements) >= MAX_ELEMENTS:
            raise ValueError("too many shapes on the map")
        el = {
            "id": self._next_id,
            "kind": kind,
            "source": source,
            "name": (str(name)[:40] if name else None),
            "points": [list(p) for p in _clean_points(points)],
        }
        self._next_id += 1
        self.elements.append(el)
        self._touch()
        return el["id"]

    def _find(self, el_id):
        for el in self.elements:
            if el["id"] == el_id:
                return el
        raise ValueError(f"no shape with id {el_id}")

    def update_element(self, el_id, points=None, kind=None, name=None):
        el = self._find(el_id)
        if points is not None:
            el["points"] = [list(p) for p in _clean_points(points)]
        if kind is not None:
            if kind not in KINDS:
                raise ValueError(f"kind must be one of {KINDS}")
            el["kind"] = kind
        if name is not None:
            el["name"] = str(name)[:40] or None
        el["source"] = "manual"  # a hand-edited shape is kept on regeneration
        self._touch()

    def delete_element(self, el_id):
        self.elements.remove(self._find(el_id))
        self._touch()

    def polygons(self, kind):
        return [el["points"] for el in self.elements if el["kind"] == kind]

    # -- floor model ------------------------------------------------------

    def has_floor_plan(self):
        return any(el["kind"] == "floor" for el in self.elements)

    def is_floor(self, x, y):
        if not any(geo.point_in_polygon(x, y, p) for p in self.polygons("floor")):
            return False
        return not any(geo.point_in_polygon(x, y, p)
                       for kind in ("obstacle", "nogo") for p in self.polygons(kind))

    def floor_mask(self):
        """Set of COVER_RES cells whose centre is cleanable floor (cached)."""
        if self._mask is not None and self._mask_version == self.version:
            return self._mask
        mask = set()
        floors = self.polygons("floor")
        blocked = self.polygons("obstacle") + self.polygons("nogo")
        for poly in floors:
            x0, y0, x1, y1 = geo.polygon_bounds(poly)
            for i in range(int(math.floor(x0 / COVER_RES)), int(math.ceil(x1 / COVER_RES)) + 1):
                cx = (i + 0.5) * COVER_RES
                for j in range(int(math.floor(y0 / COVER_RES)), int(math.ceil(y1 / COVER_RES)) + 1):
                    cy = (j + 0.5) * COVER_RES
                    if (i, j) not in mask and geo.point_in_polygon(cx, cy, poly):
                        mask.add((i, j))
        if blocked:
            for poly in blocked:
                x0, y0, x1, y1 = geo.polygon_bounds(poly)
                for i in range(int(math.floor(x0 / COVER_RES)), int(math.ceil(x1 / COVER_RES)) + 1):
                    cx = (i + 0.5) * COVER_RES
                    for j in range(int(math.floor(y0 / COVER_RES)), int(math.ceil(y1 / COVER_RES)) + 1):
                        if (i, j) in mask and geo.point_in_polygon(cx, (j + 0.5) * COVER_RES, poly):
                            mask.discard((i, j))
        self._mask, self._mask_version = mask, self.version
        return mask

    def reference_test(self, margin=0.12):
        """Fast inside-test for alignment: can the robot's centre be at (x, y)?

        The floor is shrunk by `margin` (a bit less than the robot's radius),
        so a run that is shifted towards a wall shows up as points outside.
        """
        key = (self.version, margin)
        if getattr(self, "_ref_key", None) != key:
            mask = self.floor_mask()
            off = geo.disc_offsets(margin / COVER_RES)
            self._ref_cells = geo.erode(mask, off) if off else mask
            self._ref_key = key
        cells = self._ref_cells
        return lambda x, y: (int(math.floor(x / COVER_RES)), int(math.floor(y / COVER_RES))) in cells

    # -- recording what the robot did ------------------------------------

    def sweep(self, x0, y0, x1, y1, now, clean_radius, body_radius, learn=True, clean=True):
        """Record the robot moving in a straight line from (x0, y0) to (x1, y1).

        clean: the robot was cleaning (not just driving home).
        learn: add the path to the explored area the floor plan is built from.
        """
        if clean:
            for cell in geo.cells_along(x0, y0, x1, y1, clean_radius, COVER_RES):
                self.coverage[cell] = now
        if learn:
            self.explored.update(geo.cells_along(x0, y0, x1, y1, body_radius, EXPLORE_RES))
        self.cov_version += 1

    def rebuild(self, simplify_tol=0.10, gap_close_m=0.15, min_room_m2=0.5, min_obstacle_m2=0.06):
        """Regenerate the auto floor plan from the explored raster.

        Returns the total floor area (m^2) of the generated plan.
        """
        self.elements = [el for el in self.elements if el["source"] != "auto"]
        cells = set(self.explored)
        if cells:
            off = geo.disc_offsets(gap_close_m / EXPLORE_RES)
            cells = geo.erode(geo.dilate(cells, off), off)  # close gaps between passes
            cells, _ = geo.fill_small_holes(cells, int(min_obstacle_m2 / EXPLORE_RES ** 2))
            outers, holes = geo.extract_polygons(
                cells, EXPLORE_RES, simplify_tol, min_room_m2, min_obstacle_m2)
            for k, poly in enumerate(sorted(outers, key=geo.polygon_area, reverse=True)):
                self._add_auto("floor", poly, f"Area {k + 1}")
            for poly in holes:
                self._add_auto("obstacle", poly, None)
        self._touch()
        return self.floor_area()

    def _add_auto(self, kind, poly, name):
        try:
            self.add_element(kind, poly, source="auto", name=name)
        except ValueError:
            pass  # degenerate after simplification

    def floor_area(self):
        return round(len(self.floor_mask()) * COVER_RES ** 2, 2)

    # -- coverage ---------------------------------------------------------

    def _fresh(self, cell, now, stale_after):
        t = self.coverage.get(cell)
        return t is not None and (stale_after <= 0 or now - t <= stale_after)

    def coverage_raster(self, now, stale_after):
        """Compact grid for the GUI.

        Codes: 0 nothing, 1 floor that needs cleaning, 2 floor cleaned,
        3 cleaned/driven outside the floor plan.
        """
        # Robots that can't report their position never record coverage; then
        # the plan is shown plain rather than all "needs cleaning".
        mask = self.floor_mask() if self.coverage else set()
        fresh = {c for c in self.coverage if self._fresh(c, now, stale_after)}
        cells = mask | fresh
        if not cells:
            return {"res": COVER_RES, "i0": 0, "j0": 0, "w": 0, "h": 0, "data": ""}
        i0 = min(i for i, _ in cells)
        i1 = max(i for i, _ in cells)
        j0 = min(j for _, j in cells)
        j1 = max(j for _, j in cells)
        rows = []
        for j in range(j1, j0 - 1, -1):  # top row first
            row = []
            for i in range(i0, i1 + 1):
                c = (i, j)
                if c in mask:
                    row.append("2" if c in fresh else "1")
                elif c in fresh:
                    row.append("3")
                else:
                    row.append("0")
            rows.append("".join(row))
        return {"res": COVER_RES, "i0": i0, "j0": j0, "w": i1 - i0 + 1, "h": j1 - j0 + 1,
                "data": "".join(rows)}

    def missed_spots(self, now, stale_after, min_area=0.15, limit=12):
        if not self.coverage:
            return []  # nothing has ever been tracked, so nothing can be called missed
        need = {c for c in self.floor_mask() if not self._fresh(c, now, stale_after)}
        spots = []
        for comp in geo.components(need, eight=False):
            area = len(comp) * COVER_RES ** 2
            if area < min_area:
                continue
            cx = sum(i + 0.5 for i, _ in comp) / len(comp) * COVER_RES
            cy = sum(j + 0.5 for _, j in comp) / len(comp) * COVER_RES
            spots.append({"x": round(cx, 2), "y": round(cy, 2), "area": round(area, 2)})
        spots.sort(key=lambda s: -s["area"])
        return spots[:limit]

    def stats(self, now, stale_after):
        mask = self.floor_mask()
        total = len(mask)
        clean = sum(1 for c in mask if self._fresh(c, now, stale_after))
        a = COVER_RES ** 2
        return {
            "floor_m2": round(total * a, 1),
            "cleaned_m2": round(clean * a, 1),
            "remaining_m2": round((total - clean) * a, 1),
            "percent": round(100.0 * clean / total, 1) if total and self.coverage else None,
            "explored_m2": round(len(self.explored) * EXPLORE_RES ** 2, 1),
            "obstacles": sum(1 for el in self.elements if el["kind"] == "obstacle"),
        }

    def reset_coverage(self):
        self.coverage.clear()
        self.cov_version += 1

    def erase(self):
        keep_version = (self.version, self.cov_version)
        self.__init__()
        self.version, self.cov_version = keep_version[0] + 1, keep_version[1] + 1

    # -- persistence --------------------------------------------------------

    @staticmethod
    def _rle(cells):
        rows = {}
        for i, j in sorted(cells, key=lambda c: (c[1], c[0])):
            runs = rows.setdefault(j, [])
            if runs and runs[-1][0] + runs[-1][1] == i:
                runs[-1][1] += 1
            else:
                runs.append([i, 1])
        return [[j, runs] for j, runs in rows.items()]

    @staticmethod
    def _unrle(data):
        out = set()
        for j, runs in data:
            for i, n in runs:
                out.update((i + k, int(j)) for k in range(int(n)))
        return out

    def to_dict(self):
        return {
            "elements": self.elements,
            "explored": self._rle(self.explored),
            "coverage": [[i, j, int(t)] for (i, j), t in self.coverage.items()],
            "locked": self.locked,
            "pmap_id": self.pmap_id,
            "runs": self.runs[-50:],
            "last_trail": self.last_trail,
            "next_id": self._next_id,
        }

    @classmethod
    def from_dict(cls, data):
        m = cls()
        for el in data.get("elements", []):
            if el.get("kind") not in KINDS:
                raise ValueError("invalid shape in saved map")
            m.elements.append({
                "id": int(el["id"]), "kind": el["kind"], "source": el.get("source", "manual"),
                "name": el.get("name"), "points": [list(p) for p in _clean_points(el["points"])],
            })
        m.explored = cls._unrle(data.get("explored", []))
        m.coverage = {(int(i), int(j)): float(t) for i, j, t in data.get("coverage", [])}
        m.locked = bool(data.get("locked", False))
        m.pmap_id = data.get("pmap_id")
        m.runs = list(data.get("runs", []))
        m.last_trail = [list(p) for p in data.get("last_trail", [])]
        m._next_id = max([int(data.get("next_id", 1))] + [el["id"] + 1 for el in m.elements])
        return m

