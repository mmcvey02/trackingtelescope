"""Builds and maintains the map from what the robot reports.

Lifecycle of the map:

1. **Learning** - while there is no locked map, every run's path is added to
   the explored area and the floor plan is regenerated when the run ends.
2. **Locked (reference)** - once the plan stops growing (or the user locks
   it, or draws it by hand) it becomes the robot's reference: each new run
   is aligned to it to cancel odometry drift, cleaning is measured against
   it, and it no longer changes unless the user edits or unlocks it.

The user can draw, edit and delete shapes at any time; hand-made shapes
are never overwritten by regeneration.
"""

import math
import threading
import time

from . import geometry as geo
from .geomap import GeoMap
from .wifi import activity_from, mop_params

DEFAULT_SETTINGS = {
    "stale_hours": 24.0,          # a cleaned spot needs cleaning again after this (0 = never)
    "auto_lock": True,            # lock the learned map once a run stops adding new floor
    "keep_learning": False,       # keep growing the explored area after locking
    "align": True,                # correct each run's drift against the reference map
    "auto_start_percent": 0.0,    # start a clean when this % of floor needs it (0 = off)
    "auto_start_min_hours": 12.0, # ... but not sooner than this after the last run
    "mop_wetness": 2,             # 1-3 for Combo models
    "view_rotation": 0.0,         # display only
    "view_mirror": False,         # display only
}
ALIGN_AT = (30, 120, 300)        # pose counts at which a run is (re)aligned
ALIGN_EVERY = 300
MAX_JUMP_M = 1.0                  # ignore teleports (relocalisation, bad samples)
RUN_TIMEOUT_S = 120.0
SAVE_INTERVAL_S = 5.0
LOCK_GROWTH = 0.05                # < 5 % new floor in a run => map is complete


def _downsample(points, limit):
    if len(points) <= limit:
        return points
    step = len(points) / limit
    return [points[int(k * step)] for k in range(limit)]


class MapController:
    def __init__(self, store, link, clock=time.time):
        self.store = store
        self.link = link
        self.clock = clock
        self.lock = threading.RLock()
        self.settings = dict(DEFAULT_SETTINGS)
        doc = store.load()
        if doc:
            self.map = GeoMap.from_dict(doc["map"])
            self.settings.update({k: v for k, v in doc.get("settings", {}).items()
                                  if k in DEFAULT_SETTINGS})
            self.last_run_end = doc.get("last_run_end", 0.0)
            self.notice = "Map loaded" + (" (reference locked)" if self.map.locked else " - still learning")
        else:
            self.map = GeoMap()
            self.last_run_end = 0.0
            self.notice = "No map yet - start a clean and the map will draw itself, or draw it by hand"
        prof = getattr(link, "profile", {}) or {}
        self.body_radius = prof.get("body_radius", 0.17)
        self.clean_radius = prof.get("clean_radius", 0.17)
        self.robot = {"activity": "unknown", "battery": None, "bin_full": False, "error": 0,
                      "connected": False, "link": "Connecting...", "pose": None,
                      "pose_source": None, "name": getattr(link, "name", "robot"),
                      "model": prof.get("name"),
                      "has_mop": prof.get("mop", False) and prof.get("mode_select", True)}
        self.mode_select = prof.get("mop", False) and prof.get("mode_select", True)
        self.run = None
        self._dirty = False
        self._last_save = 0.0
        self._cache = {}
        self._stop = threading.Event()
        self._thread = None
        link.on_state = self._on_state
        link.on_pose = self._on_pose
        link.on_link = self._on_link

    # -- lifecycle ------------------------------------------------------------

    def start(self):
        self.link.start()
        self._thread = threading.Thread(target=self._housekeeping, name="map-housekeeping", daemon=True)
        self._thread.start()

    def shutdown(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
        with self.lock:
            if self.run:
                self._end_run("server stopped")
        self.save(force=True)
        self.link.close()

    def _housekeeping(self):
        last_auto_check = 0.0
        while not self._stop.wait(1.0):
            now = self.clock()
            with self.lock:
                if self.run and now - self.run["last_pose"] > RUN_TIMEOUT_S \
                        and self.robot["activity"] not in ("cleaning", "returning", "paused"):
                    self._end_run("no position updates")
            if now - last_auto_check > 60:
                last_auto_check = now
                self.check_auto_start()
            self.save()

    def save(self, force=False):
        with self.lock:
            if not (force or self._dirty) or (
                    not force and time.monotonic() - self._last_save < SAVE_INTERVAL_S):
                return
            doc = {"map": self.map.to_dict(), "settings": self.settings,
                   "last_run_end": self.last_run_end}
            self._dirty = False
            self._last_save = time.monotonic()
        self.store.save(doc)

    def _changed(self):
        self._dirty = True

    @property
    def stale_after(self):
        return float(self.settings["stale_hours"]) * 3600.0

    # -- events from the robot --------------------------------------------------

    def _on_link(self, text, ok):
        with self.lock:
            self.robot["link"] = text
            self.robot["connected"] = ok

    def _on_state(self, reported):
        with self.lock:
            act = activity_from(reported)
            cms = reported.get("cleanMissionStatus") or {}
            self.robot.update(
                activity=act,
                battery=reported.get("batPct", self.robot["battery"]),
                bin_full=bool((reported.get("bin") or {}).get("full")),
                error=cms.get("error", 0) or 0,
            )
            pmap = self._pmap_id(reported)
            if pmap:
                if self.map.pmap_id is None:
                    self.map.pmap_id = pmap
                    self._changed()
                elif pmap != self.map.pmap_id and self.map.locked:
                    self.notice = ("The robot rebuilt its own internal map - if the dock moved, "
                                   "unlock and relearn, or move the shapes to match")
            if act in ("cleaning", "returning") and self.run is None:
                self._begin_run()
            elif act in ("docked", "idle", "emptying") and self.run is not None:
                self._end_run("finished")

    @staticmethod
    def _pmap_id(reported):
        pm = reported.get("pmaps")
        if isinstance(pm, list) and pm and isinstance(pm[0], dict) and pm[0]:
            return next(iter(pm[0]))
        return (reported.get("_rrtp") or {}).get("pmap_id")

    def _on_pose(self, x, y, th, source):
        with self.lock:
            now = self.clock()
            if self.run is None:
                self._begin_run()  # robots without state reports: positions mean it's running
            run = self.run
            run["last_pose"] = now
            run["raw"].append((x, y, th, now, self.robot["activity"] != "returning"))
            self.robot["pose_source"] = source
            n = len(run["raw"])
            reference = self._has_reference()
            if reference and self.settings["align"]:
                if n in ALIGN_AT or (n > ALIGN_AT[-1] and n % ALIGN_EVERY == 0):
                    self._align_run()
                if not run["aligned_once"]:
                    if n < ALIGN_AT[0]:
                        return  # hold the first few points until the run is aligned
                    self._align_run()
            if not run["flushed"]:
                run["flushed"] = True
                for p in run["raw"][:-1]:
                    self._apply_pose(*p)
            self._apply_pose(x, y, th, now, run["raw"][-1][4])

    def _has_reference(self):
        # Only a locked map is a trustworthy reference: aligning to a half-learned
        # map would drag new areas onto the part that is already known.
        return self.map.locked and self.map.has_floor_plan()

    def _align_run(self):
        run = self.run
        pts = [(p[0], p[1]) for p in run["raw"]]
        tf, score = geo.align(pts, self.map.reference_test())
        run["aligned_once"] = True
        run["score"] = round(score, 2)
        if score < 0.5 and len(pts) >= ALIGN_AT[1]:
            self.notice = (f"Only {int(score * 100)}% of this run lines up with the map - the dock may "
                           "have moved or the map needs editing")
        run["tf"] = tf

    def _apply_pose(self, x, y, th, now, cleaning):
        run = self.run
        mx, my = geo.transform(x, y, run["tf"])
        mth = th + run["tf"][2]
        prev = run["prev"]
        if prev is not None and math.hypot(mx - prev[0], my - prev[1]) <= MAX_JUMP_M:
            learn = not self.map.locked or self.settings["keep_learning"]
            self.map.sweep(prev[0], prev[1], mx, my, now, self.clean_radius, self.body_radius,
                           learn=learn, clean=cleaning)
        run["prev"] = (mx, my)
        if not run["trail"] or math.hypot(mx - run["trail"][-1][0], my - run["trail"][-1][1]) > 0.05:
            run["trail"].append([round(mx, 3), round(my, 3)])
        if self.map.has_floor_plan():
            run["inside" if self.map.is_floor(mx, my) else "outside"] += 1
        self.robot["pose"] = [round(mx, 3), round(my, 3), round(mth, 3)]
        self._changed()

    def _begin_run(self):
        self.run = {"started": self.clock(), "last_pose": self.clock(), "raw": [], "trail": [],
                    "tf": (0.0, 0.0, 0.0), "aligned_once": False, "flushed": False,
                    "prev": None, "score": None, "inside": 0, "outside": 0}
        self.notice = "Cleaning - " + ("learning the layout" if not self.map.locked
                                       else "tracking against the reference map")

    def _end_run(self, reason):
        run = self.run
        self.run = None
        if run is None:
            return
        if not run["flushed"]:
            # short run that ended before it could be aligned: apply as reported
            run["flushed"] = True
            self.run = run
            for p in run["raw"]:
                self._apply_pose(*p)
            self.run = None
        now = self.clock()
        summary = {"started": run["started"], "ended": now, "points": len(run["raw"]),
                   "align_score": run["score"], "reason": reason,
                   "offset": [round(v, 3) for v in run["tf"]]}
        self.map.last_trail = _downsample(run["trail"], 3000)
        self.last_run_end = now
        if len(run["raw"]) >= 5 and not self.map.locked:
            before = self.map.floor_area()
            after = self.map.rebuild()
            growth = (after - before) / before if before > 0 else 1.0
            summary["floor_m2"] = after
            summary["growth"] = round(growth, 3)
            learned = sum(1 for r in self.map.runs if "growth" in r) + 1
            if self.settings["auto_lock"] and learned >= 2 and growth < LOCK_GROWTH and after > 0:
                self.map.locked = True
                self.notice = (f"Map complete ({after} m²) - locked as the reference. "
                               "Edit or unlock it any time.")
            else:
                self.notice = (f"Run finished - map now covers {after} m²"
                               + (" (still learning)" if not self.map.locked else ""))
        else:
            self.notice = "Run finished"
        self.map.runs.append(summary)
        self._changed()
        self.save(force=True)

    # -- robot commands ---------------------------------------------------------------

    def command(self, verb, mode=None):
        if verb not in ("clean", "pause", "resume", "dock", "stop", "find"):
            raise ValueError("unknown command")
        params = None
        if verb == "clean":
            verb = "start"
            if mode in ("vacuum", "mop") and self.mode_select:
                params = mop_params(mode, self.settings["mop_wetness"])
        try:
            self.link.command(verb, params)
        except RuntimeError as exc:
            raise ValueError(str(exc)) from exc
        with self.lock:
            self.notice = {"start": "Start sent", "pause": "Pause sent", "resume": "Resume sent",
                           "dock": "Sent home to the dock", "stop": "Stop sent",
                           "find": "Robot is playing a sound"}[verb]

    def check_auto_start(self):
        with self.lock:
            pct = float(self.settings["auto_start_percent"])
            if pct <= 0 or not self.map.locked or self.run is not None:
                return False
            if self.robot["activity"] != "docked" or (self.robot["battery"] or 0) < 80:
                return False
            if self.clock() - self.last_run_end < float(self.settings["auto_start_min_hours"]) * 3600:
                return False
            stats = self.map.stats(self.clock(), self.stale_after)
            if stats["percent"] is None or 100.0 - stats["percent"] < pct:
                return False
            self.notice = f"Auto-start: {100 - stats['percent']:.0f}% of the floor needs cleaning"
        try:
            self.link.command("start", None)
        except RuntimeError:
            return False
        return True

    # -- map editing -----------------------------------------------------------------

    def add_shape(self, kind, points, name=None):
        with self.lock:
            el_id = self.map.add_element(kind, points, source="manual", name=name)
            self.notice = f"Added {kind}"
            self._changed()
            return el_id

    def add_walked_shape(self, kind, path, gap=0.3, square=True, name=None):
        """Outline from a path walked with the phone (see geometry.walk_to_polygon)."""
        if kind not in ("floor", "obstacle"):
            raise ValueError("kind must be floor or obstacle")
        if not isinstance(path, list) or not 3 <= len(path) <= 5000:
            raise ValueError("a walk needs between 3 and 5000 points")
        pts = []
        for p in path:
            if not (isinstance(p, (list, tuple)) and len(p) == 2):
                raise ValueError("path points must be [x, y] pairs")
            x, y = float(p[0]), float(p[1])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError("path point out of range")
            pts.append((x, y))
        gap = float(gap)
        if not 0 <= gap <= 1.5:
            raise ValueError("gap must be between 0 and 1.5 m")
        poly = geo.walk_to_polygon(pts, gap=gap, kind=kind, square=bool(square))
        if poly is None:
            raise ValueError("that walk doesn't enclose an area - walk all the way round and back to the start")
        return self.add_shape(kind, [list(p) for p in poly], name=name)

    def update_shape(self, el_id, points=None, kind=None, name=None):
        with self.lock:
            self.map.update_element(el_id, points, kind, name)
            self.notice = "Shape updated"
            self._changed()

    def delete_shape(self, el_id):
        with self.lock:
            self.map.delete_element(el_id)
            self.notice = "Shape deleted"
            self._changed()

    def rebuild(self):
        with self.lock:
            area = self.map.rebuild()
            self.notice = f"Floor plan regenerated from {self.map.stats(0, 0)['explored_m2']} m² explored ({area} m² floor)"
            self._changed()

    def set_locked(self, locked):
        with self.lock:
            if locked and not self.map.has_floor_plan():
                raise ValueError("draw or learn a floor area before locking the map")
            self.map.locked = bool(locked)
            self.notice = ("Map locked - it is now the robot's reference" if locked
                           else "Map unlocked - the next runs will extend it")
            self._changed()

    def reset_coverage(self):
        with self.lock:
            self.map.reset_coverage()
            self.notice = "Cleaning history cleared - every spot needs cleaning again"
            self._changed()

    def erase_map(self):
        with self.lock:
            self.map.erase()
            self.notice = "Map erased - it will be relearned on the next clean"
            self._changed()

    def update_settings(self, values):
        with self.lock:
            for key, val in values.items():
                if key not in DEFAULT_SETTINGS:
                    raise ValueError(f"unknown setting {key!r}")
                default = DEFAULT_SETTINGS[key]
                if isinstance(default, bool):
                    val = bool(val)
                elif isinstance(default, int):
                    val = int(val)
                    if key == "mop_wetness" and not 1 <= val <= 3:
                        raise ValueError("mop_wetness must be 1, 2 or 3")
                else:
                    val = float(val)
                    if not math.isfinite(val) or val < -360 or val > 24 * 365:
                        raise ValueError(f"{key} out of range")
                    if key != "view_rotation" and val < 0:
                        raise ValueError(f"{key} must not be negative")
                self.settings[key] = val
            self.notice = "Settings saved"
            self._changed()

    # -- read model for the GUI -----------------------------------------------------------

    def snapshot(self, have_map=None, have_cov=None):
        with self.lock:
            now = self.clock()
            m = self.map
            key = (m.version, m.cov_version, int(now // 60), self.settings["stale_hours"])
            if self._cache.get("key") != key:
                self._cache = {
                    "key": key,
                    "stats": m.stats(now, self.stale_after),
                    "missed": m.missed_spots(now, self.stale_after),
                    "raster": m.coverage_raster(now, self.stale_after),
                }
            cov_tag = f"{m.cov_version}:{key[2]}:{key[3]}"
            out = {
                "map_version": m.version,
                "cov_version": cov_tag,
                "locked": m.locked,
                "learning": not m.locked,
                "has_floor_plan": m.has_floor_plan(),
                "stats": self._cache["stats"],
                "missed": self._cache["missed"],
                "robot": dict(self.robot),
                "running": self.run is not None,
                "trail": _downsample(self.run["trail"], 2000) if self.run else m.last_trail,
                "notice": self.notice,
                "settings": dict(self.settings),
                "runs": len(m.runs),
                "capabilities": self.link.capabilities(),
            }
            if have_map != str(m.version):
                out["elements"] = [dict(el) for el in m.elements]
            if have_cov != cov_tag:
                out["coverage"] = self._cache["raster"]
            return out
