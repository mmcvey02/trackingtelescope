"""Ties the map, the planner and the robot together.

Modes:
  idle      - robot parked, nothing happens
  auto      - robot cleans every cell that needs it, then returns to the dock
  manual    - robot only moves when told to from the app (D-pad / tap to go)
  returning - robot is driving back to the dock
"""

import threading
import time

from . import planner
from .map_store import DIRTY, NAME_TO_STATE, NOGO, OBSTACLE, STATE_NAMES, UNKNOWN, GridMap

MODES = ("idle", "auto", "manual", "returning")
DEFAULT_SETTINGS = {
    "stale_hours": 24.0,        # a cleaned cell needs cleaning again after this long (0 = never)
    "clean_in_manual": True,    # run the vacuum while driving manually
}
SAVE_INTERVAL_S = 2.0


class Controller:
    def __init__(self, store, robot, default_size=(20, 15), cell_cm=30, tick_s=0.3, clock=time.time):
        self.store = store
        self.robot = robot
        self.tick_s = tick_s
        self.clock = clock
        self.lock = threading.RLock()
        self.settings = dict(DEFAULT_SETTINGS)

        doc = store.load()
        if doc:
            self.grid = GridMap.from_dict(doc["map"])
            pose = doc.get("robot", {})
            self.pose = (int(pose.get("x", self.grid.dock[0])), int(pose.get("y", self.grid.dock[1])))
            if not self.grid.in_bounds(*self.pose):
                self.pose = self.grid.dock
            self.heading = pose.get("heading", "E")
            self.settings.update({k: v for k, v in doc.get("settings", {}).items() if k in DEFAULT_SETTINGS})
            self.status = "Map loaded"
        else:
            self.grid = GridMap(default_size[0], default_size[1], cell_cm)
            self.pose = self.grid.dock
            self.heading = "E"
            self.status = "New map created"
        self.robot.reset_pose(*self.pose, self.heading)

        self.mode = "idle"
        self.path = []
        self.path_forced = False  # single manual step: try even if map says blocked
        self.version = 0
        self._dirty = True
        self._last_save = 0.0
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread = None

    # -- lifecycle ------------------------------------------------------------

    def start(self):
        self._thread = threading.Thread(target=self._run, name="roomba-controller", daemon=True)
        self._thread.start()

    def shutdown(self):
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=5)
        try:
            self.robot.stop()
            self.robot.set_vacuum(False)
        finally:
            self.save(force=True)

    def _run(self):
        while not self._stop.is_set():
            try:
                worked = self.step()
            except Exception as exc:  # keep the server alive if the robot misbehaves
                with self.lock:
                    self._set_mode("idle", f"Robot error: {exc}")
                worked = False
            self.save()
            if worked:
                time.sleep(self.tick_s)
            else:
                self._wake.wait(0.5)
                self._wake.clear()

    # -- persistence ----------------------------------------------------------

    def save(self, force=False):
        with self.lock:
            if not (force or self._dirty):
                return
            if not force and time.monotonic() - self._last_save < SAVE_INTERVAL_S:
                return
            doc = {
                "map": self.grid.to_dict(),
                "robot": {"x": self.pose[0], "y": self.pose[1], "heading": self.heading},
                "settings": self.settings,
            }
            self._dirty = False
            self._last_save = time.monotonic()
        self.store.save(doc)

    def _changed(self):
        self._dirty = True
        self.version += 1
        self._wake.set()

    @property
    def stale_after(self):
        return float(self.settings["stale_hours"]) * 3600.0

    # -- the main loop body ---------------------------------------------------

    def _set_mode(self, mode, status=None):
        self.mode = mode
        self.path = []
        self.path_forced = False
        if status:
            self.status = status
        if mode in ("idle", "returning"):
            self.robot.set_vacuum(False)
        elif mode == "auto" or self.settings["clean_in_manual"]:
            self.robot.set_vacuum(True)
        self._changed()

    def step(self):
        """Advance the robot by at most one cell. Returns True if it moved/tried to."""
        with self.lock:
            now = self.clock()
            if self.mode == "idle":
                return False
            if self.mode == "manual" and not self.path:
                return False
            if self.mode == "auto" and not self.path:
                path = planner.coverage_path(self.grid, self.pose, self.heading, now, self.stale_after)
                if path:
                    self.path = path
                    self.status = f"Cleaning - heading to {path[-1]}"
                else:
                    self._set_mode("returning", "All reachable floor is clean - returning to dock")
            if self.mode == "returning" and not self.path:
                if self.pose == self.grid.dock:
                    self.robot.dock()
                    self._set_mode("idle", "Docked - cleaning complete")
                    return False
                path = planner.path_to(self.grid, self.pose, self.grid.dock, self.heading)
                if not path:
                    self._set_mode("idle", "Cannot find a route to the dock")
                    return False
                self.path = path
            if not self.path:
                return False

            target = self.path[0]
            if not self.path_forced and not self.grid.passable(*target):
                self.path = []  # map changed under us; replan next tick
                self._changed()
                return True
            if self.grid.get(*target) == NOGO:
                self.path = []
                self.status = "That cell is a no-go zone"
                self._changed()
                return False
            direction = planner.direction_between(self.pose, target)
            mode_at_start = self.mode

        moved = self.robot.move(direction)  # may take a while on real hardware

        with self.lock:
            self.heading = direction
            if moved:
                self.pose = target
                cleaning = mode_at_start == "auto" or (
                    mode_at_start == "manual" and self.settings["clean_in_manual"])
                if cleaning:
                    self.grid.mark_cleaned(*target, now=self.clock())
                elif self.grid.get(*target) in (UNKNOWN, OBSTACLE):
                    self.grid.set(*target, DIRTY)  # we now know it is floor
                if self.path and self.path[0] == target:
                    self.path.pop(0)
                if self.mode == "manual" and not self.path:
                    self.status = f"Arrived at {target}"
            else:
                self.grid.set(*target, OBSTACLE)
                self.path = []
                self.status = f"Bumped into something at {target} - marked as obstacle"
            self.path_forced = False
            self._changed()
        return True

    # -- commands from the app ------------------------------------------------

    def set_mode(self, mode):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        with self.lock:
            labels = {
                "idle": "Stopped",
                "auto": "Automatic cleaning started",
                "manual": "Manual control - use the D-pad or tap the map",
                "returning": "Returning to dock",
            }
            if mode == "idle":
                self.robot.stop()
            self._set_mode(mode, labels[mode])
            if mode == "auto" and self.grid.needs_cleaning(*self.pose, self.clock(), self.stale_after):
                self.grid.mark_cleaned(*self.pose, now=self.clock())

    def drive(self, direction):
        """Manual: move one cell in a compass direction."""
        if direction not in planner.DIRS:
            raise ValueError("direction must be N, E, S or W")
        with self.lock:
            if self.mode != "manual":
                self._set_mode("manual")
            dx, dy = planner.DIRS[direction]
            target = (self.pose[0] + dx, self.pose[1] + dy)
            if not self.grid.in_bounds(*target):
                self.status = "Edge of the map"
            elif self.grid.get(*target) == NOGO:
                self.status = "That cell is a no-go zone"
            else:
                self.path = [target]
                self.path_forced = True
                self.status = f"Driving {direction}"
            self._changed()

    def goto(self, x, y):
        with self.lock:
            if not self.grid.in_bounds(x, y):
                raise ValueError("cell outside the map")
            path = planner.path_to(self.grid, self.pose, (x, y), self.heading)
            if self.mode != "manual":
                self._set_mode("manual")
            if path is None:
                self.status = f"No known route to ({x}, {y})"
            else:
                self.path = path
                self.path_forced = False
                self.status = f"Driving to ({x}, {y})"
            self._changed()

    def set_cells(self, cells, state_name):
        if state_name not in NAME_TO_STATE:
            raise ValueError(f"state must be one of {sorted(NAME_TO_STATE)}")
        state = NAME_TO_STATE[state_name]
        with self.lock:
            now = self.clock()
            changed = 0
            for x, y in cells:
                x, y = int(x), int(y)
                if not self.grid.in_bounds(x, y):
                    continue
                if state in (OBSTACLE, NOGO) and ((x, y) == self.pose or (x, y) == self.grid.dock):
                    continue  # the robot / dock can't be inside a wall
                self.grid.set(x, y, state, now=now)
                changed += 1
            if changed:
                if not self.path_forced:
                    self.path = []
                self.status = f"Marked {changed} cell(s) as {state_name}"
                self._changed()
            return changed

    def set_dock(self, x, y):
        with self.lock:
            self.grid.set_dock(x, y)
            self.status = f"Dock set to ({x}, {y})"
            self._changed()

    def set_pose(self, x, y, heading=None):
        with self.lock:
            if not self.grid.in_bounds(x, y):
                raise ValueError("cell outside the map")
            if heading is not None and heading not in planner.DIRS:
                raise ValueError("heading must be N, E, S or W")
            self.pose = (x, y)
            self.heading = heading or self.heading
            if self.grid.get(x, y) in (OBSTACLE, NOGO):
                self.grid.set(x, y, DIRTY)
            self.robot.reset_pose(x, y, self.heading)
            self.path = []
            self.status = f"Robot position set to ({x}, {y})"
            self._changed()

    def reset(self, scope):
        with self.lock:
            if scope == "pass":
                self.grid.reset_pass()
                self.status = "New cleaning pass - layout kept"
            elif scope == "all":
                self.grid.clear_all()
                self.status = "Map erased"
            else:
                raise ValueError("scope must be 'pass' or 'all'")
            self.path = []
            self._changed()

    def resize(self, width, height):
        with self.lock:
            self._set_mode("idle")
            self.grid.resize(width, height)
            self.robot.resize_world(self.grid.width, self.grid.height)
            if not self.grid.in_bounds(*self.pose):
                self.pose = self.grid.dock
                self.robot.reset_pose(*self.pose, self.heading)
            self.status = f"Map resized to {self.grid.width}x{self.grid.height}"
            self._changed()

    def update_settings(self, values):
        with self.lock:
            if "stale_hours" in values:
                hours = float(values["stale_hours"])
                if not 0 <= hours <= 24 * 365:
                    raise ValueError("stale_hours out of range")
                self.settings["stale_hours"] = hours
            if "clean_in_manual" in values:
                self.settings["clean_in_manual"] = bool(values["clean_in_manual"])
                if self.mode == "manual":
                    self.robot.set_vacuum(self.settings["clean_in_manual"])
            self.status = "Settings saved"
            self._changed()

    # -- read model for the GUI -----------------------------------------------

    def snapshot(self):
        with self.lock:
            now = self.clock()
            g = self.grid
            stale = self.stale_after
            cells = "".join(
                str(g.effective(x, y, now, stale)) for y in range(g.height) for x in range(g.width)
            )
            return {
                "version": self.version,
                "width": g.width,
                "height": g.height,
                "cell_cm": g.cell_cm,
                "cells": cells,
                "legend": STATE_NAMES,
                "dock": list(g.dock),
                "robot": {"x": self.pose[0], "y": self.pose[1], "heading": self.heading,
                          "driver": self.robot.name},
                "mode": self.mode,
                "status": self.status,
                "path": [list(c) for c in self.path],
                "stats": g.stats(now, stale),
                "settings": dict(self.settings),
            }

