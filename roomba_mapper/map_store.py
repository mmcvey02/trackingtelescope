"""Persistent occupancy/coverage grid for the robot vacuum.

The floor is divided into square cells. Each cell has a state (unknown,
dirty, clean, obstacle, no-go) plus the time it was last cleaned, so the
robot always knows where it has been and where it still needs to go, even
across restarts.
"""

import json
import os
import tempfile
import threading
import time

UNKNOWN = 0   # never visited; assumed to be floor that needs cleaning
DIRTY = 1     # known floor that needs cleaning
CLEAN = 2     # cleaned (see cleaned_at for when)
OBSTACLE = 3  # bumped into / marked as furniture or wall
NOGO = 4      # user-forbidden area (rugs, pet bowls, cables, ...)

STATE_NAMES = {
    UNKNOWN: "unknown",
    DIRTY: "dirty",
    CLEAN: "clean",
    OBSTACLE: "obstacle",
    NOGO: "nogo",
}
NAME_TO_STATE = {name: state for state, name in STATE_NAMES.items()}

MIN_SIZE = 2
MAX_SIZE = 200
FORMAT_VERSION = 1


class GridMap:
    def __init__(self, width, height, cell_cm=30, dock=(0, 0)):
        width, height = int(width), int(height)
        if not (MIN_SIZE <= width <= MAX_SIZE and MIN_SIZE <= height <= MAX_SIZE):
            raise ValueError(f"map size must be between {MIN_SIZE} and {MAX_SIZE} cells")
        self.width = width
        self.height = height
        self.cell_cm = float(cell_cm)
        self.cells = [UNKNOWN] * (width * height)
        self.cleaned_at = [0.0] * (width * height)
        self.clean_count = [0] * (width * height)
        self.dock = (0, 0)
        self.set_dock(*dock)

    # -- basic access -------------------------------------------------------

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def _idx(self, x, y):
        if not self.in_bounds(x, y):
            raise IndexError(f"cell ({x}, {y}) outside {self.width}x{self.height} map")
        return y * self.width + x

    def get(self, x, y):
        return self.cells[self._idx(x, y)]

    def set(self, x, y, state, now=None):
        if state not in STATE_NAMES:
            raise ValueError(f"unknown cell state {state!r}")
        i = self._idx(x, y)
        self.cells[i] = state
        if state == CLEAN:
            self.cleaned_at[i] = time.time() if now is None else now
        elif state in (UNKNOWN, DIRTY):
            self.cleaned_at[i] = 0.0

    def mark_cleaned(self, x, y, now=None):
        i = self._idx(x, y)
        self.cells[i] = CLEAN
        self.cleaned_at[i] = time.time() if now is None else now
        self.clean_count[i] += 1

    def set_dock(self, x, y):
        x, y = int(x), int(y)
        if not self.in_bounds(x, y):
            raise ValueError("dock must be inside the map")
        self.dock = (x, y)
        if self.get(x, y) in (OBSTACLE, NOGO):
            self.set(x, y, DIRTY)

    # -- derived state ------------------------------------------------------

    def effective(self, x, y, now, stale_after=0):
        """State as the planner sees it: old cleanings count as dirty again."""
        i = self._idx(x, y)
        state = self.cells[i]
        if state == CLEAN and stale_after > 0 and now - self.cleaned_at[i] > stale_after:
            return DIRTY
        return state

    def needs_cleaning(self, x, y, now, stale_after=0):
        return self.effective(x, y, now, stale_after) in (UNKNOWN, DIRTY)

    def passable(self, x, y):
        return self.in_bounds(x, y) and self.cells[self._idx(x, y)] not in (OBSTACLE, NOGO)

    def stats(self, now, stale_after=0):
        counts = {name: 0 for name in STATE_NAMES.values()}
        for y in range(self.height):
            for x in range(self.width):
                counts[STATE_NAMES[self.effective(x, y, now, stale_after)]] += 1
        floor = counts["unknown"] + counts["dirty"] + counts["clean"]
        return {
            "total": self.width * self.height,
            "floor": floor,
            "cleaned": counts["clean"],
            "remaining": counts["unknown"] + counts["dirty"],
            "unknown": counts["unknown"],
            "obstacles": counts["obstacle"],
            "nogo": counts["nogo"],
            "percent": round(100.0 * counts["clean"] / floor, 1) if floor else 100.0,
            "area_m2": round(counts["clean"] * (self.cell_cm / 100.0) ** 2, 2),
        }

    # -- bulk operations ----------------------------------------------------

    def reset_pass(self):
        """Start a fresh cleaning pass but keep the learned layout."""
        for i, state in enumerate(self.cells):
            if state == CLEAN:
                self.cells[i] = DIRTY
                self.cleaned_at[i] = 0.0

    def clear_all(self):
        n = self.width * self.height
        self.cells = [UNKNOWN] * n
        self.cleaned_at = [0.0] * n
        self.clean_count = [0] * n

    def resize(self, width, height):
        new = GridMap(width, height, self.cell_cm)
        for y in range(min(self.height, new.height)):
            for x in range(min(self.width, new.width)):
                src, dst = self._idx(x, y), new._idx(x, y)
                new.cells[dst] = self.cells[src]
                new.cleaned_at[dst] = self.cleaned_at[src]
                new.clean_count[dst] = self.clean_count[src]
        dx, dy = self.dock
        new.set_dock(min(dx, new.width - 1), min(dy, new.height - 1))
        self.__dict__.update(new.__dict__)

    # -- serialisation ------------------------------------------------------

    def to_dict(self):
        return {
            "width": self.width,
            "height": self.height,
            "cell_cm": self.cell_cm,
            "dock": list(self.dock),
            "cells": "".join(str(c) for c in self.cells),
            "cleaned_at": self.cleaned_at,
            "clean_count": self.clean_count,
        }

    @classmethod
    def from_dict(cls, data):
        grid = cls(data["width"], data["height"], data.get("cell_cm", 30))
        n = grid.width * grid.height
        cells = data.get("cells", "")
        if len(cells) != n:
            raise ValueError("cell data does not match map size")
        grid.cells = [int(c) for c in cells]
        if any(c not in STATE_NAMES for c in grid.cells):
            raise ValueError("invalid cell state in saved map")
        grid.cleaned_at = [float(t) for t in data.get("cleaned_at", [0.0] * n)][:n]
        grid.clean_count = [int(c) for c in data.get("clean_count", [0] * n)][:n]
        grid.cleaned_at += [0.0] * (n - len(grid.cleaned_at))
        grid.clean_count += [0] * (n - len(grid.clean_count))
        grid.set_dock(*data.get("dock", (0, 0)))
        return grid


class MapStore:
    """Reads and atomically writes the JSON document holding the map."""

    def __init__(self, path):
        self.path = os.path.abspath(path)
        self._lock = threading.Lock()

    def load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                doc = json.load(fh)
        except FileNotFoundError:
            return None
        if doc.get("format") != FORMAT_VERSION:
            raise ValueError(f"{self.path}: unsupported map format {doc.get('format')!r}")
        return doc

    def save(self, doc):
        doc = dict(doc, format=FORMAT_VERSION, saved_at=time.time())
        directory = os.path.dirname(self.path) or "."
        os.makedirs(directory, exist_ok=True)
        with self._lock:
            fd, tmp = tempfile.mkstemp(prefix=".roomba-map-", suffix=".tmp", dir=directory)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as fh:
                    json.dump(doc, fh, separators=(",", ":"))
                    fh.flush()
                    os.fsync(fh.fileno())
                os.replace(tmp, self.path)
            except BaseException:
                if os.path.exists(tmp):
                    os.unlink(tmp)
                raise
