"""A self-driving virtual Roomba in a virtual home, for trying the app.

It behaves like a Wi-Fi Roomba from the mapper's point of view: it decides
where to go by itself, reports state (``cleanMissionStatus``, ``batPct``...)
and its position in its own dock-origin frame, and accepts
start / pause / resume / stop / dock. Positions carry a little noise and a
per-run heading drift, so the map alignment has something to correct.
"""

import heapq
import math
import random
import threading
import time

from . import geometry as geo

# A two-room flat: living room with kitchen island and sofa, a doorway to a
# bedroom with a bed and wardrobe. The dock sits against the living room's
# west wall at the origin, facing +x.
DEFAULT_HOME = {
    "floor": [[(-0.2, -1.5), (4.8, -1.5), (4.8, 2.5), (4.6, 2.5), (4.6, 2.6), (5.6, 2.6),
               (5.6, 6.0), (2.0, 6.0), (2.0, 2.6), (3.6, 2.6), (3.6, 2.5), (-0.2, 2.5)]],
    "obstacles": [
        [(0.8, 1.6), (2.8, 1.6), (2.8, 2.5), (0.8, 2.5)],     # sofa
        [(2.5, -0.6), (3.7, -0.6), (3.7, 0.2), (2.5, 0.2)],   # kitchen island
        [(0.6, -1.0), (0.95, -1.0), (0.95, -0.65), (0.6, -0.65)],  # plant pot
        [(3.0, 4.0), (4.6, 4.0), (4.6, 6.0), (3.0, 6.0)],     # bed
        [(2.0, 2.6), (2.6, 2.6), (2.6, 3.8), (2.0, 3.8)],     # wardrobe
    ],
}

GRID = 0.1
BODY_R = 0.17
LANE = 0.24


class SimHome:
    def __init__(self, layout=None):
        layout = layout or DEFAULT_HOME
        self.floor = [list(map(tuple, p)) for p in layout["floor"]]
        self.obstacles = [list(map(tuple, p)) for p in layout["obstacles"]]
        self.segments = []
        for poly in self.floor + self.obstacles:
            for k in range(len(poly)):
                self.segments.append((poly[k], poly[(k + 1) % len(poly)]))
        self._free_cells = None

    def free(self, x, y, r=BODY_R):
        if not any(geo.point_in_polygon(x, y, p) for p in self.floor):
            return False
        if any(geo.point_in_polygon(x, y, p) for p in self.obstacles):
            return False
        return all(geo.point_segment_distance((x, y), a, b) >= r for a, b in self.segments)

    def free_cells(self):
        """Grid cells the robot centre can occupy (cached)."""
        if self._free_cells is None:
            cells = set()
            for poly in self.floor:
                x0, y0, x1, y1 = geo.polygon_bounds(poly)
                for i in range(int(math.floor(x0 / GRID)), int(math.ceil(x1 / GRID))):
                    for j in range(int(math.floor(y0 / GRID)), int(math.ceil(y1 / GRID))):
                        if self.free((i + 0.5) * GRID, (j + 0.5) * GRID):
                            cells.add((i, j))
            # keep the area connected to the dock; tiny pockets are unreachable
            comps = geo.components(cells, eight=False)
            dock = _cell(0.0, 0.0)
            main = next((c for c in comps if dock in c), max(comps, key=len))
            self._free_cells = set(main)
        return self._free_cells


def _cell(x, y):
    return int(math.floor(x / GRID)), int(math.floor(y / GRID))


def _centre(c):
    return (c[0] + 0.5) * GRID, (c[1] + 0.5) * GRID


class SimRoomba:
    """Robot 'firmware': call step(dt) to advance simulated time."""

    def __init__(self, home=None, speed=0.3, seed=None, drift_deg=2.0, noise_m=0.01,
                 mission_limit_s=3600.0):
        self.home = home or SimHome()
        self.speed = speed
        self.rng = random.Random(seed)
        self.drift_deg = drift_deg
        self.noise_m = noise_m
        self.mission_limit_s = mission_limit_s
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.phase, self.cycle = "charge", "none"
        self.battery = 100.0
        self.mission_time = 0.0
        self.missions = 0
        self.visited = set()
        self.plan = []
        self.trail = []          # true positions this mission, for the way home
        self.lane_dir = 1        # +1 / -1 along y between rows
        self.row_dir = 1         # +1 / -1 along x
        self.mode = "rows"
        self.no_progress = 0.0
        self.drift = (0.0, 0.0, 0.0)
        self.error = 0
        self.version = 0

    # -- commands --

    def command(self, verb, params=None):
        if verb in ("start", "clean"):
            if self.cycle == "none":
                self._begin_mission()
            elif self.phase in ("stop", "pause"):
                self.phase = "run"
        elif verb == "pause" and self.phase in ("run", "hmUsrDock", "hmPostMsn", "hmMidMsn"):
            self.phase = "stop"
        elif verb == "resume" and self.phase in ("stop", "pause") and self.cycle != "none":
            self.phase = "run"
        elif verb == "stop":
            self.phase, self.cycle = "stop", "none"
        elif verb == "dock" and self.phase != "charge":
            self.phase, self.cycle = "hmUsrDock", self.cycle if self.cycle != "none" else "dock"
        self.version += 1

    def _begin_mission(self):
        self.missions += 1
        self.phase, self.cycle = "run", "clean"
        self.mission_time = 0.0
        self.visited = set()
        self.plan = []
        self.trail = [(self.x, self.y)]
        self.mode = "rows"
        self.row_dir, self.lane_dir = 1, 1
        self.th = 0.0
        d = math.radians(self.rng.uniform(-self.drift_deg, self.drift_deg))
        self.drift = (self.rng.uniform(-0.03, 0.03), self.rng.uniform(-0.03, 0.03), d)

    # -- observation --

    def reported_pose(self):
        """Position as the robot itself believes it to be (with drift/noise)."""
        x, y = geo.transform(self.x, self.y, self.drift)
        n = self.noise_m
        return (x + self.rng.gauss(0, n), y + self.rng.gauss(0, n), self.th + self.drift[2])

    def reported(self):
        return {
            "batPct": int(self.battery),
            "cleanMissionStatus": {
                "cycle": self.cycle, "phase": self.phase, "error": self.error,
                "mssnM": int(self.mission_time // 60), "nMssn": self.missions,
                "sqft": int(len(self.visited) * GRID * GRID * 10.764),
            },
            "bin": {"present": True, "full": False},
            "dock": {"known": True},
            "pmaps": [{"simPmap1": "v1"}],
        }

    # -- motion --

    def step(self, dt):
        if self.phase == "charge":
            self.battery = min(100.0, self.battery + dt * 0.05)
            return
        if self.phase == "run":
            self.mission_time += dt
            self.battery = max(0.0, self.battery - dt * 0.012)
            if self.battery < 15 or self.mission_time > self.mission_limit_s:
                self.phase = "hmMidMsn" if self.battery < 15 else "hmPostMsn"
                self.version += 1
                return
            self._clean_step(dt)
        elif self.phase in ("hmUsrDock", "hmPostMsn", "hmMidMsn"):
            self._return_step(dt)

    def _mark(self):
        c = _cell(self.x, self.y)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                self.visited.add((c[0] + di, c[1] + dj))

    def _move_towards(self, tx, ty, dist):
        dx, dy = tx - self.x, ty - self.y
        d = math.hypot(dx, dy)
        if d < 1e-9:
            return True
        self.th = math.atan2(dy, dx)
        if d <= dist:
            self.x, self.y = tx, ty
            return True
        self.x += dx / d * dist
        self.y += dy / d * dist
        return False

    def _clean_step(self, dt):
        dist = self.speed * dt
        if self.plan:
            target = _centre(self.plan[0])
            if self._move_towards(*target, dist):
                self.plan.pop(0)
            self._after_move()
            return
        # boustrophedon rows along x
        nx = self.x + self.row_dir * dist
        if self.home.free(nx, self.y):
            before = len(self.visited)
            self.th = 0.0 if self.row_dir > 0 else math.pi
            self.x = nx
            self._after_move()
            self.no_progress = 0.0 if len(self.visited) > before else self.no_progress + dt
            if self.no_progress > 6.0:
                self._replan()
            return
        # bumped: shift one lane and turn around
        ny = self.y + self.lane_dir * LANE
        if self.home.free(self.x, ny) and self._clear_line(self.x, self.y, self.x, ny):
            self.y = ny
            self.row_dir *= -1
            self._after_move()
        else:
            self.lane_dir *= -1
            self._replan()

    def _clear_line(self, x0, y0, x1, y1):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
        return all(self.home.free(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n + 1))

    def _after_move(self):
        self._mark()
        self.trail.append((self.x, self.y))

    def _replan(self):
        """Drive to the nearest spot not yet cleaned (A*/Dijkstra on free cells)."""
        free = self.home.free_cells()
        start = _cell(self.x, self.y)
        if start not in free:
            start = min(free, key=lambda c: (c[0] - start[0]) ** 2 + (c[1] - start[1]) ** 2)
        dist = {start: 0}
        prev = {}
        heap = [(0, start)]
        goal = None
        while heap:
            d, c = heapq.heappop(heap)
            if d > dist[c]:
                continue
            if c not in self.visited:
                goal = c
                break
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (c[0] + di, c[1] + dj)
                if n in free and d + 1 < dist.get(n, 1e9):
                    dist[n] = d + 1
                    prev[n] = c
                    heapq.heappush(heap, (d + 1, n))
        self.no_progress = 0.0
        if goal is None:
            self.phase = "hmPostMsn"  # everything reachable is clean
            self.version += 1
            return
        path = [goal]
        while path[-1] in prev:
            path.append(prev[path[-1]])
        path.reverse()
        self.plan = path[1:] if len(path) > 1 else path
        self.row_dir = self.rng.choice((-1, 1))

    def _return_step(self, dt):
        dist = self.speed * 1.5 * dt
        while dist > 0 and self.trail:
            tx, ty = self.trail[-1]
            d = math.hypot(tx - self.x, ty - self.y)
            if d <= dist:
                self.x, self.y = tx, ty
                self.trail.pop()
                dist -= d
            else:
                self._move_towards(tx, ty, dist)
                dist = 0
        if not self.trail:
            self.x, self.y, self.th = 0.0, 0.0, 0.0
            self.phase, self.cycle = "charge", "none"
            self.version += 1


class SimLink:
    """Runs a SimRoomba in real time and exposes the same interface as WifiRoomba."""

    def __init__(self, robot=None, time_scale=10.0, pose_interval=1.0, tick=0.1, profile=None):
        from .wifi import profile_for
        self.robot = robot or SimRoomba()
        self.time_scale = time_scale
        self.pose_interval = pose_interval
        self.tick = tick
        self.profile = profile or profile_for(key="combo-essential")
        self.name = "Simulated " + self.profile["name"]
        self.on_state = None
        self.on_pose = None
        self.on_link = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.connected = True

    def start(self):
        threading.Thread(target=self._run, name="sim-roomba", daemon=True).start()

    def close(self):
        self._stop.set()

    def _run(self):
        if self.on_link:
            self.on_link(f"Connected to {self.name}", True)
        last_version = -1
        since_pose = 0.0
        while not self._stop.is_set():
            with self._lock:
                self.robot.step(self.tick)
                version = self.robot.version
                moving = self.robot.phase not in ("charge", "stop", "pause")
                pose = self.robot.reported_pose() if moving else None
                reported = self.robot.reported() if version != last_version else None
            if reported is not None:
                last_version = version
                if self.on_state:
                    self.on_state(reported)
            since_pose += self.tick
            if pose and since_pose >= self.pose_interval:
                since_pose = 0.0
                if self.on_pose:
                    self.on_pose(*pose, "sim")
            self._stop.wait(self.tick / self.time_scale)

    def command(self, verb, params=None):
        with self._lock:
            self.robot.command(verb, params)

    def capabilities(self):
        return {"connected": True, "pose_from_state": False, "rrtp": True,
                "has_mop": self.profile.get("mop", False), "can_drive": False}
