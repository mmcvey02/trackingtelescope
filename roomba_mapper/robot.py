"""Robot drivers.

The controller thinks in grid cells and compass headings (N is "up" on the
map). A driver turns that into motion: ``move(direction)`` drives the robot
one cell and returns False if it bumped into something instead.

* SimulatedRobot - a virtual vacuum in a virtual room, for trying the app
  without hardware (and for tests).
* RoombaOI - a real iRobot Roomba / Create using the iRobot Open Interface
  over a serial cable (needs ``pyserial``).
"""

import random
import struct
import threading
import time

from .planner import DIRS, HEADINGS


class RobotDriver:
    name = "base"

    def reset_pose(self, x, y, heading="E"):
        """Tell the driver where the robot physically is (manual correction)."""

    def resize_world(self, width, height):
        """Called when the map is resized."""

    def move(self, direction):
        raise NotImplementedError

    def set_vacuum(self, on):
        pass

    def dock(self):
        pass

    def stop(self):
        pass

    def close(self):
        pass


class SimulatedRobot(RobotDriver):
    name = "simulator"

    def __init__(self, width, height, obstacles=(), start=(0, 0), heading="E", step_delay=0.0):
        self.width = width
        self.height = height
        self.obstacles = set(map(tuple, obstacles))
        self.x, self.y = start
        self.heading = heading
        self.step_delay = step_delay
        self.vacuum = False
        self.docked = False

    @classmethod
    def random_room(cls, width, height, seed=None, keep_clear=((0, 0),), **kwargs):
        """A room with a few randomly placed pieces of furniture."""
        rng = random.Random(seed)
        keep = set(map(tuple, keep_clear))
        obstacles = set()
        for _ in range(max(1, (width * height) // 60)):
            w, h = rng.randint(1, max(1, width // 5)), rng.randint(1, max(1, height // 5))
            ox, oy = rng.randint(1, max(1, width - w - 1)), rng.randint(1, max(1, height - h - 1))
            block = {(x, y) for x in range(ox, ox + w) for y in range(oy, oy + h)}
            if not block & keep:
                obstacles |= block
        start = next(iter(keep))
        return cls(width, height, obstacles, start=start, **kwargs)

    def reset_pose(self, x, y, heading="E"):
        self.x, self.y, self.heading = x, y, heading

    def resize_world(self, width, height):
        self.width, self.height = width, height
        self.obstacles = {(x, y) for x, y in self.obstacles if x < width and y < height}

    def move(self, direction):
        if self.step_delay:
            time.sleep(self.step_delay)
        self.heading = direction
        self.docked = False
        dx, dy = DIRS[direction]
        nx, ny = self.x + dx, self.y + dy
        if not (0 <= nx < self.width and 0 <= ny < self.height) or (nx, ny) in self.obstacles:
            return False
        self.x, self.y = nx, ny
        return True

    def set_vacuum(self, on):
        self.vacuum = bool(on)

    def dock(self):
        self.vacuum = False
        self.docked = True


class RoombaOI(RobotDriver):
    """iRobot Open Interface driver (Roomba 500/600/700/800/900 series, Create 2).

    Positioning is dead reckoning from the wheel odometry (distance/angle
    sensor packets) so it drifts over time; use "Set robot position" in the
    app to correct it, or send the robot home to re-zero on the dock.
    """

    name = "roomba-oi"

    OP_START, OP_SAFE, OP_DRIVE, OP_MOTORS, OP_SENSORS, OP_SEEK_DOCK = 128, 131, 137, 138, 142, 143
    PKT_BUMPS, PKT_DISTANCE, PKT_ANGLE = 7, 19, 20
    STRAIGHT = 0x8000
    MOTORS_CLEAN = 0b111  # side brush, vacuum, main brush

    def __init__(self, port, cell_cm=30, speed_mm_s=200, turn_mm_s=120, baud=115200, timeout_s=10.0):
        try:
            import serial  # pyserial
        except ImportError as exc:  # pragma: no cover - depends on environment
            raise RuntimeError("RoombaOI needs pyserial: pip install pyserial") from exc
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.5)
        self.cell_mm = cell_cm * 10.0
        self.speed = int(speed_mm_s)
        self.turn_speed = int(turn_mm_s)
        self.timeout_s = timeout_s
        self.heading = "E"
        self.vacuum = False
        self._io = threading.Lock()
        self._send(self.OP_START)
        time.sleep(0.1)
        self._send(self.OP_SAFE)
        time.sleep(0.1)

    # -- low level ------------------------------------------------------------

    def _send(self, *data):
        with self._io:
            self.ser.write(bytes(data))

    def _drive(self, velocity, radius):
        v = struct.pack(">h", max(-500, min(500, int(velocity))))
        r = struct.pack(">H", radius & 0xFFFF)
        self._send(self.OP_DRIVE, *v, *r)

    def _sensor(self, packet, size, fmt):
        with self._io:
            self.ser.reset_input_buffer()
            self.ser.write(bytes((self.OP_SENSORS, packet)))
            raw = self.ser.read(size)
        if len(raw) != size:
            raise IOError(f"no reply for sensor packet {packet}")
        return struct.unpack(fmt, raw)[0]

    def _bumped(self):
        return bool(self._sensor(self.PKT_BUMPS, 1, "B") & 0b11)

    def _distance(self):
        return self._sensor(self.PKT_DISTANCE, 2, ">h")

    def _angle(self):
        return self._sensor(self.PKT_ANGLE, 2, ">h")

    # -- motion ---------------------------------------------------------------

    def _turn_to(self, direction):
        delta = (HEADINGS.index(direction) - HEADINGS.index(self.heading)) % 4
        if delta == 0:
            return
        # heading index increases clockwise; OI angle is positive counter-clockwise
        target = {1: -90, 2: 180, 3: 90}[delta]
        self._angle()  # reset the accumulator
        self._drive(self.turn_speed, 0xFFFF if target < 0 else 0x0001)
        turned, deadline = 0, time.time() + self.timeout_s
        while abs(turned) < abs(target) and time.time() < deadline:
            time.sleep(0.02)
            turned += self._angle()
        self._drive(0, self.STRAIGHT)
        self.heading = direction

    def _straight(self, velocity, distance_mm):
        """Drive straight; returns distance travelled (negative if aborted by bump)."""
        self._distance()
        self._drive(velocity, self.STRAIGHT)
        travelled, deadline = 0, time.time() + self.timeout_s
        try:
            while abs(travelled) < distance_mm and time.time() < deadline:
                time.sleep(0.02)
                travelled += self._distance()
                if velocity > 0 and self._bumped():
                    return -abs(travelled)
        finally:
            self._drive(0, self.STRAIGHT)
        return abs(travelled)

    def move(self, direction):
        self._turn_to(direction)
        travelled = self._straight(self.speed, self.cell_mm)
        if travelled < 0:
            # back up to the centre of the cell we came from
            self._straight(-self.speed, abs(travelled) + 10)
            return False
        return True

    def reset_pose(self, x, y, heading="E"):
        self.heading = heading

    def set_vacuum(self, on):
        self.vacuum = bool(on)
        self._send(self.OP_MOTORS, self.MOTORS_CLEAN if on else 0)

    def stop(self):
        self._drive(0, self.STRAIGHT)

    def dock(self):
        self.set_vacuum(False)
        self._send(self.OP_SEEK_DOCK)
        self.heading = "E"

    def close(self):
        try:
            self.stop()
            self.set_vacuum(False)
        finally:
            self.ser.close()
