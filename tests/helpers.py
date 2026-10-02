"""Shared test helpers."""


class DirectSimLink:
    """Drives a SimRoomba synchronously (no threads) for deterministic tests."""

    profile = {"name": "Simulated Combo Essential", "mop": True, "body_radius": 0.17,
               "clean_radius": 0.17}
    name = "sim"

    def __init__(self, robot):
        self.r = robot
        self.on_state = self.on_pose = self.on_link = None
        self.sent = []

    def start(self):
        pass

    def close(self):
        pass

    def capabilities(self):
        return {"connected": True, "can_drive": False}

    def command(self, verb, params=None):
        self.sent.append((verb, params))
        self.r.command(verb, params)

    def run_mission(self, clock, drift=None, pose_every=1.0):
        self.r.command("start")
        if drift is not None:
            self.r.drift = drift
        self.on_state(self.r.reported())
        version, acc = self.r.version, 0.0
        while self.r.phase != "charge":
            self.r.step(0.1)
            clock[0] += 0.1
            acc += 0.1
            if self.r.version != version:
                version = self.r.version
                self.on_state(self.r.reported())
            if acc >= pose_every and self.r.phase != "charge":
                acc = 0.0
                self.on_pose(*self.r.reported_pose(), "sim")
        self.on_state(self.r.reported())
