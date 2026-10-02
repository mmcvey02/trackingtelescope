"""Command line entry point: ``python -m roomba_mapper``."""

import argparse
import signal
import sys
import threading

from .controller import Controller
from .map_store import MapStore
from .robot import RoombaOI, SimulatedRobot
from .server import lan_address, make_server


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="roomba_mapper",
        description="Persistent cleaning map for a Roomba with a phone-friendly web GUI.")
    p.add_argument("--map", default="roomba_map.json", help="map file (created if missing)")
    p.add_argument("--host", default="0.0.0.0", help="address to listen on (default: all)")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--pin", help="require this PIN in the app (recommended on shared Wi-Fi)")
    p.add_argument("--robot", choices=("sim", "oi"), default="sim",
                   help="'sim' for the built-in simulator, 'oi' for a real Roomba via serial")
    p.add_argument("--serial", default="/dev/ttyUSB0", help="serial port for --robot oi")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--width", type=int, default=20, help="cells across (new maps only)")
    p.add_argument("--height", type=int, default=15, help="cells down (new maps only)")
    p.add_argument("--cell-cm", type=float, default=30, help="cell size in cm (new maps only)")
    p.add_argument("--tick", type=float, default=0.25, help="pause between moves, seconds")
    p.add_argument("--sim-seed", type=int, default=7, help="furniture layout for the simulator")
    p.add_argument("--auto-start", action="store_true", help="start cleaning immediately")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    store = MapStore(args.map)
    doc = store.load()
    width = doc["map"]["width"] if doc else args.width
    height = doc["map"]["height"] if doc else args.height
    cell_cm = doc["map"].get("cell_cm", args.cell_cm) if doc else args.cell_cm
    dock = tuple(doc["map"].get("dock", (0, 0))) if doc else (0, 0)

    if args.robot == "oi":
        robot = RoombaOI(args.serial, cell_cm=cell_cm, baud=args.baud)
    else:
        robot = SimulatedRobot.random_room(width, height, seed=args.sim_seed, keep_clear=(dock,))

    controller = Controller(store, robot, default_size=(args.width, args.height),
                            cell_cm=args.cell_cm, tick_s=args.tick)
    controller.start()
    if args.auto_start:
        controller.set_mode("auto")

    server = make_server(controller, args.host, args.port, args.pin)
    shown = lan_address() if args.host in ("0.0.0.0", "") else args.host
    print(f"Roomba mapper running with {robot.name} robot, map file {store.path}")
    print(f"Open on your phone (same Wi-Fi): http://{shown}:{args.port}/")
    if args.pin:
        print("PIN protection is on.")
    print("Press Ctrl+C to stop.")

    def _stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _stop)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        controller.shutdown()
        robot.close()
        print("Map saved. Bye.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
