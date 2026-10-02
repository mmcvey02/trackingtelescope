"""Command line entry point: ``python -m roomba_mapper <command>``.

Commands:
  serve         run the mapper and the phone app (default)
  discover      find Wi-Fi Roombas on the network
  get-password  read the robot's local password (from the robot, or your iRobot account)
  probe         check what a robot supports (local connection, positions)
  cloud-probe   see what maps and cleaning history iRobot's servers hold for your robot
  emulate       pretend to be a Wi-Fi Roomba, for trying everything without one
"""

import argparse
import getpass
import json
import os
import signal
import sys
import threading
import time

DEFAULT_CONFIG = "roomba_config.json"


def load_config(path):
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh)
    except FileNotFoundError:
        return {}


def save_config(path, cfg):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)  # holds the robot password
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        json.dump(cfg, fh, indent=2)
    print(f"Saved robot connection details to {os.path.abspath(path)}")


def build_parser():
    p = argparse.ArgumentParser(prog="roomba_mapper", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd")

    s = sub.add_parser("serve", help="run the mapper and phone app")
    s.add_argument("--robot", choices=("sim", "wifi"), default=None,
                   help="'wifi' for a real robot (default when a config file exists), 'sim' to try it out")
    s.add_argument("--map", default="roomba_map.json", help="map file (created if missing)")
    s.add_argument("--config", default=DEFAULT_CONFIG, help="robot connection file from get-password")
    s.add_argument("--host", default="0.0.0.0", help="address the app listens on")
    s.add_argument("--port", type=int, default=8080)
    s.add_argument("--pin", help="require this PIN in the app (recommended on shared Wi-Fi)")
    s.add_argument("--ip", help="robot IP address")
    s.add_argument("--blid", help="robot id (BLID)")
    s.add_argument("--password", help="robot local password")
    s.add_argument("--model", help="force a model profile, e.g. rvg-y1 / combo-essential")
    s.add_argument("--mqtt-port", type=int, default=8883)
    s.add_argument("--no-tls", action="store_true", help="plain MQTT (emulator only)")
    s.add_argument("--pose-source", choices=("auto", "state", "rrtp"), default="auto")
    s.add_argument("--pose-units", choices=("mm", "cm", "m"), default="mm",
                   help="units of positions in state reports")
    s.add_argument("--pose-path", help="read the position from this state field, e.g. "
                   "cleanMissionStatus.pos (see `probe`)")
    s.add_argument("--pose-angle", choices=("deg", "rad"), default="deg",
                   help="angle units of --pose-path positions")
    s.add_argument("--rrtp-topic", default=None, help="topic for position requests (see `probe`)")
    s.add_argument("--rrtp-contype", choices=("local", "remote"), default=None)
    s.add_argument("--sim-speed", type=float, default=15.0, help="simulator time multiplier")
    s.add_argument("--sim-seed", type=int, default=None)

    d = sub.add_parser("discover", help="find robots on the network")
    d.add_argument("--timeout", type=float, default=4.0)
    d.add_argument("--target", default="255.255.255.255", help="broadcast address or robot IP")
    d.add_argument("--discovery-port", type=int, default=5678)

    g = sub.add_parser("get-password", help="read the robot's local password")
    g.add_argument("--ip", help="robot IP (for the on-robot method)")
    g.add_argument("--port", type=int, default=8883)
    g.add_argument("--cloud", action="store_true",
                   help="use your iRobot account instead (needed for newer models like the RVG-Y1)")
    g.add_argument("--email", help="iRobot account e-mail (with --cloud)")
    g.add_argument("--country", default="US", help="account country code (with --cloud)")
    g.add_argument("--config", default=DEFAULT_CONFIG)
    g.add_argument("--no-save", action="store_true")

    cp = sub.add_parser("cloud-probe", help="see what maps/history iRobot's servers hold for your robot")
    cp.add_argument("--email", help="iRobot account e-mail")
    cp.add_argument("--country", default="US", help="account country code")
    cp.add_argument("--config", default=DEFAULT_CONFIG)
    cp.add_argument("--out", default="roomba_cloud", help="folder for the downloaded data")

    pr = sub.add_parser("probe", help="check what a robot supports")
    pr.add_argument("--ip")
    pr.add_argument("--blid")
    pr.add_argument("--password")
    pr.add_argument("--config", default=DEFAULT_CONFIG)
    pr.add_argument("--mqtt-port", type=int, default=8883)
    pr.add_argument("--no-tls", action="store_true")
    pr.add_argument("--listen", type=float, default=60.0, help="seconds to watch for positions")
    pr.add_argument("--dump", default="roomba_probe.jsonl",
                    help="save every message here (network details removed); '' to skip")

    e = sub.add_parser("emulate", help="pretend to be a Wi-Fi Roomba")
    e.add_argument("--host", default="127.0.0.1")
    e.add_argument("--port", type=int, default=8883)
    e.add_argument("--blid", default="EMU0001")
    e.add_argument("--password", default="emulator")
    e.add_argument("--sku", default="Y011040", help="Y011040 = Roomba Combo Essential (RVG-Y1)")
    e.add_argument("--pose-mode", choices=("rrtp", "state", "field", "none"), default="rrtp")
    e.add_argument("--rrtp-topic", default="req", help="topic the emulator answers position requests on")
    e.add_argument("--no-tls", action="store_true")
    e.add_argument("--speed", type=float, default=15.0)
    e.add_argument("--discovery-port", type=int, default=None,
                   help="answer discovery on this UDP port (5678 needs the real port free)")
    return p


def cmd_serve(args):
    from .controller import MapController
    from .server import lan_address, make_server
    from .store import MapStore
    from .wifi import WifiRoomba, profile_for

    cfg = load_config(args.config)
    robot = args.robot or ("wifi" if (cfg or args.ip) else "sim")
    if robot == "wifi":
        ip = args.ip or cfg.get("ip")
        blid = args.blid or cfg.get("blid")
        password = args.password or cfg.get("password")
        if not (ip and blid and password):
            sys.exit("A Wi-Fi robot needs --ip, --blid and --password (or run `get-password` first). "
                     "Use --robot sim to try the app without a robot.")
        sku = cfg.get("sku")
        if not args.model and not sku:
            from .wifi import discover
            found = discover(2.0, ip)
            sku = found[0]["sku"] if found else None
        profile = profile_for(key=args.model) if args.model else profile_for(sku)
        link = WifiRoomba(ip, blid, password, port=args.mqtt_port, tls=not args.no_tls,
                          profile=profile, pose_source=args.pose_source, pose_units=args.pose_units,
                          pose_path=args.pose_path or cfg.get("pose_path"),
                          pose_angle=args.pose_angle,
                          rrtp_topic=args.rrtp_topic or cfg.get("rrtp_topic") or "req",
                          rrtp_con_type=args.rrtp_contype or cfg.get("rrtp_contype") or "local")
    else:
        from .simulator import SimLink, SimRoomba
        link = SimLink(SimRoomba(seed=args.sim_seed), time_scale=args.sim_speed,
                       profile=profile_for(key=args.model or "combo-essential"))

    store = MapStore(args.map)
    controller = MapController(store, link)
    controller.start()
    server = make_server(controller, args.host, args.port, args.pin)
    shown = lan_address() if args.host in ("0.0.0.0", "") else args.host
    print(f"Roomba mapper: {link.name}. Map file: {store.path}")
    print(f"Open on your phone (same Wi-Fi): http://{shown}:{args.port}/")
    if args.pin:
        print("PIN protection is on.")
    print("Press Ctrl+C to stop.")

    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        controller.shutdown()
        print("Map saved. Bye.")


def cmd_discover(args):
    from .wifi import discover
    robots = discover(args.timeout, args.target, args.discovery_port)
    if not robots:
        print("No robots answered. Make sure the robot is on the same network and awake "
              "(press CLEAN or take it off the dock), or try --target <robot-ip>.")
        return 1
    for r in robots:
        prof = r["profile"]
        print(f"{r['ip']:15}  {r['name']!s:20}  blid={r['blid']}  sku={r['sku']}  fw={r['firmware']}")
        print(f"{'':15}  model: {prof['name']}.  {prof['notes']}")
    return 0


def cmd_get_password(args):
    from .wifi import discover, get_credentials_cloud, get_password_local, profile_for
    if args.cloud:
        email = args.email or input("iRobot account e-mail: ")
        pw = getpass.getpass("iRobot account password (sent only to iRobot): ")
        robots = get_credentials_cloud(email, pw, args.country)
        if not robots:
            print("The account has no robots.")
            return 1
        found = {r["blid"]: r for r in discover(3.0)}
        for r in robots:
            ip = (found.get(r["blid"]) or {}).get("ip")
            print(f"{r['name']}: blid={r['blid']} sku={r['sku']} ({r['profile']['name']}) ip={ip or '?'}")
        chosen = robots[0]
        if len(robots) > 1:
            idx = int(input(f"Which robot (1-{len(robots)})? ")) - 1
            chosen = robots[idx]
        cfg = {"ip": (found.get(chosen["blid"]) or {}).get("ip") or args.ip,
               "blid": chosen["blid"], "password": chosen["password"], "sku": chosen["sku"]}
        if not cfg["ip"]:
            cfg["ip"] = input("Robot IP address (see your router or the iRobot app): ").strip()
    else:
        if not args.ip:
            sys.exit("Give the robot's address with --ip (see `discover`), or use --cloud.")
        info = (discover(3.0, args.ip) or [{}])[0]
        print("Put the robot on its dock, then hold the HOME button for about 2 seconds "
              "until it plays a tone or the Wi-Fi light flashes.")
        input("Press Enter when done... ")
        password = get_password_local(args.ip, args.port)
        cfg = {"ip": args.ip, "blid": info.get("blid"), "password": password, "sku": info.get("sku")}
        if not cfg["blid"]:
            cfg["blid"] = input("Robot BLID (discovery did not answer): ").strip()
        print(f"Password received for {profile_for(cfg['sku'])['name']}.")
    if not args.no_save:
        save_config(args.config, cfg)
    else:
        print(json.dumps(cfg, indent=2))
    return 0


def cmd_probe(args):
    from .wifi import probe
    cfg = load_config(args.config)
    ip = args.ip or cfg.get("ip")
    if not ip:
        sys.exit("Give --ip (or run get-password first).")
    probe(ip, args.blid or cfg.get("blid"), args.password or cfg.get("password"),
          port=args.mqtt_port, tls=not args.no_tls, listen=args.listen, dump_path=args.dump or None)
    return 0


def cmd_cloud_probe(args):
    from .cloud import CloudError, cloud_probe
    cfg = load_config(args.config)
    email = args.email or input("iRobot account e-mail: ")
    pw = getpass.getpass("iRobot account password (sent only to iRobot): ")
    try:
        cloud_probe(email, pw, args.country, blid=cfg.get("blid"), out_dir=args.out)
    except CloudError as exc:
        print(f"Stopped: {exc}")
        return 1
    return 0


def cmd_emulate(args):
    from .emulator import RoombaEmulator
    emu = RoombaEmulator(args.host, args.port, args.blid, args.password, sku=args.sku,
                         pose_mode=args.pose_mode, tls=not args.no_tls, time_scale=args.speed,
                         discovery_port=args.discovery_port, rrtp_topic=args.rrtp_topic)
    print(f"Emulated robot listening on {args.host}:{emu.port} (TLS {'off' if args.no_tls else 'on'}), "
          f"blid={args.blid} password={args.password} sku={args.sku} positions={args.pose_mode}")
    print(f"Connect with: python -m roomba_mapper serve --robot wifi --ip {args.host} "
          f"--mqtt-port {emu.port} --blid {args.blid} --password {args.password}"
          + (" --no-tls" if args.no_tls else ""))
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        emu.close()
    return 0


def main(argv=None):
    parser = build_parser()
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0].startswith("-") and argv[0] not in ("-h", "--help"):
        argv = ["serve"] + argv
    args = parser.parse_args(argv)
    handlers = {"serve": cmd_serve, "discover": cmd_discover, "get-password": cmd_get_password,
                "probe": cmd_probe, "cloud-probe": cmd_cloud_probe, "emulate": cmd_emulate}
    return handlers[args.cmd](args) or 0


if __name__ == "__main__":
    sys.exit(main())
