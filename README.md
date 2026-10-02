# Roomba Mapper

A persistent cleaning map for a Roomba. The robot remembers, between runs,
which parts of the floor it has cleaned, where the furniture is, and which
areas still need cleaning. You can watch and control it from your phone.

- **Persistent map.** A grid of floor cells is saved to `roomba_map.json`
  every couple of seconds and on shutdown. The save is atomic, so a crash
  never corrupts the map.
- **Automatic mode.** The robot drives to the nearest cell that still needs
  cleaning, again and again, in a back-and-forth pattern. When it bumps into
  something it marks that cell as an obstacle and plans a way around it.
  When nothing reachable is left, it drives back to the dock.
- **Cleaned spots expire.** A spot counts as dirty again after a set number
  of hours (24 by default). The next automatic run then covers only what
  actually needs it.
- **Manual mode from your phone:**
  - drive with the D-pad, or tap "Go here" and then a cell
  - paint cells as cleaned, needs cleaning, obstacle, no-go or erased
  - move the dock, correct where the robot is, start a new cleaning pass,
    or resize the map
- **Phone-friendly GUI.** The page works on phone screens and supports dark
  mode. You can add it to your home screen. A PIN can protect it.

## Quick start (simulator, no hardware)

```bash
python3 -m roomba_mapper --pin 1234
```

The program prints a URL such as `http://192.168.1.20:8080/`. Open it on a
phone connected to the same Wi-Fi, enter the PIN, and tap **Auto clean**.
The simulator robot starts in a room with randomly placed furniture that it
doesn't know about. It finds the furniture by bumping into it.

Python 3.9+ is required. The simulator uses only the standard library.

## Real Roomba

This mode works with any Roomba or Create that has the iRobot Open
Interface: the 500/600/700/800/900 series and the Create 2. Connect it with
a USB-serial cable to a computer, for example a Raspberry Pi riding on the
robot or sitting next to the dock:

```bash
pip install pyserial
python3 -m roomba_mapper --robot oi --serial /dev/ttyUSB0 --pin 1234 \
    --width 25 --height 18 --cell-cm 30
```

- `--width`, `--height` and `--cell-cm` set the size of a **new** map: for
  example, 25 × 18 cells of 30 cm cover 7.5 m × 5.4 m. An existing map file
  keeps its own size; you can resize it in the app.
- The robot tracks its position from its own wheel sensors (dead
  reckoning), so the position drifts over long runs. To correct it, use
  **Robot is here**, or send the robot to the dock, which resets the
  position to the dock cell.
- Put the dock in the right cell with **Set dock** before the first
  automatic run.

## Options

| flag | default | meaning |
| --- | --- | --- |
| `--map` | `roomba_map.json` | map file (created if missing) |
| `--host` / `--port` | `0.0.0.0` / `8080` | where the web app listens |
| `--pin` | none | PIN the phone must enter |
| `--robot` | `sim` | `sim` or `oi` (real robot) |
| `--serial`, `--baud` | `/dev/ttyUSB0`, `115200` | serial settings for `oi` |
| `--tick` | `0.25` | pause between moves (seconds) |
| `--sim-seed` | `7` | furniture layout for the simulator |
| `--auto-start` | off | start cleaning right away |

## How it works

| file | role |
| --- | --- |
| `roomba_mapper/map_store.py` | grid of cells (unknown / dirty / clean / obstacle / no-go) with last-cleaned times; atomic JSON save |
| `roomba_mapper/planner.py` | breadth-first search to the nearest cell that needs cleaning, preferring to keep driving straight |
| `roomba_mapper/controller.py` | background loop for the idle / auto / manual / returning modes; applies bumps and moves to the map |
| `roomba_mapper/robot.py` | `SimulatedRobot` and `RoombaOI` (Open Interface over serial) |
| `roomba_mapper/server.py` | JSON API and static files (standard-library HTTP server) |
| `roomba_mapper/static/` | the phone GUI (HTML, CSS, JS, canvas) |

### API

Every `POST` takes a JSON body and returns the full state. If a PIN is set,
send it in the `X-Pin` header.

```
GET  /api/state
POST /api/mode      {"mode": "auto" | "manual" | "returning" | "idle"}
POST /api/drive     {"direction": "N" | "E" | "S" | "W"}
POST /api/goto      {"x": 4, "y": 7}
POST /api/cells     {"cells": [[x, y], ...], "state": "clean" | "dirty" | "obstacle" | "nogo" | "unknown"}
POST /api/dock      {"x": 0, "y": 0}
POST /api/robot     {"x": 3, "y": 2, "heading": "E"}
POST /api/reset     {"scope": "pass" | "all"}
POST /api/resize    {"width": 25, "height": 18}
POST /api/settings  {"stale_hours": 24, "clean_in_manual": true}
```

## Tests

```bash
python3 -m unittest discover -s tests -v
```
