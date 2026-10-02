# Roomba Mapper

Roomba Mapper draws a geometric map of your home from where your Roomba
drives. It saves that map on your own computer, and the map then becomes the
robot's reference: later runs are lined up with it and measured against it.
You can draw or edit the map by hand from your phone.

It talks to the robot over the robot's **built-in Wi-Fi radio**, so you
don't need any add-on hardware or serial cable. It supports the
**Roomba Combo Essential (model RVG-Y1)** and other Wi-Fi Roombas. It also
has a built-in simulator for trying everything without a robot.

## How the map works

1. **It learns automatically.** While a run is in progress, the mapper
   reads the robot's position over Wi-Fi and records every spot the robot
   passes over. When the run ends, it turns that area into a floor plan made
   of polygons:
   - room outlines, with walls snapped square to the main direction of the
     home
   - obstacles: free-standing furniture shows up as holes in the floor
2. **It becomes the reference.** After each learning run, the mapper checks
   how much new floor the run added. Once a run adds less than 5%, it locks
   the map. You can also lock it yourself at any time. After that:
   - each run's position track is aligned to the reference map, correcting
     the slow drift of the robot's own position estimate
   - cleaning progress is measured against the reference map
   - new runs no longer reshape the map
3. **You can define it by hand.** Under **✎ Edit map** you can:
   - draw rooms, obstacles and no-go zones as rectangles or polygons
   - drag corners, and drag the dot in the middle of an edge to add a
     corner
   - change a shape's type, or delete it

   Shapes you draw or edit are never overwritten when the map is
   regenerated. You can skip learning completely: draw the rooms and lock
   the map.
4. **It tracks what is clean.** The mapper records when each 10 cm cell of
   floor was last cleaned. A cell needs cleaning again after a set time
   (24 hours by default). The app shows how much is clean, how much is
   left, and numbered spots that were missed. It can also start a clean
   automatically when enough of the floor needs one.

Everything is saved in `roomba_map.json` next to where you run the
program. **⬇ Export map** downloads the floor plan as GeoJSON, in metres
with the dock at the origin.

## Mapping your home by walking it

If your robot can't report its position (as with the Combo Essential), you
can make the floor plan yourself by walking around with your phone:

```bash
python3 -m roomba_mapper serve --https --pin 1234
```

1. Open the printed `https://` address on your phone. The first time, the
   phone warns that the connection isn't private, because the certificate
   is made by this program, not a public authority. On an iPhone, tap
   **Show Details → visit this website**.
2. Tap **🚶 Walk**, stand right in front of the dock with your back to its
   wall, and tap **Start at the dock**. Allow motion access if asked. The
   dock is the map's origin, the same reference point the robot uses.
3. Turn left or right and walk along the walls in straight lines, about
   30 cm from them. At every corner tap **↰ Turned left** or
   **↱ Turned right**. Each wall's length comes from your steps; type the
   real length into the box to correct it. The app then learns your step
   length.
4. Back at the dock, tap **✓ Finish room**. The app closes the outline,
   shares out any counting error so the walls stay square, and moves the
   walls out by the distance you kept from them.
5. To add more, walk to the next room (straight lines, tapping turns) and
   tap **Trace a room from here**. For furniture, walk around it and tap
   **Trace furniture from here**.
6. Fix anything in **✎ Edit map**.

How the measurements work:

- Distances come from counting steps, so expect roughly 5–10% error unless
  you type measured lengths.
- Turns are assumed to be right angles. Draw angled walls in the editor
  afterwards.
- Motion sensors only work over `https`. If your phone won't allow them,
  type each wall's length before tapping the turn; everything else works
  the same.

## Quick start with the simulator (no robot needed)

```bash
python3 -m roomba_mapper serve --robot sim --pin 1234
```

Open the URL it prints on your phone (same Wi-Fi) and tap **▶ Clean**. A
simulated Combo Essential cleans a two-room flat at 15× speed. Two or three
runs are enough for the map to complete and lock itself.

Requires Python 3.9 or newer. It uses no third-party packages.

## Using your Roomba Combo Essential (RVG-Y1)

1. **Find the robot:**

   ```bash
   python3 -m roomba_mapper discover
   ```

2. **Get its local password** (needed once). The password is saved in
   `roomba_config.json`, readable only by you.
   - **Newer models, including the RVG-Y1:** read it from your iRobot
     account. Your account password goes only to iRobot's login servers
     and is not stored.

     ```bash
     python3 -m roomba_mapper get-password --cloud
     ```

   - **Older models:** read it straight from the robot. Put it on the
     dock, hold HOME for about 2 seconds until it beeps, then run:

     ```bash
     python3 -m roomba_mapper get-password --ip 192.168.1.50
     ```

3. **Check what your robot supports.** Start a clean in the iRobot app,
   close the app, then run:

   ```bash
   python3 -m roomba_mapper probe
   ```

   The probe tells you whether:
   - the robot accepts a local connection
   - it reports its position, either in its state reports or through the
     newer "RRTP" position request
   - automatic mapping will therefore work

4. **Run the mapper:**

   ```bash
   python3 -m roomba_mapper serve --pin 1234
   ```

   Then open the printed address on your phone. Start cleans from the app
   or from the iRobot app; either way the mapper follows along.

### If `probe` finds no position

Newer firmware does not always use the field names or request format the
older models used. `probe` therefore listens for 60 seconds and reports:

- every topic the robot sent on;
- which variant of the position request (if any) it answered;
- the fields that kept changing while the robot drove, ranked by how
  position-like they look.

It also saves every message to `roomba_probe.jsonl`, with Wi-Fi names and
addresses removed. For a useful result, start a clean and wait until the
robot has left the dock before running `probe`.

What to do with the result:

- **A position field is found.** `probe` prints a command such as
  `serve --pose-path cleanMissionStatus.pos`. If the map comes out 10×
  too small or large, add `--pose-units cm` or `--pose-units m`. If turns
  look wrong, add `--pose-angle rad`.
- **A different position request works.** `probe` prints the flags to
  use, for example `--rrtp-topic ...` or `--rrtp-contype remote`.
- **Nothing is found.** Look through `roomba_probe.jsonl` and share it so
  the format can be worked out. Meanwhile you can draw the map by hand.

### Reading maps from your iRobot account (`cloud-probe`)

Newer robots such as the Combo Essential upload each clean's map to
iRobot. `cloud-probe` logs in to your iRobot account the same way the
iRobot app does, then asks for:

- the robot's maps (rooms, borders, dock, and on some models the
  robot's paths and coverage);
- its cleaning history.

```bash
python3 -m roomba_mapper cloud-probe
```

Your password goes only to iRobot's login service. No tokens are saved.
Everything received is written to the `roomba_cloud/` folder, with
passwords, tokens and download links removed.

This API is private and undocumented: the requests follow the community
project roombapy-prime, and iRobot may change or block them at any time.

### Things to know about Wi-Fi Roombas

- **The robot steers itself.** No Wi-Fi command drives a Roomba around, so
  the mapper watches where it goes and maps that.
- **The app sends only mission commands:** Clean (with *Vacuum + mop* or
  *Vacuum only* on Combo models), Pause/Resume and Dock.
- **One local connection at a time.** Roombas accept a single local
  connection. Close the iRobot app, and any Home Assistant integration,
  while the mapper is running.
- **What a real Combo Essential does.** One RVG-Y1 (SKU Y014020, firmware
  `congo+1.1.22`) was tested with `probe` while cleaning:
  - It accepts the local connection and reports its state: mission
    phase, battery, bin and settings. So status, Start, Pause and Dock
    work.
  - It sends **no position**: no `pose` field, nothing that changes as it
    drives, and no answer to any position-request variant.
  - It echoes every message sent to it back to the sender. The mapper now
    ignores those echoes.

  With that firmware, automatic mapping isn't possible over the local
  connection. Draw the map by hand. Newer firmware may differ, so run
  `probe` again after updates.
- **Expect some position drift.** The Combo Essential navigates by
  gyroscope and wheel odometry, so its position estimate drifts more than
  camera or LiDAR models. Aligning each run to the reference map corrects
  most of this. In tests, a drift of 25 cm and 6° was reduced to 3–16 cm.
- **If you move the dock**, unlock the map and let it relearn, or move the
  shapes to match. The map's origin is the dock.

## Trying the full Wi-Fi path without a robot

The emulator speaks the same protocol as a real robot:
- MQTT over TLS with BLID/password login
- state reports
- RRTP position replies
- commands
- UDP discovery

```bash
python3 -m roomba_mapper emulate --port 18883 --speed 3            # terminal 1
python3 -m roomba_mapper serve --robot wifi --ip 127.0.0.1 --mqtt-port 18883 \
    --blid EMU0001 --password emulator --model rvg-y1              # terminal 2
```

The emulator needs the `openssl` command to make its certificate.
`--pose-mode state` makes it report positions in its state reports, like a
Roomba 980. `--pose-mode none` makes it report no position at all.

## Commands

| command | purpose |
| --- | --- |
| `serve` | run the mapper and phone app (`--robot sim/wifi`, `--https`, `--pin`, `--map`, `--port`, `--model rvg-y1`, `--pose-source auto/state/rrtp`, `--pose-path`, `--pose-units mm/cm/m`, `--pose-angle deg/rad`, `--rrtp-topic`, `--rrtp-contype`) |
| `discover` | list Wi-Fi Roombas on the network |
| `get-password` | fetch and save the robot's local password (`--cloud` or `--ip`) |
| `probe` | report what a robot supports, find unknown position fields (`--listen`, `--dump`) |
| `cloud-probe` | download the maps and cleaning history iRobot holds for your robot (`--email`, `--country`, `--out`) |
| `emulate` | pretend to be a Wi-Fi Roomba |

## Code layout

| file | role |
| --- | --- |
| `roomba_mapper/wifi.py` | discovery, password retrieval, live robot link (state, positions via state pose or RRTP, commands), model profiles including the RVG-Y1 |
| `roomba_mapper/mqtt_lite.py` | small MQTT 3.1.1 client and test broker (standard library only) |
| `roomba_mapper/geometry.py` | contour tracing, simplification, squaring of walls, run alignment |
| `roomba_mapper/geomap.py` | the map: shapes, explored area, coverage, generation, persistence |
| `roomba_mapper/controller.py` | learn → lock lifecycle, per-run alignment, auto-start |
| `roomba_mapper/server.py` | JSON API, GeoJSON export, static files |
| `roomba_mapper/static/` | phone app (canvas map with pan/zoom and shape editor) |
| `roomba_mapper/simulator.py`, `emulator.py` | virtual robot and Wi-Fi robot emulator |

### API

Every `POST` takes a JSON body and returns the current state. If a PIN is
set, send it in the `X-Pin` header.

```
GET  /api/state[?map=<map_version>&cov=<cov_version>]   (omits unchanged map/coverage)
GET  /api/export                                         GeoJSON floor plan
POST /api/command        {"command": "clean"|"pause"|"resume"|"dock"|"stop", "mode": "mop"|"vacuum"}
POST /api/shapes         {"kind": "floor"|"obstacle"|"nogo", "points": [[x, y], ...], "name": "..."}
POST /api/shapes/update  {"id": 3, "points": [...], "kind": "...", "name": "..."}
POST /api/shapes/delete  {"id": 3}
POST /api/map            {"action": "lock"|"unlock"|"rebuild"|"reset_coverage"|"erase"}
POST /api/settings       {"stale_hours": 24, "auto_start_percent": 40, ...}
```

## Tests

```bash
python3 -m unittest discover -s tests -v
```

The protocol details come from community projects (dorita980, roombapy and
roomba-v4), not from iRobot, and firmware updates may change them.
