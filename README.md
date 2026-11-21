# AgroZumo-TE2004B.601

Monorepo for the TE2004B.601 AgroZumo project. Each folder is a Raspberry Pi Pico SDK project with its own `CMakeLists.txt`.

| Folder | Firmware |
|--------|----------|
| `LocalPositioning/` | Local positioning node (CAN through MCP2515, lwIP) |
| `LocalPositioning_Zumo/` | Zumo robot node (CAN through MCP2515, motors). Drives 50 cm in y. |
| `WaypointLap_Zumo/` | Zumo robot node for the second delivery. Follows the 14 track waypoints for one lap. |
| `AlphaAngle_Zumo/` | Zumo program that measures the alpha angle with the five line sensors. |
| `tools/` | PC tool to record waypoints, log runs and plot the route (`lp_tool.py`). |

The Pico 2 W runs `LocalPositioning/` in both deliveries. The CAN message did not change.

## Connections

The Pico 2 W and the Zumo 2040 each drive one MCP2515 + TJA1050 CAN module over SPI0. The connector names (J1 to J8) are the ones on the PCBTractor board.

### Pico 2 W to its MCP2515 module

| MCP2515 module | Pico 2 W | PCBTractor |
|----------------|----------|------------|
| SCK | GP18 | J2.2 |
| SI (MOSI) | GP19 | J2.3 |
| SO (MISO) | GP16 | J2.4 |
| CS | GP17 | J2.5 |
| INT | GP20 | J2.1 |
| VCC | 3V3 (pin 36) | J2.7 |
| GND | GND | |

### Zumo 2040 to its MCP2515 module

| MCP2515 module | Zumo 2040 |
|----------------|-----------|
| SCK | GP2 |
| SI (MOSI) | GP3 |
| SO (MISO) | GP0 |
| CS | GP1 |
| INT | GP28 |
| VCC | Pico 2 W 3V3, through J4.7 |
| GND | GND |

Do not use GP18 to GP22 on the Zumo. PCBTractor routes the Zumo SPI to those pins on J5, but the Zumo front sensor array also uses them. With the array attached, the MCP2515 does not answer and `CANSTAT` reads `0x00`. Connect jumper wires to GP0 to GP3 and GP28 instead.

GP0 to GP3 are also the Zumo OLED pins, and GP3 is the RGB LED data line. The OLED can show noise while the program runs. The CAN driver holds the RGB LED clock (GP6) low, so the LEDs ignore the SPI traffic.

The Zumo motors use GP10, GP11, GP14 and GP15 on the Zumo board. They need no wiring.

### CAN bus

- Connect CAN_H to CAN_H and CAN_L to CAN_L between the two modules (J7 on the Pico side, J8 on the Zumo side).
- Put the J1 termination jumper (120 Ω) on both modules. With the power off, the resistance between CAN_H and CAN_L must be about 60 Ω. Without termination, the Pico reports `NO ACK` and a high receive error count (REC).
- Both modules must have an 8 MHz crystal. The firmware sets 500 kbps for that crystal.

### Power

| Situation | J5.8 jumper | Zumo power switch |
|-----------|-------------|-------------------|
| Pico 2 W USB cable connected (flashing, serial monitor) | Removed | ON for any CAN test |
| Demo on the track, no Pico USB cable | Installed | ON |

- J5.8 connects the Zumo 3.3 V rail to the Pico VSYS pin, so the Zumo powers the Pico. If you connect the Pico USB cable with J5.8 installed, the USB 5 V reaches the Zumo 3.3 V rail.
- The 5 V supply of both TJA1050 transceivers (J3.1 and J6.1) comes from the Zumo batteries through the power switch. If the switch is OFF, the CAN bus does not work. USB power does not change this.
- The Zumo MCP2515 module takes its 3.3 V from the Pico. If the Pico has no power, the Zumo prints `MCP2515 not responding` once a second until the Pico turns on.

### CAN message

The Pico 2 W sends the position of ArUco marker 3 to the Zumo in one frame.

| Field | Value |
|-------|-------|
| ID | `0x100` (standard 11-bit) |
| DLC | 6 |
| D0-D1 | x in mm, int16, big-endian |
| D2-D3 | y in mm, int16, big-endian |
| D4-D5 | Angle in degrees, int16, big-endian |
| Period | 100 ms. If no new camera position arrived, the Pico skips the frame. |

## Build

You need the Pico SDK. Run these commands inside one project folder:

```bash
cmake -B build -G Ninja
cmake --build build
```

The build writes the `.uf2` file to `build/`.

## Second delivery: waypoint lap

You need a laptop on the `embedded` Wi-Fi network. The laptop receives the same camera broadcast as the Pico 2 W.

### 1. Record the waypoints

The waypoints are the entrance and the exit of each turn, P1 to P14, as in the challenge drawing. The lap starts and stops at P14. Odd waypoints (P1, P3, ..., P13) are turn entrances.

```bash
python3 tools/lp_tool.py record
```

1. Put the Zumo on the waypoint, centered on the black line.
2. Turn the Zumo to face the travel direction (the green arrows in the drawing).
3. Press Enter. The tool averages the last 10 camera frames.
4. Do steps 1 to 3 for P1 to P14.

The tool writes `WaypointLap_Zumo/waypoints.h` and `tools/waypoints.csv`. It also finds how the camera angle maps to the direction of travel (`ANGLE_SIGN`, `ANGLE_OFFSET_DEG`). If it prints a warning about the angle fit, record the waypoints again.

You can also record from a log. Run `python3 tools/lp_tool.py log walk.csv`, put the Zumo on P1 to P14 in order, and stop for 2 s on each one. Then run `python3 tools/lp_tool.py extract walk.csv`. The tool finds the stops and uses the last 14.

The marker is not above the turning center of the Zumo. Both commands measure this distance from the 180-degree turns and write it as `MARKER_OFFSET_M`. The table holds turning-center positions, and the Zumo moves each camera pose to its turning center before it uses it. If you move the marker, record the waypoints again. To give a measured value instead, add `--marker-offset 0.06` before the command name.

### 2. Build and flash

Build `WaypointLap_Zumo/` (see Build) and copy `build/WaypointLapZumo.uf2` to the Zumo.

### 3. Run a lap

1. Put the Zumo on P14, facing up the track.
2. Turn on the Zumo power switch. The Pico 2 W must send LPdata.
3. Start the log on the laptop: `python3 tools/lp_tool.py log run1.csv`
4. Press button A on the Zumo. The Zumo starts 0.3 s later.
5. When the Zumo stops on P14, press Ctrl+C on the laptop.

If no LPdata arrives for 500 ms, the Zumo stops and waits. While it waits for button A, it prints the pose, the distance to P14 and the last lap time on USB serial.

### 4. Plot and lap time

```bash
uv run --with matplotlib tools/lp_tool.py plot run1.csv run2.csv run3.csv --out route.png
```

The plot shows the numbered waypoints, the desired route and each run. The tool prints the lap time of each run.

### How the Zumo follows the route

At each camera frame (about 5 per second) the Zumo:

1. Moves the pose forward by `LATENCY_S`, because the pose is about 150 ms old.
2. Finds the closest path point in the next 25 cm of the path. The progress never goes back, so the Zumo cannot skip a waypoint or jump to the next lane.
3. Takes the target point 9 cm ahead on the path.
4. Computes the pure pursuit curvature `k = 2 sin(alpha) / distance`, with alpha the angle from the heading to the target.
5. Takes the speed from the speed profile: fast on straights, slow on turns, with braking before each turn.
6. Sets left = v (1 - k b), right = v (1 + k b), with b half the track width.
7. At the final waypoint, it stops.

The path is built once from the waypoint table. A straight section is a line. A turn section is the circle arc that starts tangent to the previous straight and ends on the exit waypoint. This keeps the Zumo over the track in the 90-degree and 180-degree turns.

### Tuning for a faster lap

The values are in `WaypointLap_Zumo/follow.h`.

- `V_STRAIGHT` and `V_TURN` set the speeds. Raise them in small steps.
- Measure `M_PER_UNIT`. Run the Zumo straight at a known command, take the speed from the log (m/s), and divide by the command.
- Set `LATENCY_S` to the `frame_age_ms` of the camera JSON plus about 50 ms.
- If the Zumo cuts the turns, lower `LOOKAHEAD_PTS` or raise `K_TURN`. If it weaves on the straights, raise `LOOKAHEAD_PTS`.

### Host tests

Run these on the PC, without the Zumo:

```bash
cd WaypointLap_Zumo && gcc -O2 -I. test/test_follow.c follow.c -lm -o /tmp/test_follow && /tmp/test_follow
```

```bash
cd AlphaAngle_Zumo && gcc -I. test/test_alpha.c alpha.c -lm -o /tmp/test_alpha && /tmp/test_alpha
```

`test_follow` simulates one lap. The camera sends 5 frames per second with a 150 ms delay and 3 mm and 2 degree noise. One track slips 5 %. If the Zumo skips a waypoint or goes more than 35 mm off the path, the test fails. `test_alpha` checks the handout example (alpha = 30 degrees).

## Alpha angle (gray-level terrain)

`AlphaAngle_Zumo/` measures the angle alpha between the line of the five line sensors and the tangent to the terrain arc at the center sensor S3. The gray level of the terrain changes with the distance from the white corner.

1. Read the five sensors (RC method, `line_sensors.c`). DN1 to DN5 are GP22 to GP18. The emitters are on GP26.
2. Convert each reading to the distance r_i from the white corner with the calibration table `GRAY_CAL` in `main.c`.
3. Sensor i is at s_i = (i - 3) d on the sensor line, with d the sensor spacing (`SENSOR_SPACING_M`).
4. The law of cosines gives r_i^2 = r_3^2 + 2 s_i r_3 sin(alpha) + s_i^2.
5. Least squares over the five sensors: sin(alpha) = sum s_i (r_i^2 - r_3^2 - s_i^2) / (2 r_3 sum s_i^2).

To calibrate, put S3 on the terrain at known distances from the white corner and write the raw S3 value in `GRAY_CAL`. The table in the repo holds placeholder values. Measure `SENSOR_SPACING_M` on your Zumo. The handout example uses 15 mm.

This program does not use CAN. GP18 to GP22 are free because the MCP2515 uses GP0 to GP3.
