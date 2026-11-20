# AgroZumo-TE2004B.601

Monorepo for the TE2004B.601 AgroZumo project. Each folder is a Raspberry Pi Pico SDK project with its own `CMakeLists.txt`.

| Folder | Firmware |
|--------|----------|
| `LocalPositioning/` | Local positioning node (CAN through MCP2515, lwIP) |
| `LocalPositioning_Zumo/` | Zumo robot node (CAN through MCP2515, motors) |

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
