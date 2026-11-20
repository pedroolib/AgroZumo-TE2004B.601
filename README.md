# AgroZumo-TE2004B.601

Monorepo for the TE2004B.601 AgroZumo project. Each folder is a Raspberry Pi Pico SDK project with its own `CMakeLists.txt`.

| Folder | Firmware |
|--------|----------|
| `LocalPositioning/` | Local positioning node (CAN through MCP2515, lwIP) |
| `LocalPositioning_Zumo/` | Zumo robot node (CAN through MCP2515, motors) |

## Build

You need the Pico SDK. Run these commands inside one project folder:

```bash
cmake -B build -G Ninja
cmake --build build
```

The build writes the `.uf2` file to `build/`.
