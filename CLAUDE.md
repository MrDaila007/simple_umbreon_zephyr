# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Zephyr RTOS firmware for a racing car (Raspberry Pi Pico 2 / RP2350A). Implements a wall-follow algorithm using 6 VL53L0X ToF sensors. This is a stripped-down port of `umbreon_zephyr` — no IMU, battery monitor, display, or track learning.

**Target board:** `rpi_pico2/rp2350a/m33`  
**Zephyr version:** v4.4.0 (workspace at `~/zephyrproject-v4.4`)

## Commands

### First-time setup
```bash
./setup_zephyr.sh                    # full setup: SDK + workspace + venv
./setup_zephyr.sh --skip-sdk         # skip SDK if already installed
./setup_zephyr.sh --sdk-only         # install SDK only
```

### Build, flash, monitor
```bash
make build                           # build firmware (always pristine)
make flash                           # copy .uf2 to mounted RP2350 drive
make monitor                         # open serial console (picocom, 115200)
make clean                           # remove build directory
```

Override defaults: `make build ZEPHYR_DIR=~/my-zephyr BUILD_DIR=~/my-build`

Flash path: `~/zephyrproject-v4.4/build/zephyr/zephyr.uf2`  
Mount point: `/media/$USER/RP2350` (hold BOOTSEL while connecting USB)

## Architecture

### Threads (Zephyr)

| Thread | Priority | Stack | Role |
|--------|----------|-------|------|
| `main` | — | 4096 | Init sequence only; sleeps forever after `control_init()` |
| `control` | 2 | 4096 | Wall-follow loop at `cfg.loop_ms` (default 40 ms) |
| `wifi_cmd` | 5 | 2048 | UART1 command parser; ISR-driven RX ring buffer |

### Init sequence (`main.c`)
`settings_init/load` → `wdt_init` → `car_init` → `taho_init` → `sensors_init` (~500 ms I2C) → `wifi_cmd_init` → ESC calibration (if not calibrated) → `control_init`

### Module responsibilities

- **`settings.c`** — 34 runtime parameters in a global `cfg` struct. Persisted to flash via NVS (key 1, magic `"UMBR"`, version 10, with checksum). Protected by `cfg_mutex`. Always use `settings_get_copy()` for snapshots in hot paths; use `settings_lock/unlock()` only when mutating `cfg` directly.

- **`car.c`** — PWM servo (GP10) and ESC (GP11) control. `car_write_steer(s)` maps `[-1000, +1000]` to servo angle using `cfg.min/neutral/max_point`. `car_write_speed_ms(float)` sets PID target; `car_pid_control()` must be called each loop tick. ESC pulses: neutral=1500 µs, range 1000–2000 µs. `car_write_speed(int)` bypasses PID (resets PID state).

- **`sensors.c`** — 6× VL53L0X ToF sensors on I2C1 (GP2/GP3). Addresses `0x30–0x35`, assigned by toggling individual XSHUT pins (GP6–GP9, GP14–GP15) at init. Returns distances as `int[6]` in **cm×10** (e.g., 500 = 50 cm). Max range: 2000 (200 cm).

- **`control.c`** — Wall-follow algorithm. Uses 4 sensors: `IDX_LEFT(3)`, `IDX_FRONT_LEFT(4)`, `IDX_FRONT_RIGHT(1)`, `IDX_RIGHT(2)`. Computes `diff = R - L`; steering = `diff * coef`. Front-blocked state (`FL < FOD` or `FR < FOD`) switches to `spd_blocked`/`coe_blocked`. Telemetry sent every 5 ticks (~200 ms).

- **`tachometer.c`** — Optical encoder on GP13, RISING-edge interrupt. Speed computed from pulse interval. Glitch filter at `cfg.tach_glitch_filter_us` (default 35 µs).

- **`wifi_cmd.c`** — UART1 (GP4 TX, GP5 RX, 115200 baud) command protocol. ISR writes to RX ring buffer; `wifi_cmd` thread dispatches `$`-prefixed commands. TX uses a 1024-byte primary ring buffer with a 512-byte overflow buffer.

### VL53L0X enhanced driver (`modules/vl53l0x_enhanced/`)

Out-of-tree Zephyr module replacing the stock `CONFIG_VL53L0X` driver. Set `CONFIG_VL53L0X=n` and `CONFIG_VL53L0X_ENHANCED=y`. Key feature: `CONFIG_VL53L0X_ENHANCED_RECONFIGURE_ADDRESS=y` enables runtime I2C address reassignment via XSHUT GPIO, required for the 6-sensor daisy-chain init.

### WiFi command protocol

Commands arrive on UART1 as `$CMD\n` lines. Supported subset:

| Command | Effect |
|---------|--------|
| `$PING` | → `$PONG` |
| `$GET` | → `$CFG:...` (all 34 params) |
| `$SET:KEY=VAL,...` | Update cfg in RAM |
| `$SAVE` / `$LOAD` / `$RST` | NVS persistence |
| `$START` / `$STOP` | Start/stop wall-follow |
| `$DRV:steer,speed` | Manual override (500 ms timeout) |
| `$DRVEN` / `$DRVOFF` | Enable/disable manual drive |
| `$SRV:angle` | Direct servo angle (0–180°) |
| `$ESC:us` | Direct ESC pulse (1000–2000 µs) |
| `$LOG:ON` / `$LOG:OFF` | Toggle `$L:` debug output |

Telemetry format (every ~200 ms): `ms,s0,s1,s2,s3,s4,s5,steer,speed,target\n`

### Key configuration constants

- Distances: cm×10 units throughout. `DEFAULT_FOD=800` (80 cm), `DEFAULT_SOD=600`, `DEFAULT_ACD=400`, `DEFAULT_CFD=100` (10 cm).
- ESC: neutral=1500 µs (never changes), range 1000–2000 µs.
- `SETTINGS_VERSION=10` — bump this when adding/removing NVS fields (triggers reset to defaults on mismatch).

### Board overlay

`boards/rpi_pico2_rp2350a_m33.overlay` defines all hardware bindings. USB CDC console is always on (via `usb_console.conf` + `boards/usb_console.overlay`). NVS storage partition is at flash offset `0x3F0000` (64 KB).
