# simple_umbreon_zephyr

Wall-follow racing car firmware on [Zephyr RTOS](https://zephyrproject.org/) for Raspberry Pi Pico 2 (RP2350A, Cortex-M33).

The car reads a ring of 6 VL53L0X ToF distance sensors and uses a PID speed controller to follow the inner wall of a race track. All parameters can be tuned live over a WiFi serial bridge without reflashing.

## Hardware

| Component | Part | Connection |
|-----------|------|------------|
| MCU | Raspberry Pi Pico 2 (RP2350A) | — |
| Distance sensors | 6× ST VL53L0X ToF | I2C1, GP2 SDA / GP3 SCL |
| Steering servo | Standard RC servo | PWM GP10 |
| Drive motor | Brushed ESC (1000–2000 µs) | PWM GP11 |
| Tachometer | Optical encoder | GPIO GP13 (interrupt) |
| WiFi bridge | ESP8266 (UART pass-through) | UART1, GP4 TX / GP5 RX |

Sensors are wired to XSHUT pins GP6–GP9, GP14, GP15 and assigned I2C addresses 0x30–0x35 at boot.

## Prerequisites

- Linux host (Ubuntu 22.04+ recommended)
- USB cable for flashing / serial console

## Setup

```bash
# Full first-time setup: SDK + Zephyr workspace (~10 min)
make setup

# Or if SDK is already installed:
make setup SKIP_SDK=1    # (passes --skip-sdk to setup_zephyr.sh)
```

## Build & Flash

```bash
make build               # compile firmware

# Hold BOOTSEL, connect USB, release BOOTSEL — RP2350 mounts as USB drive
make flash               # copy zephyr.uf2 to the drive

make monitor             # open serial console (picocom, 115200)
```

Run `make` with no arguments to see all targets and current variable values.

## Sensor layout

```
       FRONT
  [4]FL   FR[1]
[5]HL       HR[0]
  [3]L     R[2]
       REAR
```

Indices match the `IDX_*` constants in `src/settings.h`. Distances are in **cm×10** throughout (e.g. 500 = 50 cm, max 2000 = 200 cm).

## WiFi command protocol

Commands are sent as `$CMD\n` lines over UART1 (115200 baud). The car replies on the same UART.

### Configuration

| Command | Description |
|---------|-------------|
| `$GET` | Read all parameters → `$CFG:KEY=VAL,...` |
| `$SET:KEY=VAL,...` | Write one or more parameters to RAM |
| `$SAVE` | Persist current RAM config to flash (NVS) |
| `$LOAD` | Reload config from flash |
| `$RST` | Reset all parameters to firmware defaults |

**Example** — raise top speed and save:
```
$SET:XSP=1620
$SAVE
```

### Control

| Command | Description |
|---------|-------------|
| `$START` | Start wall-follow autonomous loop |
| `$STOP` | Stop and brake |
| `$STATUS` | Query run state → `$STS:RUN` or `$STS:STOP` |
| `$DRVEN` | Enable manual drive mode |
| `$DRVOFF` | Disable manual drive mode |
| `$DRV:steer,speed` | Manual steer (−1000…+1000) and speed (m/s); times out after 500 ms |

### Diagnostics

| Command | Description |
|---------|-------------|
| `$PING` | → `$PONG` |
| `$BAT` | → `$BAT:0.00` (voltage, not wired in this build) |
| `$SRV:angle` | Move servo to raw angle 0–180° |
| `$ESC:us` | Set ESC to raw pulse width 1000–2000 µs |
| `$LOG:ON/OFF` | Toggle `$L:` debug lines |

### Telemetry

While running the car streams one line every ~200 ms:

```
ms,s0,s1,s2,s3,s4,s5,steer,speed,target
```

`s0`–`s5` are sensor distances (cm×10), `speed` is measured m/s, `target` is PID setpoint m/s.

## Tuning

See **[docs/tuning.md](docs/tuning.md)** for a full description of every parameter and a step-by-step tuning procedure.

## License

Apache 2.0 — see [LICENSE](LICENSE).
