# Tuning Guide

Parameters are stored in flash and edited live via WiFi without reflashing. The workflow is:

```
$GET              # read current values
$SET:KEY=VAL,...  # adjust one or more values
$SAVE             # persist to flash
$RST              # reset everything to firmware defaults
```

All distance values are in **cm×10** (500 = 50 cm). Speed values are in **m/s**. ESC values are in **µs** (neutral = 1500, range 1000–2000).

---

## 1. Sensor thresholds

These four distances control the car's situational awareness. Tune them after verifying that all six sensors read reasonable values at rest.

| Key | Default | Description |
|-----|---------|-------------|
| `FOD` | 800 | **Front obstacle distance** (80 cm). A front sensor reading below this flags that lane as blocked. Raise to react earlier, lower if the car brakes too soon. |
| `SOD` | 600 | **Side open distance** (60 cm). If both side sensors exceed this, the car treats itself as running in open space and biases hard right to find a wall. |
| `ACD` | 400 | **All close distance** (40 cm). If all four main sensors are below this simultaneously, the car assumes it is boxed in and forces maximum right steering. |
| `CFD` | 100 | **Close front distance** (10 cm). Reserved for emergency braking logic; currently unused in the simple build. |

**Starting point:** set `FOD` to roughly half the track width, `SOD` to the full track width. Narrow tracks → lower values.

---

## 2. Speed settings

| Key | Default | Description |
|-----|---------|-------------|
| `SPD1` | 0.48 | **Clear speed** (m/s). Target speed when neither front sensor is blocked. |
| `SPD2` | 0.32 | **Blocked speed** (m/s). Target speed when one or both front sensors are blocked (cornering). |
| `SLW` | 0.85 | **Speed slew rate** (m/s²). Maximum rate of change of the PID setpoint. Prevents wheel spin on sudden speed jumps. Set to 0 for instant step. |
| `KOP` | 18.0 | **Kick percent** (%). On a forward start from rest, adds `KOP%` of the ESC span as an extra pulse for `KOM` ms to overcome motor stiction. |
| `KOM` | 300 | **Kick duration** (ms). See above. Set to 0 to disable kick. |

**Tuning speed:**
1. Start with `SPD1=0.3`, `SPD2=0.2` and confirm the car completes a lap.
2. Raise `SPD1` in 0.05 m/s steps until the car slides on corners, then back off one step.
3. Lower `SPD2` relative to `SPD1` to slow down in corners.
4. If the car stalls from rest, increase `KOP` or `KOM`. If it lurches, reduce `KOP`.

---

## 3. Steering coefficients

| Key | Default | Description |
|-----|---------|-------------|
| `COE1` | 0.28 | **Clear coefficient**. Multiplies the side-distance error (`R − L`) to produce a steering command when both front sensors are clear. Lower = gentler wall tracking. |
| `COE2` | 0.65 | **Blocked coefficient**. Same multiplier but used when a front sensor is blocked (cornering). Higher = sharper turn-in. |

The steering command is `steer = (R − L) × coef`, clamped to the servo range. With the default sensor layout the car hugs the right wall.

**Tuning steering:**
- If the car oscillates while straight, lower `COE1`.
- If corners are cut or the car hits the inner wall, raise `COE2`.
- If the car understeers on corners, raise `COE2` or lower `FOD` so it commits to the turn earlier.

---

## 4. PID speed controller

The PID runs inside `car_pid_control()` at `LMS` ms intervals. It measures speed from the optical tachometer and drives the ESC to reach the target set by the control loop.

| Key | Default | Description |
|-----|---------|-------------|
| `KP` | 66.4 | Proportional gain. Main response. |
| `KI` | 243.5 | Integral gain. Eliminates steady-state error. |
| `KD` | 4.16 | Derivative gain. Damps oscillation (derivative-on-measurement). |

The output is: `ESC = min_speed + ff + KP·e + KI·∫e·dt + KD·(-dv/dt)`

where `ff` is a feedforward term equal to `(min_speed − 1500)` that pre-compensates for the ESC dead band.

**Tuning procedure:**
1. Set `KI=0`, `KD=0`. Raise `KP` until speed is roughly tracking, then back off 20%.
2. Add `KI` to eliminate the steady-state offset. Integral windup is clamped at ±50.
3. Add a small `KD` if there is visible speed oscillation on straights.

---

## 5. ESC and servo limits

| Key | Default | Description |
|-----|---------|-------------|
| `MSP` | 1540 | **Min forward speed** (µs). ESC pulse at minimum forward throttle. Must be above the ESC's dead band. |
| `XSP` | 1600 | **Max forward speed** (µs). ESC pulse at full forward throttle. |
| `BSP` | 1460 | **Min brake speed** (µs). ESC pulse at minimum reverse/brake. |
| `MNP` | 60 | **Min servo angle** (°). Full-left steering angle. |
| `XNP` | 120 | **Max servo angle** (°). Full-right steering angle. |
| `NTP` | 90 | **Neutral servo angle** (°). Straight-ahead position. |
| `SVR` | 0 | **Servo reverse** (0/1). Flip steering direction. |
| `CAL` | 0 | **Calibrated flag** (0/1). Set to 1 after ESC calibration to skip it on boot. |

**ESC calibration:** power on with `CAL=0` (or `$SET:CAL=0` then reboot). The firmware will cycle max→min→neutral automatically and set `CAL=1`.

**Servo neutral:** use `$SRV:90` to command the servo directly while the car is stationary, then adjust `NTP` until the wheels are straight.

---

## 6. Tachometer

| Key | Default | Description |
|-----|---------|-------------|
| `ENH` | 68 | **Encoder holes**. Number of holes (or slots) per revolution on the encoder disk. |
| `WDM` | 0.060 | **Wheel diameter** (m). Used to convert rev/s to m/s. Measure the driven wheel. |
| `TGF` | 35 | **Tachometer glitch filter** (µs). Pulses shorter than this are rejected as electrical noise. Raise if speed reads erratically at low speed. |
| `LMS` | 40 | **Control loop period** (ms). Lower = faster response but more CPU. Minimum 10. |

Speed is computed from the interval between the last two encoder pulses, not a pulse-count average. If no pulse arrives for 500 ms the speed is forced to zero.

---

## 7. Full parameter reference

| Key | cfg field | Units | Range |
|-----|-----------|-------|-------|
| `FOD` | `front_obstacle_dist` | cm×10 | 10–2000 |
| `SOD` | `side_open_dist` | cm×10 | 10–2000 |
| `ACD` | `all_close_dist` | cm×10 | 10–2000 |
| `CFD` | `close_front_dist` | cm×10 | 10–2000 |
| `KP` | `pid_kp` | — | 0–5000 |
| `KI` | `pid_ki` | — | 0–10000 |
| `KD` | `pid_kd` | — | 0–2000 |
| `MSP` | `min_speed` | µs | 1000–2000 |
| `XSP` | `max_speed` | µs | 1000–2000 |
| `BSP` | `min_bspeed` | µs | 1000–2000 |
| `MNP` | `min_point` | ° | 0–180 |
| `XNP` | `max_point` | ° | 0–180 |
| `NTP` | `neutral_point` | ° | 0–180 |
| `ENH` | `encoder_holes` | count | 1–2000 |
| `WDM` | `wheel_diam_m` | m | 0.01–1.0 |
| `LMS` | `loop_ms` | ms | 10–1000 |
| `SPD1` | `spd_clear` | m/s | 0–5 |
| `SPD2` | `spd_blocked` | m/s | 0–5 |
| `SLW` | `spd_slew` | m/s² | 0–20 |
| `KOP` | `kick_pct` | % | 0–80 |
| `KOM` | `kick_ms` | ms | 0–5000 |
| `COE1` | `coe_clear` | — | 0–5 |
| `COE2` | `coe_blocked` | — | 0–5 |
| `SVR` | `servo_reverse` | 0/1 | — |
| `CAL` | `calibrated` | 0/1 | — |
| `TGF` | `tach_glitch_filter_us` | µs | 1–500 |
