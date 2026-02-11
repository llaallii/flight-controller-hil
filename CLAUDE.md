# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Summary

Hardware-in-the-Loop (HIL) multirotor simulation with a companion/flight-controller split:
- **PC side:** MuJoCo physics (1 kHz) + GUI viewer (60 Hz) + ArduPilot SITL (navigation/outer loops)
- **FC side:** Rate PID + Quad-X mixer + failsafe (Stage 2: Python dummy, Stage 3: STM32F407)
- **Glue:** MAVLink v2 everywhere; ArduPilot JSON backend for sim state

## Running

All commands run from `hil/` directory.

```bash
# Create local conda env (one-time)
conda env create --prefix ./.conda-env --file environment.yml
conda activate ./.conda-env

# Stage 1 — MuJoCo viewer + built-in hover controller (no ArduPilot, no FC)
python bridge.py --stage 1

# Stage 2 — Full pipeline (4 terminals, start in this order)
#   T1: cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v Copter --model JSON -I0 -l -35.3632621,149.1652374,584,0 --out=127.0.0.1:14550 --out=127.0.0.1:14551
#   T2: python bridge.py --stage 2          (wait for AP=connected)
#   T3: python dummy_fc.py                  (wait for FC=ok in bridge output)
#   T4: python fly_mission.py                (interactive waypoints; or connect GCS to 127.0.0.1:14551)

# Stage 3 — STM32 replaces dummy FC
python bridge.py --stage 3 --serial /dev/ttyUSB0

# Dummy FC at reduced rate (if 1 kHz overruns on your machine)
python dummy_fc.py --rate 500
```

## Architecture — The Three-Process Split

The system runs as up to three cooperating processes connected by UDP:

1. **`bridge.py`** — Main process. Owns the MuJoCo model, steps physics at 1 kHz, syncs the viewer at 60 Hz, generates all sensor data, and routes MAVLink between ArduPilot and the FC. Contains `ArduPilotJSONBackend` (UDP JSON on port 9002), `MAVLinkManager` (pymavlink on ports 14550/14560/14561), and a `SimpleHoverController` for Stage 1 standalone testing.

2. **ArduPilot SITL** — External process launched via `sim_vehicle.py -v Copter --model JSON`. Receives sim state via JSON, runs EKF + position/attitude controllers, outputs `ATTITUDE_TARGET` (body rates + thrust) on its MAVLink port. Its internal motor outputs (JSON servo data) are **intentionally ignored** — the FC closes the rate loop instead.

3. **`dummy_fc.py`** — Flight controller endpoint. Receives `SET_ATTITUDE_TARGET` (msg 82) + `SCALED_IMU` (msg 26), runs rate PID + Quad-X mixer, returns `SERVO_OUTPUT_RAW` (msg 36) motor PWMs. Uses the **exact same MAVLink contract** the future STM32 firmware will use — swapping to Stage 3 changes only the transport (UDP → serial).

## Key Data Flow

```
Bridge → ArduPilot:  JSON state (imu, position, attitude, velocity) on UDP :9002
ArduPilot → Bridge:  ATTITUDE_TARGET (msg 83) on MAVLink UDP :14550
Bridge → FC:         SET_ATTITUDE_TARGET (msg 82) + SCALED_IMU (msg 26) on UDP :14561
FC → Bridge:         SERVO_OUTPUT_RAW (msg 36) on UDP :14560
Bridge → MuJoCo:     data.ctrl[0:4] = motor commands from FC
```

## Critical Conventions

- **Coordinate frames:** MuJoCo is X-fwd, Y-left, Z-up. NED is X-north, Y-east, Z-down. Transform: `v_ned = diag(1, -1, -1) @ v_mujoco`. Body gyro/accel use the same diagonal transform. All frame logic lives in `config.py` (`MJ_TO_NED`, `BODY_GYRO_MJ_TO_NED`) and `sensors.py`.

- **MAVLink IDs:** ArduPilot=sysid 1, Bridge=sysid 2, FC=sysid 3 (all compid 1). These are in `config.py` and must match across bridge, dummy FC, and future STM32 firmware.

- **Motor numbering (Quad-X):** M1=Front-Right(CW), M2=Back-Left(CW), M3=Front-Left(CCW), M4=Back-Right(CCW). The mixer matrix in `config.py` (`MIXER`) maps `[throttle, roll, pitch, yaw]` → `[m1, m2, m3, m4]`. The MJCF actuator `gear` signs must stay consistent with this.

- **Timing model:** The bridge main loop uses `time.perf_counter()` with hybrid sleep+busy-wait for 1 kHz precision. Viewer sync (`viewer.sync()`) is gated to 60 Hz inside the same loop — it only copies data, actual GL rendering is on a background thread from `launch_passive`.

## config.py is the Single Source of Truth

All constants — MAVLink IDs, UDP ports, PID gains, mixer matrix, sensor rates, vehicle parameters, frame transforms — are defined in `config.py`. Both `bridge.py` and `dummy_fc.py` import from it. The future STM32 C firmware must mirror these values exactly.

## Sensor Rate Scheduling

Sensors are scheduled by tick-count modulo, derived from their Hz rate at 1 kHz physics:
- IMU: every tick (1 kHz)
- GPS: via JSON state to ArduPilot (every response to ArduPilot's ~400 Hz poll)
- LiDAR: every 100 ticks (10 Hz), via `DISTANCE_SENSOR` MAVLink to ArduPilot
- Camera: 30 Hz, raw frames via side-channel (not MAVLink), metadata-only via MAVLink

## STM32 Integration (Stage 3)

Documented in `hil/docs/STM32_PLAN.md`. The FC message contract is frozen:
- **Receives:** `SET_ATTITUDE_TARGET` (82), `SCALED_IMU` (26)
- **Sends:** `SERVO_OUTPUT_RAW` (36), `HEARTBEAT` (0)
- Transport changes from UDP to UART serial (921600 baud, DMA circular RX).
- Bridge handles the swap via `--stage 3 --serial /dev/ttyUSB0`.
