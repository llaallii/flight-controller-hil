# HIL Multirotor Simulation

A Hardware-in-the-Loop quadcopter simulation that pairs **MuJoCo** physics
with an **ArduPilot SITL** autopilot and a split **companion / flight-controller**
architecture — designed so the flight controller can graduate from a Python
prototype to real STM32F407 firmware with only a transport swap.

```
┌──────────────────────── PC (Windows) ────────────────────────────┐
│                                                                   │
│   ArduPilot SITL          MuJoCo Bridge          Flight Controller│
│   ──────────────          ────────────           ────────────────│
│   Navigation              Physics @ 1 kHz        Rate PID @ 1 kHz│
│   State estimation        GUI viewer @ 60 Hz     Quad-X mixer    │
│   Outer-loop control      Sensor generation      Failsafe        │
│                           MAVLink routing                         │
│                                                                   │
│   JSON UDP :9002  ◄─────► bridge.py ◄──MAVLink──► dummy_fc.py   │
│   MAVLink  :14550 ◄─────►           ◄──UDP──────► (or STM32)    │
└───────────────────────────────────────────────────────────────────┘
```

---

## Features

- **MuJoCo rigid-body dynamics** at 1 kHz with a live GUI viewer at 60 Hz
- **Four virtual sensors** generated from sim state: IMU (1 kHz), GPS (10 Hz),
  LiDAR (10 Hz, single-beam + 2-D horizontal scan), Camera (30 Hz off-screen RGB)
- **MAVLink v2** for all command and telemetry links
- **ArduPilot SITL** as the PC-side autopilot (EKF, navigation, outer-loop control)
- **Staged learning path** — fly with a Python dummy FC first, swap to STM32 later
- **NED convention** internally with clearly documented MuJoCo-to-NED frame transforms
- **Deterministic physics** with `dt = 0.001` and bounded wall-clock jitter

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Conda | Miniconda or Anaconda |
| OS | Windows 10/11 |
| GPU | Any with OpenGL (for MuJoCo viewer) |
| ArduPilot SITL | Pre-built `ArduCopter.exe` or built from source in WSL2 |

---

## Quick Start

### 1. Create the local conda environment

The environment is created **inside** the `hil/` folder (`.conda-env/`) so
everything stays self-contained.

```powershell
cd hil
conda env create --prefix ./.conda-env --file environment.yml
conda activate ./.conda-env
```

Verify MuJoCo is working:

```powershell
python -c "import mujoco; import mujoco.viewer; print('MuJoCo', mujoco.__version__, 'OK')"
```

> **Updating the environment** after `environment.yml` changes:
> ```powershell
> conda env update --prefix ./.conda-env --file environment.yml --prune
> ```
>
> **Removing the environment:**
> ```powershell
> conda deactivate
> conda env remove --prefix ./.conda-env
> ```

### 2. Stage 1 — Smoke test (no ArduPilot needed)

```powershell
python bridge.py --stage 1
```

A MuJoCo window opens. The drone lifts off and hovers at ~1.5 m using a
built-in controller. The console prints the physics loop rate (target: 1000 Hz).
Close the viewer window or press `Ctrl+C` to stop.

### 3. Stage 2 — Full pipeline (ArduPilot + Dummy FC)

Open **three** terminals:

**Terminal 1 — ArduPilot SITL:**

```powershell
cd C:\ardupilot-sitl
.\ArduCopter.exe --model JSON -I0 --home -35.3632621,149.1652374,584,0
```

> **Getting ArduCopter.exe:** Download the Windows SITL binary from
> https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/ or build
> from source in WSL2 (see [docs/BUILD_AND_RUN.md](docs/BUILD_AND_RUN.md)).

**Terminal 2 — MuJoCo Bridge:**

```powershell
cd hil
python bridge.py --stage 2
```

Wait until the console prints `AP=connected`.

**Terminal 3 — Dummy Flight Controller:**

```powershell
cd hil
python dummy_fc.py
```

The FC prints periodic status (setpoints, gyro readings, motor outputs).

### 4. Fly a mission

Connect a GCS to ArduPilot on UDP port **14551**:

| GCS | Connection string |
|-----|-------------------|
| QGroundControl | UDP 127.0.0.1:14551 |
| Mission Planner | UDP 127.0.0.1:14551 |

Or script with pymavlink:

```python
from pymavlink import mavutil

m = mavutil.mavlink_connection('udp:127.0.0.1:14551')
m.wait_heartbeat()
print("ArduPilot connected")

# Arm
m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)

# Takeoff to 5 m
m.mav.command_long_send(1, 1, 22, 0, 0, 0, 0, 0, 0, 0, 5)
```

The drone should lift off and hover visibly in the MuJoCo GUI.

---

## Project Structure

```
hil/
├── bridge.py              Main entry — MuJoCo physics + viewer + MAVLink routing
├── dummy_fc.py            Python dummy flight controller (Stage 2)
├── sensors.py             Sensor generation from MuJoCo state (IMU, GPS, LiDAR, camera)
├── config.py              Single source of truth for all constants
├── environment.yml        Conda environment definition (local prefix install)
├── .conda-env/            Local conda environment (created by conda, git-ignored)
├── models/
│   └── quadcopter.xml     MuJoCo MJCF model (body, motors, sensors, environment)
└── docs/
    ├── ARCHITECTURE.md    Data-flow diagrams, MAVLink tables, timing diagrams
    ├── STM32_PLAN.md      Stage 3 firmware plan (UART DMA, timer ISR, C skeletons)
    └── BUILD_AND_RUN.md   Detailed install + validation checklists
```

---

## Architecture

### Three-Stage Evolution

| Stage | Flight Controller | Transport | Purpose |
|-------|-------------------|-----------|---------|
| **1** | Built-in `SimpleHoverController` | In-process | Verify model, motors, viewer |
| **2** | `dummy_fc.py` (Python + pymavlink) | MAVLink over UDP | End-to-end pipeline, PID tuning |
| **3** | STM32F407 firmware (C) | MAVLink over Serial | Real embedded target |

Stages 2 and 3 use the **exact same MAVLink message contract** — the swap
requires changing only the transport layer (UDP to UART serial).

### Control Loop Split

```
ArduPilot SITL (PC)                    Flight Controller
───────────────────                    ─────────────────
EKF state estimation                   Rate PID (p, q, r)
Position controller                    Quad-X motor mixer
Velocity controller                    Failsafe (stale setpoint > 50 ms)
Attitude controller
       │                                       ▲
       ▼                                       │
  ATTITUDE_TARGET ──► Bridge ──► SET_ATTITUDE_TARGET
  (body rates + thrust)          (body rates + thrust)
                                       │
                                       ▼
                                  SERVO_OUTPUT_RAW ──► Bridge ──► MuJoCo
                                  (motor PWM 1000-2000)          data.ctrl
```

### Data Flow Summary

| Path | Protocol | Port | Messages |
|------|----------|------|----------|
| Bridge → ArduPilot | JSON UDP | :9002 | `{timestamp, imu, position, attitude, velocity}` |
| Bridge → ArduPilot | MAVLink UDP | :14550 | `DISTANCE_SENSOR` (132) for LiDAR |
| ArduPilot → Bridge | MAVLink UDP | :14550 | `ATTITUDE_TARGET` (83) |
| Bridge → FC | MAVLink UDP | :14561 | `SET_ATTITUDE_TARGET` (82), `SCALED_IMU` (26) |
| FC → Bridge | MAVLink UDP | :14560 | `SERVO_OUTPUT_RAW` (36), `HEARTBEAT` (0) |
| All entities | MAVLink | various | `HEARTBEAT` (0) at 1 Hz |

### MAVLink System IDs

| Entity | sysid | compid |
|--------|-------|--------|
| ArduPilot SITL | 1 | 1 |
| MuJoCo Bridge | 2 | 1 |
| Flight Controller (Dummy FC *and* STM32) | 3 | 1 |

---

## Coordinate Frames

| Frame | X | Y | Z |
|-------|---|---|---|
| MuJoCo world | Forward (North) | Left (West) | **Up** |
| NED | North | East | **Down** |

**Transform:** `v_ned = diag(1, -1, -1) * v_mujoco` (180-degree rotation about X).

Body-frame angular velocity: `p = wx`, `q = -wy`, `r = -wz`.

All frame transforms are defined in `config.py` (`MJ_TO_NED`, `BODY_GYRO_MJ_TO_NED`)
and applied in `sensors.py`.

---

## Motor Layout (Quad-X)

```
         Front (+X)
    M3 (CCW)    M1 (CW)
    Blue         Red
        ╲       ╱
          ╲   ╱
            ╳
          ╱   ╲
        ╱       ╲
    M2 (CW)    M4 (CCW)
    Green       Yellow
         Rear (-X)
```

| Motor | Position | Spin | Mixer row |
|-------|----------|------|-----------|
| M1 | Front-Right | CW | `+Thr -Roll +Pitch -Yaw` |
| M2 | Back-Left | CW | `+Thr +Roll -Pitch -Yaw` |
| M3 | Front-Left | CCW | `+Thr +Roll +Pitch +Yaw` |
| M4 | Back-Right | CCW | `+Thr -Roll -Pitch +Yaw` |

---

## Sensor Summary

| Sensor | Source | Rate | MAVLink / Transport |
|--------|--------|------|---------------------|
| IMU (gyro + accel) | MuJoCo built-in `accelerometer` + `gyro` sensors | 1 kHz | JSON to ArduPilot, `SCALED_IMU` to FC |
| GPS | Flat-earth conversion from MuJoCo position | 10 Hz | JSON to ArduPilot |
| LiDAR (altitude) | MuJoCo `rangefinder` sensor (downward) | 10 Hz | `DISTANCE_SENSOR` to ArduPilot |
| LiDAR (2-D scan) | `mj_ray()` raycasting, 36 rays horizontal | 10 Hz | Available in `sensors.py`, not routed yet |
| Camera (RGB) | `mujoco.Renderer` off-screen render | 30 Hz | Side-channel (UDP :14570); metadata via MAVLink |

---

## Configuration

All tunable parameters live in **`config.py`** — the single source of truth
shared by bridge, dummy FC, and (by convention) the future STM32 firmware.

Key sections:

| Section | What it controls |
|---------|-----------------|
| MAVLink IDs | `ARDUPILOT_SYSID`, `BRIDGE_SYSID`, `FC_SYSID` |
| UDP Ports | `AP_JSON_BIND_PORT`, `AP_MAVLINK_PORT`, `FC_LISTEN_PORT`, `BRIDGE_FC_LISTEN_PORT` |
| Timing | `PHYSICS_DT`, `RENDER_FPS`, sensor `*_RATE_HZ` |
| Vehicle | `VEHICLE_MASS_KG`, `ARM_LENGTH_M`, `MOTOR_MAX_THRUST_N`, `HOVER_THROTTLE` |
| Mixer | `MIXER` (4x4 numpy matrix) |
| PID | `RATE_PID` dict with `kp`, `ki`, `kd`, `imax` per axis |
| Failsafe | `SETPOINT_TIMEOUT_MS`, `FAILSAFE_MOTOR_OUTPUT` |
| Frames | `MJ_TO_NED`, `BODY_GYRO_MJ_TO_NED` |

---

## Dummy FC Usage (Stage 2)

```powershell
python dummy_fc.py              # 1 kHz (default)
python dummy_fc.py --rate 500   # 500 Hz (if 1 kHz overruns)
```

The dummy FC:
- Binds to UDP **:14561** (receives setpoints + IMU from bridge)
- Sends to UDP **:14560** (motor outputs + heartbeat to bridge)
- Runs rate PID with derivative-on-measurement and integrator anti-windup
- Enforces failsafe: motors go to 0 if no setpoint arrives within 50 ms
- Prints periodic status every 5 seconds

### PID Tuning

Edit `RATE_PID` in `config.py`. Default gains:

```
roll:   kp=0.08  ki=0.02  kd=0.003
pitch:  kp=0.08  ki=0.02  kd=0.003
yaw:    kp=0.15  ki=0.05  kd=0.000
```

If the drone oscillates, reduce `kp` by 50%. If it drifts slowly, increase `ki`.

---

## Stage 3 — STM32 Swap

See [docs/STM32_PLAN.md](docs/STM32_PLAN.md) for the full firmware plan.

The swap is transport-only:

```powershell
# Stop dummy FC, connect STM32 via USB-Serial
python bridge.py --stage 3 --serial COM3
```

The STM32 firmware must:
- Use **sysid=3, compid=1** (same as dummy FC)
- Consume `SET_ATTITUDE_TARGET` (82) and `SCALED_IMU` (26)
- Produce `SERVO_OUTPUT_RAW` (36) and `HEARTBEAT` (0)
- Run at 921600 baud, MAVLink v2, UART with DMA circular RX

---

## Troubleshooting

### Physics rate below 1 kHz

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| 500-800 Hz | Viewer overhead | Lower `RENDER_FPS` to 30 in `config.py` |
| 200-400 Hz | LiDAR raycasting | Lower `LIDAR_NUM_RAYS` or `LIDAR_RATE_HZ` |
| < 200 Hz | Camera rendering | Lower `CAMERA_RATE_HZ` or don't call `get_camera_frame()` |

### ArduPilot won't connect

- Must be started with `--model JSON` flag.
- Check port 9002 is free: `netstat -ano | findstr 9002`
- ArduPilot may take 5-10 s to initialize before sending JSON.

### FC stuck in FAILSAFE

- Bridge must be running in `--stage 2` (not stage 1).
- ArduPilot must be **armed** and in a flight mode (GUIDED, LOITER, AUTO)
  to produce rate setpoints.
- Verify the bridge requests `ATTITUDE_TARGET` streaming (happens automatically
  on first ArduPilot heartbeat).

### Drone flips on takeoff

- Check motor numbering: `MIXER` in `config.py` must match actuator order
  in `quadcopter.xml`.
- Check spin directions: `gear` Z-torque sign in XML must match the
  CW/CCW convention.
- Check gyro signs: `BODY_GYRO_MJ_TO_NED` must be `diag(1, -1, -1)`.

### Sniffing MAVLink traffic

```powershell
pip install MAVProxy
mavproxy.py --master=udp:127.0.0.1:14550 --console
```

---

## Further Documentation

| Document | Contents |
|----------|----------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Mermaid data-flow + timing diagrams, full MAVLink message tables, coordinate frame math |
| [docs/STM32_PLAN.md](docs/STM32_PLAN.md) | STM32F407 firmware architecture, UART DMA config, timer ISR, C code skeletons |
| [docs/BUILD_AND_RUN.md](docs/BUILD_AND_RUN.md) | Step-by-step install, ArduPilot SITL setup options, per-stage validation checklists |

---

## License

Private project — not yet licensed for distribution.
