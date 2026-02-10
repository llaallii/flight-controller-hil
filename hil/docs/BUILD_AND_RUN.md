# Build & Run Instructions (Windows)

## Prerequisites

- Conda (Miniconda or Anaconda)
- Git (for ArduPilot)

---

## Step 1 — Create the Local Conda Environment

The environment installs into `hil/.conda-env/` so it stays self-contained.

```powershell
cd hil
conda env create --prefix ./.conda-env --file environment.yml
conda activate ./.conda-env
```

Verify MuJoCo viewer works:

```powershell
python -c "import mujoco; import mujoco.viewer; print('MuJoCo', mujoco.__version__, 'OK')"
```

To update after editing `environment.yml`:

```powershell
conda env update --prefix ./.conda-env --file environment.yml --prune
```

---

## Step 2 — Install ArduPilot SITL on Windows

### Option A: Pre-built binary (simplest)

1. Download the latest ArduCopter SITL for Windows:
   - Go to https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/
   - Download `ArduCopter.exe` (or the zip containing it)
   - Place it in a known directory, e.g. `C:\ardupilot-sitl\`

2. Create a default parameter file `copter.parm` alongside the exe:
   ```
   # Minimal params for JSON backend
   FRAME_CLASS 1
   FRAME_TYPE  1
   ```

### Option B: Build from source (WSL2)

```bash
# In WSL2 Ubuntu:
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

Run from WSL2 — UDP still reaches Windows localhost.

---

## Step 3 — Stage 1: Verify MuJoCo Model & Viewer

This stage uses a built-in hover controller — no ArduPilot, no FC needed.

```powershell
cd hil
python bridge.py --stage 1
```

**Expected result:**
- MuJoCo GUI window opens showing the quadcopter.
- Drone lifts off and hovers at ~1.5 m altitude.
- Console prints position and loop rate (~1000 Hz).
- You can rotate the camera with mouse in the viewer.

**What to verify:**
- [ ] Drone lifts off (positive Z).
- [ ] Drone remains stable (no wild oscillation).
- [ ] Motor directions correct (nose points +X, forward).
- [ ] Console shows rate close to 1000 Hz.
- [ ] No significant overruns reported.

Press Ctrl+C or close the viewer window to stop.

---

## Step 4 — Stage 2: Full Pipeline (ArduPilot + Dummy FC)

Open **three terminals** (PowerShell or CMD):

### Terminal 1: ArduPilot SITL

```powershell
cd C:\ardupilot-sitl
.\ArduCopter.exe --model JSON -I0 --home -35.3632621,149.1652374,584,0
```

ArduPilot will start and wait for the JSON backend to connect on UDP :9002.
It also opens MAVLink output on UDP :14550.

### Terminal 2: MuJoCo Bridge

```powershell
cd hil
python bridge.py --stage 2
```

The bridge will:
- Bind to UDP :9002 (JSON backend for ArduPilot).
- Bind to UDP :14550 (MAVLink from ArduPilot).
- Send setpoints to UDP :14561 (Dummy FC).
- Listen on UDP :14560 (motor outputs from FC).

Console should show "AP=connected" after ArduPilot links up.

### Terminal 3: Dummy Flight Controller

```powershell
cd hil
python dummy_fc.py
```

Console should show periodic status logs with setpoints and motor values.

### Flying a Mission

Connect a GCS (Mission Planner, QGroundControl) to ArduPilot's MAVLink port
to upload waypoints, arm, and fly:

```
QGC connection: UDP :14551  (ArduPilot's second GCS output)
```

Or use pymavlink scripting:

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection('udp:127.0.0.1:14551')
m.wait_heartbeat()

# Arm
m.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)

# Takeoff to 5m
m.mav.command_long_send(1, 1, 22, 0, 0, 0, 0, 0, 0, 0, 5)
```

**What to verify:**
- [ ] ArduPilot connects (bridge prints "AP=connected").
- [ ] Dummy FC receives setpoints (FC prints non-zero sp values).
- [ ] Motors spin (FC prints non-zero motor values).
- [ ] Drone flies visibly in MuJoCo GUI.
- [ ] Altitude holds after takeoff.
- [ ] No failsafe triggers during flight.

---

## Step 5 — Stage 3: STM32 Swap (future)

1. Stop `dummy_fc.py` (Ctrl+C).
2. Flash STM32F407 with firmware matching the same MAVLink contract.
3. Connect STM32 USART2 to PC via USB-Serial adapter.
4. Note the COM port (e.g., COM3).
5. Restart bridge:

```powershell
python bridge.py --stage 3 --serial COM3
```

6. Verify FC heartbeat appears in bridge logs.
7. Arm and fly as before — same ArduPilot, same MuJoCo model.

---

## Troubleshooting

### Physics loop below 1 kHz

| Symptom | Cause | Fix |
|---------|-------|-----|
| Rate ~500–800 Hz | Viewer sync overhead | Reduce `RENDER_FPS` in config.py to 30 |
| Rate ~200–400 Hz | LiDAR raycasting too heavy | Reduce `LIDAR_NUM_RAYS` or `LIDAR_RATE_HZ` |
| Rate ~100 Hz | Camera rendering blocking | Reduce `CAMERA_RATE_HZ` or disable camera |

### ArduPilot won't connect

- Ensure ArduPilot is started with `--model JSON`.
- Check that no other process is binding UDP :9002.
- Try `netstat -ano | findstr 9002` to check port usage.
- ArduPilot may take 5–10 seconds to initialize before sending JSON.

### Dummy FC shows "FAILSAFE"

- Means no SET_ATTITUDE_TARGET received within 50 ms.
- Check that bridge is running in Stage 2 (`--stage 2`).
- Check that ArduPilot is outputting ATTITUDE_TARGET: the bridge requests
  this stream when it first receives a heartbeat from ArduPilot.
- Ensure ArduPilot is armed and in a flight mode (GUIDED, LOITER, AUTO).
  Un-armed ArduPilot may not output rate commands.

### Drone flips immediately

- Motor numbering mismatch: verify `MIXER` matrix in config.py matches
  the MJCF motor site positions.
- Spin direction wrong: check `gear` signs in quadcopter.xml actuators
  match the Quad-X convention.
- Gyro sign wrong: check `BODY_GYRO_MJ_TO_NED` transform in config.py.

### Drone drifts or oscillates

- PID gains too aggressive: reduce `kp` in `RATE_PID` by 50%.
- Loop rate too low: ensure Dummy FC runs at 1 kHz. Use `--rate 500`
  if 1 kHz isn't achievable on your machine.
- Latency: check that bridge forwards setpoints immediately (every tick
  with new ATTITUDE_TARGET, not batched).

### MuJoCo viewer freezes / black screen

- Ensure GPU drivers are up to date.
- `launch_passive` requires a GPU. If running in a VM or remote desktop,
  use software rendering: `set MUJOCO_GL=egl` (Linux) or try
  `set MUJOCO_GL=osmesa` before running.
- On Windows with multiple GPUs, ensure the correct GPU is selected.

### MAVLink bandwidth concerns

At 1 kHz with full-size messages:

| Message | Size (bytes) | Rate | Bandwidth |
|---------|-------------|------|-----------|
| SET_ATTITUDE_TARGET | ~49 | 250 Hz | ~12 KB/s |
| SCALED_IMU | ~34 | 1000 Hz | ~34 KB/s |
| SERVO_OUTPUT_RAW | ~30 | 1000 Hz | ~30 KB/s |
| HEARTBEAT | ~17 | 1 Hz | negligible |
| **Total Bridge↔FC** | | | **~76 KB/s** |

This is well within UDP localhost and 921600-baud serial capacity
(~92 KB/s usable at 921600 baud).

### Verifying MAVLink messages

Use `mavproxy.py` to sniff traffic:

```powershell
pip install MAVProxy   # inside the activated .conda-env
mavproxy.py --master=udp:127.0.0.1:14550 --console
```

In the MAVProxy console, type `status` to see received messages.

---

## Validation Checklist

### Stage 1

- [ ] MuJoCo viewer opens and renders the quadcopter scene.
- [ ] Drone lifts to ~1.5 m and hovers.
- [ ] Physics rate is stable at ~1000 Hz (check console).
- [ ] Viewer is responsive (camera rotation works smoothly).
- [ ] Closing the viewer window stops the bridge cleanly.

### Stage 2

- [ ] ArduPilot JSON backend connects (bridge shows "AP=connected").
- [ ] Bridge receives ATTITUDE_TARGET from ArduPilot.
- [ ] Dummy FC receives SET_ATTITUDE_TARGET and SCALED_IMU.
- [ ] Dummy FC produces non-zero motor outputs.
- [ ] Drone flies in MuJoCo GUI under ArduPilot control.
- [ ] GCS can connect and display telemetry.
- [ ] Arm → Takeoff → Hover → Land cycle works.
- [ ] Failsafe triggers if Dummy FC is killed mid-flight.
- [ ] Failsafe clears when Dummy FC restarts and resumes.

### Stage 3

- [ ] Bridge connects to STM32 over serial.
- [ ] STM32 heartbeat received by bridge.
- [ ] Same arm/takeoff/hover/land cycle works.
- [ ] Loop timing on STM32 is stable at 1 kHz (measure with scope on debug pin).
