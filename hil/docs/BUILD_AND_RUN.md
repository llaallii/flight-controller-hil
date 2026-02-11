# Build & Run Instructions (WSL / Linux)

## Prerequisites

- Conda (Miniconda or Anaconda)
- Git (for ArduPilot)
- Ubuntu packages for ArduPilot build (installed by `install-prereqs-ubuntu.sh`)

---

## Step 1 — Create the Local Conda Environment

The environment installs into `hil/.conda-env/` so it stays self-contained.

```bash
cd hil
conda env create --prefix ./.conda-env --file environment.yml
conda activate ./.conda-env
```

Verify MuJoCo viewer works:

```bash
python -c "import mujoco; import mujoco.viewer; print('MuJoCo', mujoco.__version__, 'OK')"
```

To update after editing `environment.yml`:

```bash
conda env update --prefix ./.conda-env --file environment.yml --prune
```

---

## Step 2 — Build ArduPilot SITL in WSL / Linux

```bash
# Clone ArduPilot (if not already done)
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Install build prerequisites (one-time)
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build SITL
./waf configure --board sitl
./waf copter
```

The built binary lands in `ardupilot/build/sitl/bin/arducopter`.
You can also use `sim_vehicle.py` which handles the build and launch automatically.

---

## Step 3 — Stage 1: Verify MuJoCo Model & Viewer

This stage uses a built-in hover controller — no ArduPilot, no FC needed.

```bash
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

Open **four terminals** and start them **in this order**:

### Terminal 1: ArduPilot SITL (start first)

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v Copter --model JSON -I0 \
  -l -35.3632621,149.1652374,584,0 \
  --out=127.0.0.1:14550 --out=127.0.0.1:14551
```

> **WSL2 note:** The `--out=127.0.0.1:...` flags are **required** on WSL2.
> Without them, MavProxy auto-detects the Windows host IP (e.g. `172.x.x.x`)
> and sends MAVLink there instead of localhost. The bridge and notebook will
> never receive MAVLink messages.

Wait for the build to finish and MAVProxy to print its prompt.

### Terminal 2: MuJoCo Bridge (start second)

```bash
cd hil
python bridge.py --stage 2
```

The bridge will:
- Bind to UDP :9002 (JSON backend for ArduPilot).
- Bind to UDP :14550 (MAVLink from ArduPilot).
- Send setpoints to UDP :14561 (Dummy FC).
- Listen on UDP :14560 (motor outputs from FC).

Wait until console shows `AP=connected`.

### Terminal 3: Dummy Flight Controller (start third)

```bash
cd hil
python dummy_fc.py
```

Wait until the bridge console shows `FC=ok`.

### Terminal 4: Flying a Mission (start last)

```bash
cd hil
python fly_mission.py
```

The script connects to ArduPilot, arms, and interactively asks for takeoff
altitude and waypoints (N,E offsets in metres). Ctrl+C triggers immediate
LAND for safety.

Or connect a GCS (Mission Planner, QGroundControl) to ArduPilot's MAVLink port:

```
QGC connection: UDP 127.0.0.1:14551  (ArduPilot's second GCS output)
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
4. Find the serial device (e.g., `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`).
5. Restart bridge:

```bash
python bridge.py --stage 3 --serial /dev/ttyUSB0
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
- Try `ss -tulnp | grep 9002` to check port usage.
- ArduPilot may take 5–10 seconds to initialize before sending JSON.

### WSL2: Bridge never gets ATTITUDE_TARGET / notebook hangs on wait_heartbeat

On WSL2, `sim_vehicle.py` auto-detects the Windows host IP (e.g. `172.x.x.x`)
and routes MavProxy's MAVLink output there instead of localhost. The bridge
and notebook never receive any MAVLink messages.

**Fix:** always pass explicit `--out` flags pointing to `127.0.0.1`:

```bash
Tools/autotest/sim_vehicle.py -v Copter --model JSON -I0 \
  -l -35.3632621,149.1652374,584,0 \
  --out=127.0.0.1:14550 --out=127.0.0.1:14551
```

### Dummy FC shows "FAILSAFE"

- Means no SET_ATTITUDE_TARGET received within 50 ms.
- Check that bridge is running in Stage 2 (`--stage 2`).
- Check that ArduPilot is outputting ATTITUDE_TARGET: the bridge requests
  this stream when it first receives a heartbeat from ArduPilot.
- Ensure ArduPilot is armed and in a flight mode (GUIDED, LOITER, AUTO).
  Un-armed ArduPilot may not output rate commands.
- **On WSL2:** see the note above about `--out` flags.

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
  use software rendering: `export MUJOCO_GL=egl` or try
  `export MUJOCO_GL=osmesa` before running.
- On WSL2, ensure a display server (e.g., WSLg) is available for the GUI viewer.

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

```bash
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
