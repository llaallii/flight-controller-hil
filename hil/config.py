"""
HIL Multirotor Project — Shared Configuration
================================================
All constants shared between bridge.py, dummy_fc.py, and future STM32 firmware.
Changing a value here propagates everywhere; the STM32 firmware must mirror
the MAVLink IDs, message types, and rates defined below.
"""

import math

# ═══════════════════════════════════════════════════════════════════════════════
#  MAVLink System / Component IDs
# ═══════════════════════════════════════════════════════════════════════════════
ARDUPILOT_SYSID  = 1   # ArduPilot SITL default
ARDUPILOT_COMPID = 1

BRIDGE_SYSID     = 2   # MuJoCo bridge (PC)
BRIDGE_COMPID    = 1

FC_SYSID         = 3   # Flight-controller endpoint (Dummy FC *and* future STM32)
FC_COMPID        = 1

# ═══════════════════════════════════════════════════════════════════════════════
#  Network Endpoints — Stage 2 (all UDP on localhost)
# ═══════════════════════════════════════════════════════════════════════════════
# ArduPilot JSON physics backend (bidirectional state/servo exchange)
AP_JSON_BIND_PORT       = 9002      # Bridge binds here; ArduPilot sends servos to it

# ArduPilot MAVLink GCS ports (ArduPilot outputs telemetry here)
AP_MAVLINK_PORT         = 14550     # Primary GCS output — bridge listens here
AP_MAVLINK_PORT2        = 14551     # Secondary output (QGC / spare)

# Bridge <-> Flight-Controller MAVLink (UDP)
BRIDGE_FC_LISTEN_PORT   = 14560     # Bridge binds; FC sends motor outputs here
FC_LISTEN_PORT          = 14561     # FC binds; Bridge sends setpoints + IMU here

# ═══════════════════════════════════════════════════════════════════════════════
#  Stage 3 — Serial transport (replaces UDP between Bridge and STM32)
# ═══════════════════════════════════════════════════════════════════════════════
STM32_SERIAL_PORT       = "COM3"    # Adjust to actual port
STM32_BAUD_RATE         = 921600    # MAVLink v2 @ 921600 baud

# ═══════════════════════════════════════════════════════════════════════════════
#  Timing
# ═══════════════════════════════════════════════════════════════════════════════
PHYSICS_DT              = 0.001     # 1 kHz physics step (MuJoCo dt)
RENDER_FPS              = 60        # GUI viewer target frame-rate
RENDER_INTERVAL_S       = 1.0 / RENDER_FPS

# ═══════════════════════════════════════════════════════════════════════════════
#  Sensor Publication Rates (Hz) — Bridge -> ArduPilot / FC
# ═══════════════════════════════════════════════════════════════════════════════
IMU_RATE_HZ             = 1000      # Gyro + accel  (every physics tick)
GPS_RATE_HZ             = 10        # Position fix
LIDAR_RATE_HZ           = 10        # 2-D scan / rangefinder
CAMERA_RATE_HZ          = 30        # RGB frame capture (metadata via MAVLink)

# Derived: ticks between sensor publications at 1 kHz physics
IMU_EVERY_N_TICKS       = max(1, round(1000 / IMU_RATE_HZ))
GPS_EVERY_N_TICKS       = max(1, round(1000 / GPS_RATE_HZ))
LIDAR_EVERY_N_TICKS     = max(1, round(1000 / LIDAR_RATE_HZ))
CAMERA_EVERY_N_TICKS    = max(1, round(1000 / CAMERA_RATE_HZ))

# Rate at which Bridge requests ATTITUDE_TARGET from ArduPilot (Hz)
AP_ATTITUDE_TARGET_HZ   = 250

# ═══════════════════════════════════════════════════════════════════════════════
#  Vehicle Physical Parameters
# ═══════════════════════════════════════════════════════════════════════════════
VEHICLE_MASS_KG         = 1.5       # Total mass
ARM_LENGTH_M            = 0.22      # Centre-of-mass to motor distance
MOTOR_MAX_THRUST_N      = 7.0       # Max thrust per motor
TORQUE_TO_THRUST_RATIO  = 0.02      # |reaction-torque| / thrust  (kτ / kf)
GRAVITY                 = 9.81

# Hover throttle (approximate)
HOVER_THROTTLE          = (VEHICLE_MASS_KG * GRAVITY) / (4.0 * MOTOR_MAX_THRUST_N)

# Motor positions in body frame (MuJoCo: X-fwd, Y-left, Z-up)
_D = ARM_LENGTH_M / math.sqrt(2.0)  # diagonal offset ≈ 0.1556 m
MOTOR_POS = {
    1: ( _D, -_D, 0.0),   # Front-Right
    2: (-_D,  _D, 0.0),   # Back-Left
    3: ( _D,  _D, 0.0),   # Front-Left
    4: (-_D, -_D, 0.0),   # Back-Right
}

# Spin direction: +1 = CCW (top-view), -1 = CW (top-view)
# CCW produces +Z reaction torque in body frame; CW produces -Z.
MOTOR_SPIN = {1: -1, 2: -1, 3: +1, 4: +1}   # 1,2 CW; 3,4 CCW

# ═══════════════════════════════════════════════════════════════════════════════
#  Quad-X Mixer Matrix  (throttle, roll, pitch, yaw) -> (m1, m2, m3, m4)
#
#  Positive roll  = right-side down   (body +X cross +Y = +Z; MuJoCo Y=left)
#  Positive pitch = nose up
#  Positive yaw   = CCW top-view  (body +Z up)
#
#         Front
#    M3(CCW)   M1(CW)
#        \     /
#         \   /
#          \ /
#          / \
#         /   \
#        /     \
#    M2(CW)   M4(CCW)
#         Rear
#
#  Motor  Throttle  Roll   Pitch   Yaw
#  ──────────────────────────────────────
#  M1 FR    +1       -1     +1     -1      (CW  → -yaw)
#  M2 BL    +1       +1     -1     -1      (CW  → -yaw)
#  M3 FL    +1       +1     +1     +1      (CCW → +yaw)
#  M4 BR    +1       -1     -1     +1      (CCW → +yaw)
# ═══════════════════════════════════════════════════════════════════════════════
import numpy as np
MIXER = np.array([
    # Thr   Roll  Pitch   Yaw
    [ 1.0, -1.0,  1.0,  -1.0],   # M1 Front-Right (CW)
    [ 1.0,  1.0, -1.0,  -1.0],   # M2 Back-Left   (CW)
    [ 1.0,  1.0,  1.0,   1.0],   # M3 Front-Left  (CCW)
    [ 1.0, -1.0, -1.0,   1.0],   # M4 Back-Right  (CCW)
], dtype=np.float64)

# ═══════════════════════════════════════════════════════════════════════════════
#  Rate PID Gains  (used by Dummy FC and later STM32)
# ═══════════════════════════════════════════════════════════════════════════════
RATE_PID = {
    "roll":  {"kp": 0.080, "ki": 0.020, "kd": 0.003, "imax": 0.30},
    "pitch": {"kp": 0.080, "ki": 0.020, "kd": 0.003, "imax": 0.30},
    "yaw":   {"kp": 0.150, "ki": 0.050, "kd": 0.000, "imax": 0.30},
}

# ═══════════════════════════════════════════════════════════════════════════════
#  Failsafe
# ═══════════════════════════════════════════════════════════════════════════════
SETPOINT_TIMEOUT_MS     = 50        # If no setpoint received within this, trigger failsafe
FAILSAFE_MOTOR_OUTPUT   = 0.0       # Disarm (0 = motors off)

# ═══════════════════════════════════════════════════════════════════════════════
#  GPS Home / Reference Point  (ArduPilot Canberra default)
# ═══════════════════════════════════════════════════════════════════════════════
GPS_HOME_LAT_DEG        = -35.3632621
GPS_HOME_LON_DEG        = 149.1652374
GPS_HOME_ALT_AMSL_M     = 584.0
GPS_HOME_HEADING_DEG    = 0.0       # Initial heading (North)

# ═══════════════════════════════════════════════════════════════════════════════
#  Frame Transform Helpers  (MuJoCo Z-up ↔ NED)
#
#  MuJoCo world:  X = North/forward,  Y = West/left,  Z = Up
#  NED:           X = North,          Y = East,        Z = Down
#
#  v_ned = T @ v_mujoco   where T = diag(1, -1, -1)
#  (180-deg rotation about X-axis)
# ═══════════════════════════════════════════════════════════════════════════════
MJ_TO_NED = np.diag([1.0, -1.0, -1.0])
NED_TO_MJ = MJ_TO_NED            # Same matrix (self-inverse)

# Body-frame angular velocity:
#   MuJoCo body: [wx, wy, wz]  with Y=left, Z=up
#   NED body:    [p,  q,  r ]  with Y=right, Z=down
#   p = wx,  q = -wy,  r = -wz
BODY_GYRO_MJ_TO_NED = np.diag([1.0, -1.0, -1.0])

# Body-frame accelerometer (same transform)
BODY_ACCEL_MJ_TO_NED = np.diag([1.0, -1.0, -1.0])

# ═══════════════════════════════════════════════════════════════════════════════
#  LiDAR Configuration
# ═══════════════════════════════════════════════════════════════════════════════
LIDAR_MAX_RANGE_M       = 40.0
LIDAR_MIN_RANGE_M       = 0.10
LIDAR_NUM_RAYS          = 36        # Rays in a 360-deg horizontal sweep (2-D)
LIDAR_FOV_DEG           = 360.0     # Full rotation for 2-D
LIDAR_ANGULAR_RES_DEG   = LIDAR_FOV_DEG / LIDAR_NUM_RAYS

# ═══════════════════════════════════════════════════════════════════════════════
#  Camera Configuration
# ═══════════════════════════════════════════════════════════════════════════════
CAMERA_WIDTH            = 320
CAMERA_HEIGHT           = 240
CAMERA_NAME             = "fpv_camera"
# Raw frames delivered via side-channel (shared-memory or UDP), not MAVLink.
# MAVLink carries only CAMERA_IMAGE_CAPTURED metadata + timestamps.
CAMERA_SHMEM_NAME       = "hil_camera_frame"   # Named shared-memory (optional)
CAMERA_UDP_PORT         = 14570                 # Alternative: raw frame UDP port
