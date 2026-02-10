"""
Sensor Generation from MuJoCo State
=====================================
Extracts IMU, GPS, LiDAR, and camera data from the MuJoCo simulation,
converts to NED-convention MAVLink-ready values.

Coordinate frames
-----------------
MuJoCo world:   X-fwd, Y-left, Z-up   (right-handed)
NED world:      X-north, Y-east, Z-down
Body NED:       X-fwd, Y-right, Z-down

Transform:  v_ned = diag(1, -1, -1) @ v_mujoco   (180-deg rotation about X)
"""

import math
import numpy as np
import mujoco
import config as cfg

# Earth radius for flat-earth lat/lon conversion (metres)
_EARTH_R = 6378137.0


class SensorGenerator:
    """Generates sensor readings from MuJoCo model/data at each physics tick."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.m = model
        self.d = data

        # Sensor-data slice indices (populated by _resolve_sensors)
        self._accel_adr = None
        self._gyro_adr = None
        self._range_adr = None
        self._resolve_sensors()

        # Drone body id (for raycasting exclusion)
        self.drone_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")

        # Camera renderer (lazy-init on first call)
        self._renderer = None

        # LiDAR ray directions cache (body-frame, horizontal plane)
        n = cfg.LIDAR_NUM_RAYS
        angles = np.linspace(0, 2 * math.pi, n, endpoint=False)
        # Rays in body frame (MuJoCo: X-fwd, Y-left, Z-up) → horizontal sweep
        self._lidar_dirs_body = np.column_stack([
            np.cos(angles), np.sin(angles), np.zeros(n)
        ])  # shape (N, 3)

    # ──────────────────────────────────────────────────────────────────────
    #  Internal helpers
    # ──────────────────────────────────────────────────────────────────────
    def _resolve_sensors(self):
        """Find sensor-data addresses by name."""
        for i in range(self.m.nsensor):
            name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_SENSOR, i)
            adr = self.m.sensor_adr[i]
            dim = self.m.sensor_dim[i]
            if name == "imu_accel":
                self._accel_adr = (adr, adr + dim)
            elif name == "imu_gyro":
                self._gyro_adr = (adr, adr + dim)
            elif name == "lidar_alt":
                self._range_adr = (adr, adr + dim)

    def _body_rotation_matrix(self) -> np.ndarray:
        """Return 3x3 rotation matrix (body → MuJoCo-world) for the drone."""
        # MuJoCo stores body quaternion as [w, x, y, z]
        quat = self.d.qpos[3:7].copy()
        R = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(R, quat)
        return R.reshape(3, 3)

    def _euler_from_quat_ned(self) -> tuple:
        """
        Return (roll, pitch, yaw) in NED convention from MuJoCo quat.
        roll  = rotation about body-X  (right-wing-down positive)
        pitch = rotation about body-Y  (nose-up positive)
        yaw   = rotation about body-Z  (clockwise-from-above positive in NED)
        """
        R_body_mj = self._body_rotation_matrix()  # body → MuJoCo-world
        # Convert to NED: R_ned = T @ R_mj @ T^T  where T = diag(1,-1,-1)
        T = cfg.MJ_TO_NED
        R_ned = T @ R_body_mj @ T  # T is its own inverse

        # ZYX Euler extraction from rotation matrix (aerospace sequence)
        # R_ned = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        pitch = -math.asin(np.clip(R_ned[2, 0], -1.0, 1.0))
        if abs(R_ned[2, 0]) < 0.9999:
            roll = math.atan2(R_ned[2, 1], R_ned[2, 2])
            yaw = math.atan2(R_ned[1, 0], R_ned[0, 0])
        else:
            roll = math.atan2(-R_ned[1, 2], R_ned[1, 1])
            yaw = 0.0
        return roll, pitch, yaw

    # ──────────────────────────────────────────────────────────────────────
    #  IMU  (1 kHz — every tick)
    # ──────────────────────────────────────────────────────────────────────
    def get_imu_ned(self):
        """
        Returns
        -------
        gyro_ned  : ndarray (3,)  [p, q, r] rad/s in body-NED
        accel_ned : ndarray (3,)  [ax, ay, az] m/s² in body-NED (specific force)
        """
        # MuJoCo sensor outputs are already in body frame
        gyro_mj = self.d.sensordata[self._gyro_adr[0]:self._gyro_adr[1]].copy()
        accel_mj = self.d.sensordata[self._accel_adr[0]:self._accel_adr[1]].copy()

        # Transform body-frame vectors:  NED-body Y = -MuJoCo-body Y, Z = -Z
        gyro_ned = cfg.BODY_GYRO_MJ_TO_NED @ gyro_mj
        accel_ned = cfg.BODY_ACCEL_MJ_TO_NED @ accel_mj
        return gyro_ned, accel_ned

    # ──────────────────────────────────────────────────────────────────────
    #  GPS  (5–10 Hz)
    # ──────────────────────────────────────────────────────────────────────
    def get_gps(self):
        """
        Returns
        -------
        lat, lon  : float   degrees (WGS-84)
        alt_amsl  : float   metres above mean sea level
        vn, ve, vd: float   m/s   NED velocity
        num_sats  : int
        hdop      : float
        """
        # Position in MuJoCo world (metres, Z-up)
        pos_mj = self.d.qpos[0:3].copy()     # [x_fwd, y_left, z_up]
        vel_mj = self.d.qvel[0:3].copy()     # [vx, vy, vz] world-frame

        # Flat-earth: north = MuJoCo X, east = -MuJoCo Y
        north_m = pos_mj[0]
        east_m = -pos_mj[1]
        down_m = -pos_mj[2]

        lat = cfg.GPS_HOME_LAT_DEG + math.degrees(north_m / _EARTH_R)
        lon = cfg.GPS_HOME_LON_DEG + math.degrees(
            east_m / (_EARTH_R * math.cos(math.radians(cfg.GPS_HOME_LAT_DEG)))
        )
        alt_amsl = cfg.GPS_HOME_ALT_AMSL_M - down_m   # up is positive alt

        vn = vel_mj[0]
        ve = -vel_mj[1]
        vd = -vel_mj[2]

        return lat, lon, alt_amsl, vn, ve, vd, 12, 0.8  # sats=12, hdop=0.8

    # ──────────────────────────────────────────────────────────────────────
    #  LiDAR  (10 Hz)
    # ──────────────────────────────────────────────────────────────────────
    def get_lidar_altitude(self) -> float:
        """Single-beam downward rangefinder (metres). -1 if no hit."""
        if self._range_adr is not None:
            val = self.d.sensordata[self._range_adr[0]]
            if val < 0:
                return -1.0
            return float(val)
        return -1.0

    def get_lidar_2d_scan(self) -> np.ndarray:
        """
        Horizontal 2-D LiDAR scan around the drone.

        Returns
        -------
        ranges : ndarray (N,) — range in metres for each ray (-1 = no hit).
        """
        n = cfg.LIDAR_NUM_RAYS
        ranges = np.full(n, -1.0, dtype=np.float64)

        # Drone position in world frame
        pos = self.d.qpos[0:3].copy()
        R = self._body_rotation_matrix()  # body → world

        geomid = np.array([-1], dtype=np.int32)

        for i in range(n):
            # Transform ray direction from body to world
            ray_world = R @ self._lidar_dirs_body[i]

            dist = mujoco.mj_ray(
                self.m, self.d,
                pos,                    # ray origin
                ray_world,              # ray direction (world frame)
                None,                   # geomgroup (None = all)
                1,                      # flg_static: include static geoms
                self.drone_body_id,     # bodyexclude: skip drone itself
                geomid,                 # output: hit geom id
            )
            if dist >= 0 and dist <= cfg.LIDAR_MAX_RANGE_M:
                ranges[i] = dist

        return ranges

    # ──────────────────────────────────────────────────────────────────────
    #  Camera  (30 Hz — rendered off-screen)
    # ──────────────────────────────────────────────────────────────────────
    def get_camera_frame(self) -> np.ndarray:
        """
        Render an RGB frame from the FPV camera.

        Returns
        -------
        frame : ndarray (H, W, 3) uint8 RGB.
        """
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.m, height=cfg.CAMERA_HEIGHT, width=cfg.CAMERA_WIDTH
            )

        self._renderer.update_scene(self.d, camera=cfg.CAMERA_NAME)
        frame = self._renderer.render()
        return frame  # (H, W, 3) uint8

    # ──────────────────────────────────────────────────────────────────────
    #  JSON state dict for ArduPilot SITL backend
    # ──────────────────────────────────────────────────────────────────────
    def get_ardupilot_json_state(self, sim_time_s: float) -> dict:
        """
        Build the JSON state dict expected by ArduPilot's SIM_JSON backend.
        All values in ArduPilot's NED / body-NED convention.
        """
        gyro_ned, accel_ned = self.get_imu_ned()
        lat, lon, alt, vn, ve, vd, _, _ = self.get_gps()
        roll, pitch, yaw = self._euler_from_quat_ned()

        return {
            "timestamp": sim_time_s,
            "imu": {
                "gyro":       [float(gyro_ned[0]), float(gyro_ned[1]), float(gyro_ned[2])],
                "accel_body": [float(accel_ned[0]), float(accel_ned[1]), float(accel_ned[2])],
            },
            "position": [lat, lon, alt],
            "attitude": [roll, pitch, yaw],
            "velocity": [vn, ve, vd],
        }
