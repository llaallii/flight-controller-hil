#!/usr/bin/env python3
"""
MuJoCo HIL Bridge
==================
Central hub connecting MuJoCo physics, ArduPilot SITL, and the Flight Controller.

Data-flow summary
-----------------
  MuJoCo  ──(state JSON)──▶  ArduPilot SITL  ──(ATTITUDE_TARGET MAVLink)──▶  Bridge
  Bridge  ──(setpoint + IMU MAVLink)──▶  Dummy FC / STM32
  FC  ──(motor outputs MAVLink)──▶  Bridge  ──(ctrl)──▶  MuJoCo actuators

Usage
-----
  Stage 1 (standalone hover test — no ArduPilot, no FC):
      python bridge.py --stage 1

  Stage 2 (full pipeline with ArduPilot + Dummy FC):
      1) Start ArduPilot SITL:  arducopter.exe --model JSON -I0
      2) python bridge.py --stage 2
      3) python dummy_fc.py

  Stage 3 (STM32 replaces Dummy FC):
      python bridge.py --stage 3 --serial COM3
"""

import argparse
import json
import math
import socket
import struct
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
from pymavlink import mavutil

import config as cfg
from sensors import SensorGenerator


# ═══════════════════════════════════════════════════════════════════════════════
#  ArduPilot JSON Backend
# ═══════════════════════════════════════════════════════════════════════════════
class ArduPilotJSONBackend:
    """
    Speaks the ArduPilot SIM_JSON protocol over UDP.
    ArduPilot sends servo outputs → we reply with state.
    """

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", cfg.AP_JSON_BIND_PORT))
        self.sock.setblocking(False)
        self.ap_addr = None          # Filled on first recv
        self.servo = [0.0] * 16     # Latest servo outputs (ignored in stage 2+)
        self.connected = False

    def poll_servo(self) -> bool:
        """Non-blocking receive of servo JSON from ArduPilot. Returns True if received."""
        try:
            raw, addr = self.sock.recvfrom(65535)
            self.ap_addr = addr
            self.connected = True
            obj = json.loads(raw.decode("utf-8", errors="replace"))
            for i in range(16):
                key = f"servo{i+1}"
                if key in obj:
                    self.servo[i] = float(obj[key])
            return True
        except (BlockingIOError, OSError):
            return False

    def send_state(self, state_dict: dict):
        """Send JSON state to ArduPilot (reply to servo request)."""
        if self.ap_addr is None:
            return
        payload = json.dumps(state_dict).encode("utf-8")
        try:
            self.sock.sendto(payload, self.ap_addr)
        except OSError:
            pass

    def close(self):
        self.sock.close()


# ═══════════════════════════════════════════════════════════════════════════════
#  MAVLink Manager
# ═══════════════════════════════════════════════════════════════════════════════
class MAVLinkManager:
    """
    Manages two MAVLink links:
      1. Bridge ↔ ArduPilot  (receive ATTITUDE_TARGET + telemetry)
      2. Bridge ↔ FC         (send setpoints/IMU, receive motor outputs)
    """

    def __init__(self, stage: int, serial_port: str | None = None):
        self.stage = stage

        # ── ArduPilot MAVLink (GCS port) ──
        # ArduPilot SITL outputs on 127.0.0.1:14550 by default.
        # We bind to that port to receive.
        self.mav_ap = None
        if stage >= 2:
            self.mav_ap = mavutil.mavlink_connection(
                f"udpin:0.0.0.0:{cfg.AP_MAVLINK_PORT}",
                source_system=cfg.BRIDGE_SYSID,
                source_component=cfg.BRIDGE_COMPID,
                dialect="common",
            )
            self.mav_ap.mav.srcSystem = cfg.BRIDGE_SYSID
            self.mav_ap.mav.srcComponent = cfg.BRIDGE_COMPID

        # ── FC MAVLink ──
        self.mav_fc_out = None   # Bridge → FC
        self.mav_fc_in = None    # FC → Bridge

        if stage >= 2:
            if stage == 3 and serial_port:
                # Stage 3: serial to STM32
                uri = f"{serial_port}:{cfg.STM32_BAUD_RATE}"
                self.mav_fc_out = mavutil.mavlink_connection(
                    uri,
                    source_system=cfg.BRIDGE_SYSID,
                    source_component=cfg.BRIDGE_COMPID,
                    dialect="common",
                )
                self.mav_fc_in = self.mav_fc_out  # Same serial link
            else:
                # Stage 2: UDP to dummy FC
                self.mav_fc_out = mavutil.mavlink_connection(
                    f"udpout:127.0.0.1:{cfg.FC_LISTEN_PORT}",
                    source_system=cfg.BRIDGE_SYSID,
                    source_component=cfg.BRIDGE_COMPID,
                    dialect="common",
                )
                self.mav_fc_in = mavutil.mavlink_connection(
                    f"udpin:0.0.0.0:{cfg.BRIDGE_FC_LISTEN_PORT}",
                    source_system=cfg.BRIDGE_SYSID,
                    source_component=cfg.BRIDGE_COMPID,
                    dialect="common",
                )

        # Latest received data
        self.attitude_target = None   # ATTITUDE_TARGET from ArduPilot
        self.motor_outputs = np.zeros(4)  # From FC (0..1 per motor)
        self.fc_heartbeat_ok = False
        self._at_rate_request_sent = False

    def request_attitude_target_stream(self):
        """Ask ArduPilot to stream ATTITUDE_TARGET at the configured rate."""
        if self.mav_ap is None or self._at_rate_request_sent:
            return
        interval_us = int(1e6 / cfg.AP_ATTITUDE_TARGET_HZ)
        # MAVLink SET_MESSAGE_INTERVAL  (msg_id 511)
        self.mav_ap.mav.command_long_send(
            cfg.ARDUPILOT_SYSID, cfg.ARDUPILOT_COMPID,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,                                          # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET,  # param1: msg id
            interval_us,                                # param2: interval µs
            0, 0, 0, 0, 0,                             # param3-7 unused
        )
        self._at_rate_request_sent = True

    def poll_ardupilot(self):
        """Non-blocking drain of ArduPilot MAVLink messages."""
        if self.mav_ap is None:
            return
        while True:
            msg = self.mav_ap.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "ATTITUDE_TARGET":
                self.attitude_target = msg
            elif mtype == "HEARTBEAT":
                # Good time to request streams if not yet done
                if not self._at_rate_request_sent:
                    self.request_attitude_target_stream()

    def poll_fc(self):
        """Non-blocking drain of FC MAVLink messages."""
        if self.mav_fc_in is None:
            return
        while True:
            msg = self.mav_fc_in.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "SERVO_OUTPUT_RAW":
                # FC sends motor PWM 1000-2000 in servo1..servo4
                self.motor_outputs[0] = np.clip((msg.servo1_raw - 1000) / 1000.0, 0, 1)
                self.motor_outputs[1] = np.clip((msg.servo2_raw - 1000) / 1000.0, 0, 1)
                self.motor_outputs[2] = np.clip((msg.servo3_raw - 1000) / 1000.0, 0, 1)
                self.motor_outputs[3] = np.clip((msg.servo4_raw - 1000) / 1000.0, 0, 1)
            elif mtype == "HEARTBEAT":
                self.fc_heartbeat_ok = True

    def send_setpoint_to_fc(self, roll_rate: float, pitch_rate: float,
                            yaw_rate: float, thrust: float, time_boot_ms: int):
        """
        Forward rate setpoints + thrust to the FC as SET_ATTITUDE_TARGET.
        Rates in rad/s (body-NED), thrust 0..1.
        """
        if self.mav_fc_out is None:
            return
        # type_mask: ignore attitude quaternion, use body rates + thrust
        type_mask = (
            0b00000111  # ignore roll/pitch/yaw attitude (bits 0-2)
        )
        self.mav_fc_out.mav.set_attitude_target_send(
            time_boot_ms,
            cfg.FC_SYSID,           # target system
            cfg.FC_COMPID,          # target component
            type_mask,
            [1.0, 0.0, 0.0, 0.0],  # quaternion (ignored per type_mask)
            roll_rate,              # body_roll_rate  rad/s
            pitch_rate,             # body_pitch_rate rad/s
            yaw_rate,               # body_yaw_rate   rad/s
            thrust,                 # thrust 0..1
        )

    def send_imu_to_fc(self, gyro_ned, accel_ned, time_boot_ms: int):
        """
        Send IMU data to FC as SCALED_IMU (msg 26).
        gyro in rad/s → millirad/s;  accel in m/s² → milli-g.
        """
        if self.mav_fc_out is None:
            return
        self.mav_fc_out.mav.scaled_imu_send(
            time_boot_ms,
            int(accel_ned[0] / 9.81 * 1000),   # xacc milli-g
            int(accel_ned[1] / 9.81 * 1000),   # yacc
            int(accel_ned[2] / 9.81 * 1000),   # zacc
            int(gyro_ned[0] * 1000),            # xgyro mrad/s
            int(gyro_ned[1] * 1000),            # ygyro
            int(gyro_ned[2] * 1000),            # zgyro
            0, 0, 0,                            # mag (unused)
        )

    def send_heartbeat(self):
        """Bridge heartbeat."""
        if self.mav_ap is not None:
            self.mav_ap.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        if self.mav_fc_out is not None:
            self.mav_fc_out.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )

    def send_lidar_to_ap(self, alt_m: float, time_boot_ms: int):
        """
        Send downward-rangefinder reading to ArduPilot as DISTANCE_SENSOR (msg 132).
        """
        if self.mav_ap is None or alt_m < 0:
            return
        self.mav_ap.mav.distance_sensor_send(
            time_boot_ms,
            int(cfg.LIDAR_MIN_RANGE_M * 100),   # min_distance cm
            int(cfg.LIDAR_MAX_RANGE_M * 100),   # max_distance cm
            int(alt_m * 100),                    # current_distance cm
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            0,                                   # id
            mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,  # downward
            255,                                 # covariance (unknown)
        )


# ═══════════════════════════════════════════════════════════════════════════════
#  Simple Hover Controller  (Stage 1 only — replaced by ArduPilot + FC later)
# ═══════════════════════════════════════════════════════════════════════════════
class SimpleHoverController:
    """
    Minimal altitude + attitude hold for Stage 1 smoke-test.
    Directly outputs motor commands (no separate FC).
    """

    def __init__(self):
        self.target_alt = 1.5   # metres (MuJoCo Z)
        self.target_yaw = 0.0
        self.alt_int = 0.0
        self.prev_alt_err = 0.0

    def update(self, model, data, dt=cfg.PHYSICS_DT) -> np.ndarray:
        """Returns motor commands [m1, m2, m3, m4] in 0..1."""
        pos = data.qpos[0:3]
        vel = data.qvel[0:3]
        quat = data.qpos[3:7]
        omega = data.qvel[3:6]   # body angular velocity in MuJoCo frame

        # Altitude PD
        alt_err = self.target_alt - pos[2]
        self.alt_int = np.clip(self.alt_int + alt_err * dt, -0.5, 0.5)
        alt_rate = vel[2]
        throttle = cfg.HOVER_THROTTLE + 0.6 * alt_err + 0.08 * self.alt_int - 0.3 * (-alt_rate)

        # Attitude from quaternion (simplified Euler extraction)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = -math.asin(np.clip(R[2, 0], -1, 1))
        yaw = math.atan2(R[1, 0], R[0, 0])

        # Attitude PD → rate commands
        roll_cmd = -3.0 * roll - 0.3 * omega[0]
        pitch_cmd = -3.0 * pitch - 0.3 * omega[1]
        yaw_err = self.target_yaw - yaw
        yaw_cmd = -1.0 * yaw_err - 0.2 * omega[2]

        # Mix
        cmd = np.array([throttle, roll_cmd, pitch_cmd, yaw_cmd])
        motors = cfg.MIXER @ cmd
        return np.clip(motors, 0.0, 1.0)


# ═══════════════════════════════════════════════════════════════════════════════
#  Main Bridge Loop
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="MuJoCo HIL Bridge")
    parser.add_argument("--stage", type=int, default=1, choices=[1, 2, 3],
                        help="1=standalone, 2=ArduPilot+DummyFC, 3=ArduPilot+STM32")
    parser.add_argument("--serial", type=str, default=None,
                        help="COM port for Stage 3 (e.g. COM3)")
    parser.add_argument("--model", type=str, default="models/quadcopter.xml",
                        help="Path to MJCF model")
    args = parser.parse_args()

    print(f"[Bridge] Starting — Stage {args.stage}")
    print(f"[Bridge] Physics dt = {cfg.PHYSICS_DT} s  ({1/cfg.PHYSICS_DT:.0f} Hz)")
    print(f"[Bridge] Render target = {cfg.RENDER_FPS} Hz")

    # ── Load MuJoCo ──
    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)

    # Verify timestep matches config
    assert abs(model.opt.timestep - cfg.PHYSICS_DT) < 1e-9, \
        f"Model dt={model.opt.timestep} != config PHYSICS_DT={cfg.PHYSICS_DT}"

    sensors = SensorGenerator(model, data)

    # ── Initialize subsystems ──
    ap_json = None
    mav = None
    hover_ctrl = None

    if args.stage >= 2:
        ap_json = ArduPilotJSONBackend()
        mav = MAVLinkManager(args.stage, serial_port=args.serial)
        print(f"[Bridge] ArduPilot JSON backend on UDP :{cfg.AP_JSON_BIND_PORT}")
        print(f"[Bridge] ArduPilot MAVLink on UDP :{cfg.AP_MAVLINK_PORT}")
        print(f"[Bridge] FC MAVLink out → :{cfg.FC_LISTEN_PORT}  in ← :{cfg.BRIDGE_FC_LISTEN_PORT}")
    else:
        hover_ctrl = SimpleHoverController()
        print("[Bridge] Stage 1 — using built-in hover controller")

    # ── Timing state ──
    tick = 0
    sim_time = 0.0
    last_render_t = 0.0
    last_heartbeat_t = 0.0
    perf_log_interval = 5.0
    last_perf_t = 0.0
    overrun_count = 0
    total_ticks = 0

    # ── Launch viewer (runs in background thread) ──
    print("[Bridge] Launching MuJoCo viewer …")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("[Bridge] Viewer running. Entering physics loop.")

        try:
            while viewer.is_running():
                wall_start = time.perf_counter()

                # ────────────── 1. Apply motor commands ──────────────
                if args.stage == 1:
                    motors = hover_ctrl.update(model, data)
                    data.ctrl[:4] = motors
                elif args.stage >= 2 and mav is not None:
                    data.ctrl[:4] = mav.motor_outputs

                # ────────────── 2. Step physics ──────────────────────
                mujoco.mj_step(model, data)
                sim_time += cfg.PHYSICS_DT
                tick += 1
                total_ticks += 1
                time_boot_ms = int(sim_time * 1000)

                # ────────────── 3. Generate sensors ──────────────────
                gyro_ned, accel_ned = sensors.get_imu_ned()

                # ────────────── 4. ArduPilot JSON exchange ───────────
                if ap_json is not None:
                    got_servo = ap_json.poll_servo()
                    if got_servo or ap_json.connected:
                        state = sensors.get_ardupilot_json_state(sim_time)
                        ap_json.send_state(state)

                # ────────────── 5. MAVLink I/O ───────────────────────
                if mav is not None:
                    # Receive from ArduPilot
                    mav.poll_ardupilot()

                    # Receive from FC
                    mav.poll_fc()

                    # Forward setpoints to FC (every tick for low latency)
                    if mav.attitude_target is not None:
                        at = mav.attitude_target
                        mav.send_setpoint_to_fc(
                            at.body_roll_rate,
                            at.body_pitch_rate,
                            at.body_yaw_rate,
                            at.thrust,
                            time_boot_ms,
                        )

                    # Send IMU to FC (every tick — 1 kHz)
                    if tick % cfg.IMU_EVERY_N_TICKS == 0:
                        mav.send_imu_to_fc(gyro_ned, accel_ned, time_boot_ms)

                    # Send GPS to ArduPilot via JSON (already in state);
                    # also send LiDAR via MAVLink
                    if tick % cfg.LIDAR_EVERY_N_TICKS == 0:
                        alt = sensors.get_lidar_altitude()
                        mav.send_lidar_to_ap(alt, time_boot_ms)

                    # Heartbeat at 1 Hz
                    if wall_start - last_heartbeat_t >= 1.0:
                        mav.send_heartbeat()
                        last_heartbeat_t = wall_start

                # ────────────── 6. Sync viewer at ~60 Hz ─────────────
                wall_now = time.perf_counter()
                if wall_now - last_render_t >= cfg.RENDER_INTERVAL_S:
                    viewer.sync()
                    last_render_t = wall_now

                # ────────────── 7. Wall-clock pacing (1 kHz) ─────────
                wall_elapsed = time.perf_counter() - wall_start
                sleep_s = cfg.PHYSICS_DT - wall_elapsed
                if sleep_s > 0:
                    # Busy-wait for last ~0.2 ms for precision
                    target = wall_start + cfg.PHYSICS_DT
                    if sleep_s > 0.0003:
                        time.sleep(sleep_s - 0.0002)
                    while time.perf_counter() < target:
                        pass
                else:
                    overrun_count += 1

                # ────────────── 8. Performance logging ────────────────
                if wall_now - last_perf_t >= perf_log_interval:
                    if total_ticks > 0:
                        rate = total_ticks / (wall_now - (last_perf_t if last_perf_t > 0 else wall_start - perf_log_interval))
                        ap_str = "connected" if (ap_json and ap_json.connected) else "waiting"
                        fc_str = "ok" if (mav and mav.fc_heartbeat_ok) else "waiting"
                        pos = data.qpos[0:3]
                        print(f"[Bridge] tick={tick:>8d}  rate={rate:7.1f} Hz  "
                              f"overruns={overrun_count}  "
                              f"pos=({pos[0]:+6.2f},{pos[1]:+6.2f},{pos[2]:+6.2f})  "
                              f"AP={ap_str}  FC={fc_str}")
                    overrun_count = 0
                    total_ticks = 0
                    last_perf_t = wall_now

        except KeyboardInterrupt:
            print("\n[Bridge] Shutting down …")
        finally:
            if ap_json:
                ap_json.close()
            print("[Bridge] Done.")


if __name__ == "__main__":
    main()
