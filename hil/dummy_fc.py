#!/usr/bin/env python3
"""
Python Dummy Flight Controller  (Stage 2)
==========================================
Emulates the future STM32F407 inner-loop controller using the EXACT SAME
MAVLink message contract.  Replacing this with STM32 firmware requires
only a transport swap (UDP → UART serial) — same sysid/compid, same messages.

Responsibilities
----------------
1. Receive SET_ATTITUDE_TARGET (body-rate setpoints p,q,r + thrust) from Bridge.
2. Receive SCALED_IMU (gyro feedback) from Bridge.
3. Run 1 kHz rate PID (roll, pitch, yaw) using gyro as feedback.
4. Mix PID outputs through Quad-X mixer → 4 motor outputs [0..1].
5. Publish motor outputs to Bridge as SERVO_OUTPUT_RAW.
6. Publish HEARTBEAT at 1 Hz.
7. Enforce failsafe: if setpoints stale > 50 ms → motors to 0.

Message Contract  (same for STM32)
-----------------------------------
  RECEIVE:
    SET_ATTITUDE_TARGET (msg 82)  — body_roll_rate, body_pitch_rate,
                                    body_yaw_rate (rad/s), thrust (0..1)
    SCALED_IMU          (msg 26)  — xgyro, ygyro, zgyro (mrad/s)

  TRANSMIT:
    SERVO_OUTPUT_RAW    (msg 36)  — servo1..servo4 = PWM 1000..2000
    HEARTBEAT           (msg  0)  — MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC

Usage
-----
    python dummy_fc.py           # defaults: UDP, 1 kHz loop
    python dummy_fc.py --rate 500   # 500 Hz loop for slower machines
"""

import argparse
import math
import time

import numpy as np
from pymavlink import mavutil

import config as cfg


# ═══════════════════════════════════════════════════════════════════════════════
#  PID Controller
# ═══════════════════════════════════════════════════════════════════════════════
class PID:
    """Discrete PID with anti-windup clamp and derivative-on-measurement."""

    __slots__ = ("kp", "ki", "kd", "imax", "integral", "prev_meas", "output")

    def __init__(self, kp: float, ki: float, kd: float, imax: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.imax = imax
        self.integral = 0.0
        self.prev_meas = 0.0
        self.output = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_meas = 0.0
        self.output = 0.0

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.imax / max(self.ki, 1e-9),
                                self.imax / max(self.ki, 1e-9))
        i_term = self.ki * self.integral

        # Derivative on measurement (avoids setpoint kick)
        d_meas = (measurement - self.prev_meas) / dt if dt > 0 else 0.0
        self.prev_meas = measurement
        d_term = -self.kd * d_meas

        self.output = p_term + i_term + d_term
        return self.output


# ═══════════════════════════════════════════════════════════════════════════════
#  Quad-X Mixer
# ═══════════════════════════════════════════════════════════════════════════════
def quad_x_mix(throttle: float, roll_out: float, pitch_out: float,
               yaw_out: float) -> np.ndarray:
    """
    Apply Quad-X mixer and return motor commands [m1..m4] clamped to [0, 1].
    Uses the same MIXER matrix defined in config.py.
    """
    cmd = np.array([throttle, roll_out, pitch_out, yaw_out])
    motors = cfg.MIXER @ cmd
    return np.clip(motors, 0.0, 1.0)


# ═══════════════════════════════════════════════════════════════════════════════
#  Dummy Flight Controller
# ═══════════════════════════════════════════════════════════════════════════════
class DummyFC:

    def __init__(self, loop_rate_hz: int = 1000):
        self.loop_rate = loop_rate_hz
        self.dt = 1.0 / loop_rate_hz

        # MAVLink connections (Bridge → FC, FC → Bridge)
        # FC listens for setpoints + IMU from Bridge
        self.mav_in = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{cfg.FC_LISTEN_PORT}",
            source_system=cfg.FC_SYSID,
            source_component=cfg.FC_COMPID,
            dialect="common",
        )
        # FC sends motor outputs + heartbeat to Bridge
        self.mav_out = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{cfg.BRIDGE_FC_LISTEN_PORT}",
            source_system=cfg.FC_SYSID,
            source_component=cfg.FC_COMPID,
            dialect="common",
        )

        # PID controllers
        self.pid_roll = PID(**cfg.RATE_PID["roll"])
        self.pid_pitch = PID(**cfg.RATE_PID["pitch"])
        self.pid_yaw = PID(**cfg.RATE_PID["yaw"])

        # State
        self.setpoint_p = 0.0    # roll rate  rad/s
        self.setpoint_q = 0.0    # pitch rate
        self.setpoint_r = 0.0    # yaw rate
        self.thrust = 0.0        # 0..1

        self.gyro_p = 0.0        # measured roll rate  rad/s
        self.gyro_q = 0.0
        self.gyro_r = 0.0

        self.motors = np.zeros(4)

        self.last_setpoint_time = 0.0    # wall-clock of last setpoint
        self.failsafe_active = False

        self.running = True
        self.tick_count = 0

    def poll_mavlink(self):
        """Non-blocking read of all pending MAVLink messages."""
        while True:
            msg = self.mav_in.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()

            if mtype == "SET_ATTITUDE_TARGET":
                # Body-rate setpoints + thrust from ArduPilot via Bridge
                self.setpoint_p = msg.body_roll_rate
                self.setpoint_q = msg.body_pitch_rate
                self.setpoint_r = msg.body_yaw_rate
                self.thrust = np.clip(msg.thrust, 0.0, 1.0)
                self.last_setpoint_time = time.perf_counter()
                if self.failsafe_active:
                    print("[FC] Setpoints resumed — exiting failsafe")
                    self.failsafe_active = False

            elif mtype == "SCALED_IMU":
                # Gyro from MuJoCo via Bridge  (mrad/s → rad/s)
                self.gyro_p = msg.xgyro / 1000.0
                self.gyro_q = msg.ygyro / 1000.0
                self.gyro_r = msg.zgyro / 1000.0

    def check_failsafe(self):
        """Trigger failsafe if setpoints are stale."""
        now = time.perf_counter()
        if self.last_setpoint_time == 0.0:
            # Never received setpoints — stay idle
            if not self.failsafe_active:
                self.failsafe_active = True
            return

        age_ms = (now - self.last_setpoint_time) * 1000.0
        if age_ms > cfg.SETPOINT_TIMEOUT_MS:
            if not self.failsafe_active:
                print(f"[FC] FAILSAFE — setpoints stale ({age_ms:.0f} ms)")
                self.failsafe_active = True
                self.pid_roll.reset()
                self.pid_pitch.reset()
                self.pid_yaw.reset()

    def control_step(self):
        """One iteration of rate PID + mixer."""
        if self.failsafe_active:
            self.motors[:] = cfg.FAILSAFE_MOTOR_OUTPUT
            return

        roll_out = self.pid_roll.update(self.setpoint_p, self.gyro_p, self.dt)
        pitch_out = self.pid_pitch.update(self.setpoint_q, self.gyro_q, self.dt)
        yaw_out = self.pid_yaw.update(self.setpoint_r, self.gyro_r, self.dt)

        self.motors = quad_x_mix(self.thrust, roll_out, pitch_out, yaw_out)

    def send_motor_output(self):
        """Publish motor outputs to Bridge as SERVO_OUTPUT_RAW (msg 36)."""
        time_usec = int(time.perf_counter() * 1e6) & 0xFFFFFFFF
        # Motor command 0..1 → PWM 1000..2000
        pwm = (self.motors * 1000.0 + 1000.0).astype(int)
        self.mav_out.mav.servo_output_raw_send(
            time_usec,
            0,              # port
            pwm[0],         # servo1 (M1)
            pwm[1],         # servo2 (M2)
            pwm[2],         # servo3 (M3)
            pwm[3],         # servo4 (M4)
            0, 0, 0, 0,    # servo5-8 unused
        )

    def send_heartbeat(self):
        """FC heartbeat — same message the STM32 will send."""
        self.mav_out.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0,  # custom_mode
            mavutil.mavlink.MAV_STATE_ACTIVE if not self.failsafe_active
            else mavutil.mavlink.MAV_STATE_CRITICAL,
        )

    def run(self):
        """Main control loop."""
        print(f"[FC] Dummy Flight Controller starting — {self.loop_rate} Hz")
        print(f"[FC] sysid={cfg.FC_SYSID}  compid={cfg.FC_COMPID}")
        print(f"[FC] Listening on UDP :{cfg.FC_LISTEN_PORT}")
        print(f"[FC] Sending to Bridge UDP :{cfg.BRIDGE_FC_LISTEN_PORT}")
        print(f"[FC] Failsafe timeout = {cfg.SETPOINT_TIMEOUT_MS} ms")

        last_heartbeat = 0.0
        last_log = 0.0

        try:
            while self.running:
                t_start = time.perf_counter()

                # 1. Read incoming MAVLink
                self.poll_mavlink()

                # 2. Failsafe check
                self.check_failsafe()

                # 3. Rate PID + mixer
                self.control_step()

                # 4. Publish motor outputs (every tick for tight loop)
                self.send_motor_output()

                # 5. Heartbeat at 1 Hz
                now = time.perf_counter()
                if now - last_heartbeat >= 1.0:
                    self.send_heartbeat()
                    last_heartbeat = now

                # 6. Periodic status log (every 5 s)
                if now - last_log >= 5.0:
                    sp = f"sp=({self.setpoint_p:+.3f},{self.setpoint_q:+.3f},{self.setpoint_r:+.3f})"
                    gy = f"gyro=({self.gyro_p:+.3f},{self.gyro_q:+.3f},{self.gyro_r:+.3f})"
                    mo = f"motors=[{self.motors[0]:.3f},{self.motors[1]:.3f},{self.motors[2]:.3f},{self.motors[3]:.3f}]"
                    fs = "FAILSAFE" if self.failsafe_active else "OK"
                    print(f"[FC] tick={self.tick_count:>8d}  {sp}  {gy}  {mo}  {fs}")
                    last_log = now

                self.tick_count += 1

                # 7. Timing: maintain loop rate
                elapsed = time.perf_counter() - t_start
                sleep_s = self.dt - elapsed
                if sleep_s > 0:
                    target = t_start + self.dt
                    if sleep_s > 0.0003:
                        time.sleep(sleep_s - 0.0002)
                    while time.perf_counter() < target:
                        pass

        except KeyboardInterrupt:
            print("\n[FC] Shutting down …")
        finally:
            # Motors off
            self.motors[:] = 0.0
            self.send_motor_output()
            print("[FC] Done.")


# ═══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="Dummy Flight Controller (Stage 2)")
    parser.add_argument("--rate", type=int, default=1000,
                        help="Control loop rate in Hz (default: 1000)")
    args = parser.parse_args()

    fc = DummyFC(loop_rate_hz=args.rate)
    fc.run()


if __name__ == "__main__":
    main()
