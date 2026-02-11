#!/usr/bin/env python3
"""
Interactive Mission Script
===========================
Connects to ArduPilot SITL, arms, takes off, flies user-entered waypoints,
and lands.  Ctrl+C at any time triggers immediate LAND.

Usage:
    python fly_mission.py

Requires ArduPilot, bridge.py, and dummy_fc.py to be running first.
See README.md for startup order.
"""

import math
import sys
import time

from pymavlink import mavutil


# ─── Constants ───────────────────────────────────────────────────────────────

CONN_STRING = "udpin:0.0.0.0:14551"
CONNECT_TIMEOUT = 15          # seconds to wait for heartbeat
WP_ACCEPT_RADIUS = 1.0       # metres — close enough to waypoint
WP_TIMEOUT = 30               # seconds per waypoint before giving up


# ─── Connection ──────────────────────────────────────────────────────────────

def connect():
    print(f"Connecting to ArduPilot on {CONN_STRING} ...")
    mav = mavutil.mavlink_connection(CONN_STRING)
    hb = mav.wait_heartbeat(timeout=CONNECT_TIMEOUT)
    if hb is None:
        print(f"\nERROR: No heartbeat received within {CONNECT_TIMEOUT}s.")
        print("Check that:")
        print("  1. ArduPilot SITL is running")
        print("  2. You launched sim_vehicle.py with:")
        print("       --out=127.0.0.1:14550 --out=127.0.0.1:14551")
        print("     (required on WSL2 — without these flags, MAVLink")
        print("      goes to the Windows host instead of localhost)")
        sys.exit(1)
    print(f"Connected!  sysid={mav.target_system}  "
          f"comp={mav.target_component}")
    return mav


# ─── MAVLink helpers ─────────────────────────────────────────────────────────

def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"Unknown mode: {mode_name}")
        return False
    mav.set_mode(mode_id)
    while True:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb is None:
            print(f"  Timeout waiting for mode {mode_name}")
            return False
        if hb.custom_mode == mode_id:
            return True


def arm(mav):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    mav.motors_armed_wait()


def takeoff(mav, alt_m):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt_m,
    )


def goto_ned(mav, north, east, down):
    type_mask = 0b0000_1111_1111_1000   # position only
    mav.mav.set_position_target_local_ned_send(
        0,
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        north, east, down,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )


def wait_altitude(mav, target_alt, timeout=15):
    """Wait until altitude (negative D in NED) is within 1 m of target."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                             timeout=1)
        if pos:
            alt = -pos.z
            print(f"  alt: {alt:.1f} m / {target_alt:.1f} m", end="\r")
            if abs(alt - target_alt) < 1.0:
                print()
                return True
    print()
    return False


def wait_waypoint(mav, n, e, d, timeout=WP_TIMEOUT):
    """Wait until position is within WP_ACCEPT_RADIUS of target."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                             timeout=1)
        if pos:
            dn = pos.x - n
            de = pos.y - e
            dd = pos.z - d
            dist = math.sqrt(dn**2 + de**2 + dd**2)
            print(f"  pos: N={pos.x:+.1f} E={pos.y:+.1f} "
                  f"D={pos.z:+.1f}  dist={dist:.1f}m", end="\r")
            if dist < WP_ACCEPT_RADIUS:
                print()
                return True
    print()
    return False


def emergency_land(mav):
    """Switch to LAND mode — used on Ctrl+C."""
    print("\nEmergency LAND triggered!")
    try:
        set_mode(mav, "LAND")
    except Exception:
        pass


# ─── Interactive prompts ─────────────────────────────────────────────────────

def prompt_altitude():
    while True:
        raw = input("Enter takeoff altitude in metres [5]: ").strip()
        if raw == "":
            return 5.0
        try:
            alt = float(raw)
            if alt <= 0:
                print("  Must be positive.")
                continue
            return alt
        except ValueError:
            print("  Enter a number.")


def prompt_waypoints():
    print("\nEnter waypoints as N,E (metres from home). "
          "Type 'done' to finish.")
    wps = []
    while True:
        raw = input("  wp> ").strip()
        if raw.lower() == "done":
            break
        parts = raw.replace(" ", "").split(",")
        if len(parts) != 2:
            print("  Format: N,E  (e.g. 3,0 or -2,4)")
            continue
        try:
            n, e = float(parts[0]), float(parts[1])
            wps.append((n, e))
        except ValueError:
            print("  Enter two numbers separated by a comma.")
    return wps


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    mav = connect()

    try:
        # GUIDED mode
        print("Setting GUIDED mode... ", end="", flush=True)
        if not set_mode(mav, "GUIDED"):
            print("FAILED")
            sys.exit(1)
        print("OK")

        # Arm
        print("Arming... ", end="", flush=True)
        arm(mav)
        print("OK")

        # Get altitude and waypoints before flying
        alt = prompt_altitude()
        wps = prompt_waypoints()
        if not wps:
            print("No waypoints entered — will just takeoff, hover, and land.")

        # Takeoff
        print(f"\nTaking off to {alt:.0f}m...")
        takeoff(mav, alt)
        if not wait_altitude(mav, alt, timeout=20):
            print("WARNING: altitude target not reached, continuing anyway")

        # Fly waypoints
        if wps:
            print(f"\nFlying {len(wps)} waypoint(s) at {alt:.0f}m altitude:\n")
            for i, (n, e) in enumerate(wps, 1):
                d = -alt
                print(f"  [{i}/{len(wps)}] -> N={n:.1f}, E={e:.1f}, D={d:.1f}")
                goto_ned(mav, n, e, d)
                if wait_waypoint(mav, n, e, d):
                    print(f"  [{i}/{len(wps)}] reached")
                else:
                    print(f"  [{i}/{len(wps)}] TIMEOUT — moving to next")
        else:
            print("Hovering for 5 seconds...")
            time.sleep(5)

        # Land
        print("\nLanding...")
        set_mode(mav, "LAND")
        mav.motors_disarmed_wait()
        print("Landed and disarmed. Mission complete!")

    except KeyboardInterrupt:
        emergency_land(mav)
        print("Waiting for disarm...")
        try:
            mav.motors_disarmed_wait()
            print("Landed safely.")
        except KeyboardInterrupt:
            print("Force quit.")


if __name__ == "__main__":
    main()
