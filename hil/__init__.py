"""
HIL Multirotor Project
======================
Hardware-in-the-Loop simulation with MuJoCo, ArduPilot SITL,
and a companion/flight-controller split architecture.

Modules
-------
config      Shared constants (MAVLink IDs, ports, rates, vehicle params)
sensors     Sensor generation from MuJoCo state
bridge      MuJoCo HIL bridge (main entry point)
dummy_fc    Python dummy flight controller (Stage 2)
"""
