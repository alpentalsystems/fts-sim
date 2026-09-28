# Simulated drone flight termination system

A flight termination system (FTS) for a small drone that watches the flight
independently of the autopilot and, on a fault, cuts motor power and deploys
a parachute. Everything runs in simulation: PX4 SITL and Gazebo for the
drone, the FTS as Zephyr firmware on native_sim, ROS 2 for the ground
station and tests.

Design: [docs/2026-09-28-fts-sim-design.md](docs/2026-09-28-fts-sim-design.md)

## Host tests

```sh
cmake -S tests/host -B build/host && cmake --build build/host && ctest --test-dir build/host
```
