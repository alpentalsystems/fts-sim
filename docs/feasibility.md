# PX4 SITL feasibility on the VM

Date: 2026-09-28. Result: **go**. PX4 SITL flies the x500 in Gazebo on the
VM at real time, with low CPU use.

## Environment

- UTM VM, Ubuntu 22.04 aarch64, 8 cores, 7.7 GB RAM.
- Gazebo `gz sim` 8.15.0 (Harmonic), ROS 2 Humble.
- PX4 v1.15.4, shallow clone with submodules (1.2 GB), SITL build directory
  379 MB. Free disk after everything: 11 GB.

## Setup notes

- `Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools` stops at the Python step:
  `matplotlib>=3.0.*` in `Tools/setup/requirements.txt` is rejected by the
  newer pip. Install from a copy with the `.*` removed:
  `sed 's/>=\([0-9.]*\)\.\*/>=\1/' Tools/setup/requirements.txt > /tmp/px4-req.txt && python3 -m pip install --user -r /tmp/px4-req.txt`.
- The build fails in `px_update_git_header.py` because a shallow clone has no
  NuttX tags. The SITL build does not use NuttX; a local placeholder tag
  fixes it: `git -C platforms/nuttx/NuttX/nuttx tag nuttx-0.0.0`.
- uXRCE-DDS agent: `sudo snap install micro-xrce-dds-agent --edge`
  (v1.0.2), run as `micro-xrce-dds-agent udp4 -p 8888`.

## Run

- Start: `HEADLESS=1 make px4_sitl gz_x500` (keep stdin open, for example
  `tail -f /dev/null | ...`, when run in the background). Commands from
  another shell: `build/px4_sitl_default/bin/px4-commander takeoff` or `land`.
- Real-time factor: 1.00. CPU: PX4 about 18%, Gazebo server about 18%.
- `commander takeoff` climbs to about 2.1 m and holds; `commander land`
  lands (`vehicle_land_detected.landed: True`).
- Two "Yaw estimate error" preflight warnings right after startup, then
  "Ready for takeoff".

## Facts for plan 2

| Item | Value |
|---|---|
| World name | `default` (`Tools/simulation/gz/worlds/default.sdf`) |
| Model name | `x500_0` (model `x500`) |
| Motor command topic | `/x500_0/command/motor_speed`, type `gz.msgs.Actuators` (also `/model/x500_0/command/motor_speed`) |
| Existing sensors | `/world/default/model/x500_0/link/base_link/sensor/{imu_sensor/imu, navsat_sensor/navsat, air_pressure_sensor/air_pressure}` |
| Clock | `/world/default/clock`, `/clock` |
| MAVLink | UDP to 127.0.0.1:14550 (GCS link) and 127.0.0.1:14540 (onboard link, from port 14580) |
| Heartbeat rate | 1 Hz by default; `px4-mavlink stream -u 14580 -s HEARTBEAT -r 10` gives 10 Hz on 14540 (measured) |
| ROS 2 | 45 `/fmu/...` topics after the agent starts (for example `/fmu/out/sensor_combined`, `/fmu/out/failsafe_flags`) |

## Consequences for plan 2

- The bridge reads heartbeats on UDP 14540 after setting the rate to 10 Hz at
  startup, as the spec assumes. The PX4 sensors above are PX4's own; the FTS
  gets its own IMU, GNSS, and barometer added to the model copy.
- Motor gating: the model copy renames the motor command topic that the
  motor plugins subscribe to, and the bridge forwards
  `/x500_0/command/motor_speed` to it, or zero speeds after termination.
