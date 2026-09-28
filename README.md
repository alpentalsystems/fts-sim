# Simulated drone flight termination system

A flight termination system (FTS) for a small drone that watches the flight
independently of the autopilot and, on a fault, cuts motor power and deploys
a parachute. Everything runs in simulation: PX4 SITL and Gazebo for the
drone, the FTS as Zephyr firmware on native_sim, ROS 2 for the ground
station and tests.

Design: [docs/2026-09-28-fts-sim-design.md](docs/2026-09-28-fts-sim-design.md)

## Host tests (Mac or VM)

```sh
cmake -S tests/host -B build/host && cmake --build build/host && ctest --test-dir build/host
cmake -S bridge -B build/bridge-host -DFTS_BRIDGE_GZ=OFF && cmake --build build/bridge-host && ctest --test-dir build/bridge-host
python3 -m pytest ros2_ws/src/fts_ground/test
```

## Simulation (VM with PX4, Gazebo 8, ROS 2 Humble)

```sh
tools/sync-vm.sh                       # Mac: copy the tree to ~/fts-sim on the VM
tools/setup-vm.sh                      # VM, once: Zephyr workspace
tools/build-vm.sh                      # VM: firmware, plugin, bridge, ROS 2 packages
tools/run-scenarios.sh fence manual    # VM: one fresh simulation per scenario
```

Results: [docs/test-log.md](docs/test-log.md).

## License

Apache-2.0; see [LICENSE](LICENSE).
