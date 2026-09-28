#!/usr/bin/env bash
# Builds everything on the VM.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
export ZEPHYR_BASE=${ZEPHYR_BASE:-$HOME/zephyrproject/zephyr}
export ZEPHYR_TOOLCHAIN_VARIANT=host
export PATH=$HOME/.local/bin:$PATH

# FTS firmware, with the Python 3.12 venv from setup-vm.sh
(
	PATH=$HOME/zephyrproject/.venv/bin:$PATH
	cmake -GNinja -S "$REPO/fts" -B "$REPO/build/fts" -DBOARD=native_sim/native/64 \
		-DPython3_EXECUTABLE="$HOME/zephyrproject/.venv/bin/python"
	ninja -C "$REPO/build/fts"
)

# Gazebo plugins
cmake -GNinja -S "$REPO/sim/plugins" -B "$REPO/build/sim"
ninja -C "$REPO/build/sim"

# I/O bridge
cmake -GNinja -S "$REPO/bridge" -B "$REPO/build/bridge"
ninja -C "$REPO/build/bridge"
ctest --test-dir "$REPO/build/bridge" --output-on-failure

# ROS 2 packages
set +u
source /opt/ros/humble/setup.bash
set -u
(cd "$REPO/ros2_ws" && colcon build --symlink-install)
