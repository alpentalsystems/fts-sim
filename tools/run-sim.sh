#!/usr/bin/env bash
# Starts Gazebo, the FTS firmware, the bridge, PX4 and the ground station (VM side).
# Usage: tools/run-sim.sh [--gui]
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
PX4=${PX4_DIR:-$HOME/PX4-Autopilot}
RUN=$REPO/run
GUI=0
if [ "${1:-}" = "--gui" ]; then
	GUI=1
fi

if [ -f "$RUN/pids" ]; then
	echo "simulation already running (see $RUN/pids); run tools/stop-sim.sh first" >&2
	exit 1
fi
mkdir -p "$RUN"
rm -f "$RUN"/*.log "$RUN/truth.csv" "$RUN/ptys"

export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH=$REPO/sim/models:$REPO/sim/worlds:$PX4/Tools/simulation/gz/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$REPO/build/sim

start() {
	local name=$1
	shift
	"$@" > "$RUN/$name.log" 2>&1 &
	echo "$name $!" >> "$RUN/pids"
}

wait_for() {
	local what=$1
	local check=$2
	for _ in $(seq 1 90); do
		if eval "$check" > /dev/null 2>&1; then
			return 0
		fi
		sleep 1
	done
	echo "timeout waiting for $what" >&2
	exit 1
}

start gz gz sim -r -s "$REPO/sim/worlds/fts_field.sdf"
if [ "$GUI" = 1 ]; then
	start gzgui gz sim -g
fi
wait_for "Gazebo" "gz topic -l | grep -q '^/world/fts_field/clock'"

start fts stdbuf -oL -eL "$REPO/build/fts/zephyr/zephyr.exe"
wait_for "FTS PTYs" "grep -q 'uart_1 connected to pseudotty' '$RUN/fts.log'"
PTY1=$(sed -n 's/^uart connected to pseudotty: \(.*\)$/\1/p' "$RUN/fts.log")
PTY2=$(sed -n 's/^uart_1 connected to pseudotty: \(.*\)$/\1/p' "$RUN/fts.log")
echo "$PTY1 $PTY2" > "$RUN/ptys"

start bridge "$REPO/build/bridge/fts_bridge" --link "$PTY1" --truth "$RUN/truth.csv"
wait_for "bridge" "grep -q 'fts_bridge: running' '$RUN/bridge.log'"

# With no arguments PX4 uses build/px4_sitl_default/etc and .../rootfs.
(export PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=x500_fts_0 PX4_GZ_WORLD=fts_field &&
	start px4 "$PX4/build/px4_sitl_default/bin/px4" -d)
wait_for "PX4 ready" "grep -q 'Ready for takeoff' '$RUN/px4.log'"
wait_for "PX4 heartbeat rate" \
	"cd '$PX4/build/px4_sitl_default' && ./bin/px4-mavlink stream -u 14580 -s HEARTBEAT -r 10"

set +u
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
set -u
start gs "$REPO/ros2_ws/install/fts_ground/lib/fts_ground/ground_station" --ros-args -p link:="$PTY2"
wait_for "ground station" "grep -q 'link 2 on' '$RUN/gs.log'"
echo "simulation running; logs in $RUN"
