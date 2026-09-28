#!/usr/bin/env bash
# Runs scenarios one by one, each in a fresh simulation (VM side).
# Usage: tools/run-scenarios.sh [--gui] SCENARIO...
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
GUI=()
if [ "${1:-}" = "--gui" ]; then
	GUI=(--gui)
	shift
fi
if [ $# -eq 0 ]; then
	echo "usage: $0 [--gui] SCENARIO..." >&2
	exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
set -u

failed=()
for s in "$@"; do
	out=$REPO/runs/$s
	rm -rf "$out"
	mkdir -p "$out"
	"$REPO/tools/run-sim.sh" "${GUI[@]}"
	ros2 bag record -o "$out/bag" /fts/status > "$out/bag.log" 2>&1 &
	bag=$!
	status=0
	"$REPO/ros2_ws/install/fts_ground/lib/fts_ground/runner" "$s" --out "$out" --repo "$REPO" \
		2>&1 | tee "$out/runner.log" || status=$?
	kill -INT "$bag"
	wait "$bag" || echo "rosbag exited with status $?" >&2
	"$REPO/tools/stop-sim.sh"
	cp "$REPO"/run/*.log "$REPO/run/truth.csv" "$out/"
	if [ "$status" -eq 0 ]; then
		# -s: system numpy and matplotlib, not the user-installed numpy 2
		python3 -s "$REPO/tools/plot_run.py" "$out"
		echo "scenario $s passed"
	else
		failed+=("$s")
		echo "scenario $s FAILED (status $status)" >&2
	fi
done
if [ ${#failed[@]} -gt 0 ]; then
	echo "failed: ${failed[*]}" >&2
	exit 1
fi
