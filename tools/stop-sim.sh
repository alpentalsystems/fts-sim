#!/usr/bin/env bash
# Stops everything tools/run-sim.sh started (VM side).
set -uo pipefail
RUN=$(cd "$(dirname "$0")/.." && pwd)/run
if [ ! -f "$RUN/pids" ]; then
	echo "no $RUN/pids: nothing to stop" >&2
	exit 1
fi
while read -r name pid; do
	kill -CONT "$pid" 2> /dev/null
	if kill "$pid" 2> /dev/null; then
		echo "stopped $name ($pid)"
	fi
done < "$RUN/pids"
sleep 3
while read -r name pid; do
	if kill -0 "$pid" 2> /dev/null; then
		kill -9 "$pid"
		echo "killed $name ($pid)"
	fi
done < "$RUN/pids"
rm -f "$RUN/pids"
