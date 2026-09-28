"""Prints a markdown table of runs/*/result.json.

Usage: tools/summarize_runs.py [RUNS_DIR]
"""
import json
import sys
from pathlib import Path

COLUMNS = (
    ("scenario", "Scenario"),
    ("pass", "Pass"),
    ("cause", "Cause"),
    ("fault_to_trigger_ms", "Fault to trigger (ms)"),
    ("fault_to_relay_ms", "Fault to relay (ms)"),
    ("fault_to_chute_ms", "Fault to parachute (ms)"),
    ("trigger_altitude_m", "Altitude at trigger (m)"),
    ("vz_under_chute_m_s", "Descent under parachute (m/s)"),
    ("drift_m", "Drift (m)"),
    ("beyond_fence_m", "Beyond fence (m)"),
    ("max_distance_from_center_m", "Max distance from center (m)"),
)


def main() -> None:
    runs = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "runs"
    results = [json.loads(p.read_text()) for p in sorted(runs.glob("*/result.json"))]
    if not results:
        raise SystemExit(f"no results in {runs}")
    print("| " + " | ".join(h for _, h in COLUMNS) + " |")
    print("|" + "---|" * len(COLUMNS))
    for r in results:
        print("| " + " | ".join("" if r.get(k) is None else str(r[k]) for k, _ in COLUMNS) + " |")


if __name__ == "__main__":
    main()
