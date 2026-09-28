"""Plots one scenario run: altitude and vertical speed over time, and the ground track.

Usage: tools/plot_run.py RUN_DIR (needs truth.csv and result.json)
"""
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/fts_ground"))
from fts_ground import metrics  # noqa: E402

EVENTS = (("t_fault_us", "fault", "gray"), ("t_trigger_us", "trigger", "red"),
          ("t_chute_us", "parachute", "green"))


def main() -> None:
    out = Path(sys.argv[1])
    rows = metrics.read_truth(out / "truth.csv")
    result = json.loads((out / "result.json").read_text())
    t0 = rows[0].t_us
    z0 = rows[0].z
    ts = [(r.t_us - t0) / 1e6 for r in rows]

    fig, (alt, speed) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    alt.plot(ts, [r.z - z0 for r in rows])
    alt.set_ylabel("Altitude (m)")
    speed.plot(ts, [r.vz for r in rows])
    speed.set_ylabel("Vertical speed (m/s)")
    speed.set_xlabel("Simulation time (s)")
    for key, label, color in EVENTS:
        t = result.get(key)
        if t is not None and t >= 0:
            for ax in (alt, speed):
                ax.axvline((t - t0) / 1e6, color=color, linestyle="--", label=label)
    alt.legend()
    alt.set_title(f"{result['scenario']}: cause {result['cause']}")
    fig.tight_layout()
    fig.savefig(out / "altitude.png", dpi=120)

    fig, ax = plt.subplots(figsize=(6, 6))
    h = metrics.FENCE_HALF_M
    ax.plot([-h, h, h, -h, -h], [-h, -h, h, h, -h], "r-", label="fence")
    ax.plot([r.x for r in rows], [r.y for r in rows], label="track")
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "track.png", dpi=120)


if __name__ == "__main__":
    main()
