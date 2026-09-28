"""Checks the bridge against a running fts_field world (VM, real-time factor 1).

Runs the bridge on a PTY pair for 5 s and checks the frame rates it sends
and that it writes ground truth.
"""
import os
import subprocess
import sys
import time
import tty
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "ros2_ws/src/fts_ground"))
from fts_ground import proto  # noqa: E402

SECONDS = 5.0
RANGES = {"imu": (80, 120), "gnss": (8, 12), "baro": (16, 24), "readback": (8, 12)}
TYPES = {"imu": proto.MSG_IMU, "gnss": proto.MSG_GNSS, "baro": proto.MSG_BARO,
         "readback": proto.MSG_READBACK}


def main() -> None:
    master, slave = os.openpty()
    tty.setraw(master)
    truth = REPO / "build/bridge_check_truth.csv"
    bridge = subprocess.Popen([str(REPO / "build/bridge/fts_bridge"), "--link", os.ttyname(slave),
                               "--truth", str(truth)])
    try:
        time.sleep(1.0)
        os.set_blocking(master, False)
        # Drop what queued up during startup so the count covers SECONDS only.
        try:
            while os.read(master, 4096):
                pass
        except BlockingIOError:
            pass
        parser = proto.Parser()
        counts = {}
        end = time.time() + SECONDS
        while time.time() < end:
            try:
                data = os.read(master, 4096)
            except BlockingIOError:
                time.sleep(0.01)
                continue
            for msg_type, _ in parser.feed(data):
                counts[msg_type] = counts.get(msg_type, 0) + 1
        rates = {name: counts.get(t, 0) / SECONDS for name, t in TYPES.items()}
        print("rates (Hz, wall clock):", rates)
        ok = all(lo <= rates[name] <= hi for name, (lo, hi) in RANGES.items())
        rows = truth.read_text().splitlines()
        print("truth rows:", len(rows) - 1, "last:", rows[-1])
        ok = ok and len(rows) > 100
        print("bridge check", "passed" if ok else "FAILED")
        sys.exit(0 if ok else 1)
    finally:
        bridge.terminate()
        bridge.wait(5)


if __name__ == "__main__":
    main()
