"""Firmware smoke test on the VM: drive the native_sim FTS through its PTYs.

Feeds 1.2 s of healthy sensor frames on link 1 while link 2 is still
closed, checks PBIT passes (STATUS on link 1), then opens link 2, sends
ARM and checks the answer.
"""
import os
import re
import subprocess
import sys
import time
import tty
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "ros2_ws/src/fts_ground"))
from fts_ground import proto  # noqa: E402

LAT, LON = 473979711, 85461637
MS = 1000


def open_pty(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    return fd


def latest_status(fd: int, parser: proto.Parser, seconds: float) -> proto.Status | None:
    os.set_blocking(fd, False)
    last = None
    end = time.time() + seconds
    while time.time() < end:
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            time.sleep(0.01)
            continue
        for msg_type, payload in parser.feed(data):
            if msg_type == proto.MSG_STATUS:
                last = proto.decode_status(payload)
    return last


def sensor_frames(t0_ms: int, t1_ms: int) -> bytes:
    out = bytearray()
    for t_ms in range(t0_ms, t1_ms, 10):
        t = t_ms * MS
        if t_ms % 100 == 0:
            out += proto.encode_gnss(t, LAT, LON, 500, 3, 10)
            out += proto.encode_ap_heartbeat(t, t_ms // 100)
            out += proto.encode_readback(t, 0, 0)
        if t_ms % 50 == 0:
            out += proto.encode_baro(t, 101325.0, 15.0)
        out += proto.encode_imu(t, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 9.81))
    return bytes(out)


def main() -> None:
    exe = REPO / "build/fts/zephyr/zephyr.exe"
    proc = subprocess.Popen(["stdbuf", "-oL", "-eL", str(exe)], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    try:
        ptys = {}
        while len(ptys) < 2:
            line = proc.stdout.readline()
            if not line:
                raise SystemExit("firmware exited before printing its PTYs")
            m = re.match(r"(uart|uart_1) connected to pseudotty: (\S+)", line)
            if m:
                ptys[m.group(1)] = m.group(2)
        link1 = open_pty(ptys["uart"])
        os.write(link1, sensor_frames(0, 1200))
        st = latest_status(link1, proto.Parser(), 2.0)
        print("link 1 after PBIT:", st)
        if st is None or st.state != "SAFE":
            raise SystemExit("expected SAFE on link 1")
        link2 = open_pty(ptys["uart_1"])
        parser2 = proto.Parser()
        latest_status(link2, parser2, 0.5)  # drain what was queued before open
        os.write(link2, proto.encode_gs_cmd(1, proto.CMD_ARM))
        st = latest_status(link2, parser2, 1.0)
        print("link 2 after ARM:", st)
        if st is None or st.state != "ARMED":
            raise SystemExit("expected ARMED on link 2")
        print("firmware smoke test passed")
    finally:
        proc.terminate()
        proc.wait(5)


if __name__ == "__main__":
    main()
