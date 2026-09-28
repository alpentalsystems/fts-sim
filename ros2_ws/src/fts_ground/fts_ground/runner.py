"""Scenario runner: flies PX4 over MAVLink, injects one fault and records the result."""

import argparse
import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from pymavlink import mavutil
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from fts_ground import metrics
from fts_interfaces.msg import FtsStatus

ORIGIN_LAT = 47.3979711
ORIGIN_LON = 8.5461637
EARTH_R = 6371000.0
TAKEOFF_M = 20.0
SETTLE_S = 3.0
LANDING_WAIT_S = 25.0
FALSE_ALARM_HOLD_S = 12.0
REPOSITION_SPEED_M_S = 8.0
ACK_TIMEOUT_S = 3.0
COMMAND_TRIES = 3
FAULT_TRIES = 5
FAULT_CONFIRM_S = 1.0

EXPECTED_CAUSE = {
    "fence": "FENCE",
    "ceiling": "CEILING",
    "control": "CONTROL",
    "freeze": "AP_FREEZE",
    "link": "LINK",
    "gnss": "GNSS_LOST",
    "manual": "MANUAL",
}
FALSE_ALARMS = ("near_fence", "gnss_jump", "hb_drop")
SCENARIOS = tuple(EXPECTED_CAUSE) + FALSE_ALARMS
MAV = mavutil.mavlink
NAN = math.nan


def local_to_latlon(east: float, north: float) -> tuple[float, float]:
    lat = ORIGIN_LAT + math.degrees(north / EARTH_R)
    lon = ORIGIN_LON + math.degrees(east / (EARTH_R * math.cos(math.radians(ORIGIN_LAT))))
    return lat, lon


def latlon_to_local(lat: float, lon: float) -> tuple[float, float]:
    north = math.radians(lat - ORIGIN_LAT) * EARTH_R
    east = math.radians(lon - ORIGIN_LON) * EARTH_R * math.cos(math.radians(ORIGIN_LAT))
    return east, north


class Px4:
    """PX4 over its ground station MAVLink link (UDP 14550)."""

    def __init__(self) -> None:
        self.m = mavutil.mavlink_connection("udpin:127.0.0.1:14550", source_system=255,
                                            source_component=190)
        if self.m.wait_heartbeat(timeout=60) is None:
            raise RuntimeError("no PX4 heartbeat on UDP 14550")
        self._stop = threading.Event()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _heartbeat(self) -> None:
        while not self._stop.is_set():
            self.m.mav.heartbeat_send(MAV.MAV_TYPE_GCS, MAV.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._stop.wait(1.0)

    def close(self) -> None:
        self._stop.set()

    def _ack(self, cmd: int) -> int | None:
        """Result of the ack for cmd, or None if none came within ACK_TIMEOUT_S."""
        end = time.time() + ACK_TIMEOUT_S
        while time.time() < end:
            ack = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
            if ack is not None and ack.command == cmd:
                return ack.result
        return None

    def command(self, cmd: int, *params: float) -> None:
        """COMMAND_LONG, resent (with a higher confirmation) if no ack arrives."""
        p = list(params) + [0.0] * (7 - len(params))
        for confirmation in range(COMMAND_TRIES):
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd,
                                         confirmation, *p)
            result = self._ack(cmd)
            if result is None:
                print(f"no ack for command {cmd}, try {confirmation + 1}", flush=True)
                continue
            if result != MAV.MAV_RESULT_ACCEPTED:
                raise RuntimeError(f"command {cmd} rejected: result {result}")
            return
        raise RuntimeError(f"no ack for command {cmd} after {COMMAND_TRIES} tries")

    def arm(self) -> None:
        """Arms, retrying while PX4 is still finishing its preflight checks."""
        for attempt in range(15):
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         MAV.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1.0, 0, 0, 0, 0, 0, 0)
            result = self._ack(MAV.MAV_CMD_COMPONENT_ARM_DISARM)
            if result == MAV.MAV_RESULT_ACCEPTED:
                return
            print(f"PX4 arm rejected (result {result}), attempt {attempt + 1}", flush=True)
            time.sleep(2.0)
        raise RuntimeError("PX4 did not arm")

    def takeoff(self) -> None:
        self.command(MAV.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, NAN, NAN, NAN, NAN)

    def land(self) -> None:
        self.command(MAV.MAV_CMD_NAV_LAND, 0, 0, 0, NAN, NAN, NAN, NAN)

    def set_param(self, name: str, value: float) -> None:
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(),
                                  value, MAV.MAV_PARAM_TYPE_REAL32)
        end = time.time() + 5.0
        while time.time() < end:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is not None and msg.param_id == name:
                return
        raise RuntimeError(f"no PARAM_VALUE for {name}")

    def position(self) -> tuple[float, float, float, float]:
        """(east m, north m, relative altitude m, AMSL altitude m)."""
        msg = self.m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is None:
            raise RuntimeError("no GLOBAL_POSITION_INT from PX4")
        east, north = latlon_to_local(msg.lat / 1e7, msg.lon / 1e7)
        return east, north, msg.relative_alt / 1000.0, msg.alt / 1000.0

    def home_amsl(self) -> float:
        _, _, rel, amsl = self.position()
        return amsl - rel

    def wait(self, cond, timeout: float, what: str) -> None:
        end = time.time() + timeout
        while time.time() < end:
            if cond(*self.position()[:3]):
                return
        raise RuntimeError(f"timeout waiting for {what}")

    def reposition(self, home_amsl: float, east: float, north: float, rel_alt: float) -> None:
        lat, lon = local_to_latlon(east, north)
        for attempt in range(COMMAND_TRIES):
            self.m.mav.command_int_send(self.m.target_system, self.m.target_component,
                                        MAV.MAV_FRAME_GLOBAL, MAV.MAV_CMD_DO_REPOSITION, 0, 0,
                                        REPOSITION_SPEED_M_S, 1.0, 0, NAN,
                                        int(lat * 1e7), int(lon * 1e7), home_amsl + rel_alt)
            result = self._ack(MAV.MAV_CMD_DO_REPOSITION)
            if result is None:
                print(f"no ack for reposition, try {attempt + 1}", flush=True)
                continue
            if result != MAV.MAV_RESULT_ACCEPTED:
                raise RuntimeError(f"reposition rejected: result {result}")
            return
        raise RuntimeError(f"no ack for reposition after {COMMAND_TRIES} tries")


class Runner(Node):
    def __init__(self) -> None:
        super().__init__("fts_runner")
        self.status: FtsStatus | None = None
        self.history: list[dict] = []
        # t_last_cmd_us in the first status of each state; PINGs update it later
        self.cmd_at_state: dict[str, int] = {}
        self.create_subscription(FtsStatus, "/fts/status", self.on_status, 10)

    def on_status(self, msg: FtsStatus) -> None:
        if self.status is None or msg.state != self.status.state:
            self.history.append({"t_us": msg.t_us, "state": msg.state, "cause": msg.cause})
            self.cmd_at_state[msg.state] = msg.t_last_cmd_us
            print(f"FTS {msg.state} cause {msg.cause} at t_us {msg.t_us}", flush=True)
        self.status = msg

    def call(self, name: str, srv_type, request) -> None:
        client = self.create_client(srv_type, f"/fts_ground_station/{name}")
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"service {name} not available")
        future = client.call_async(request)
        end = time.time() + 5.0
        while not future.done():
            if time.time() > end:
                raise RuntimeError(f"service {name} timed out")
            time.sleep(0.05)
        response = future.result()
        if not response.success:
            raise RuntimeError(f"service {name}: {response.message}")

    def trigger(self, name: str) -> None:
        self.call(name, Trigger, Trigger.Request())

    def set_link(self, enabled: bool) -> None:
        request = SetBool.Request()
        request.data = enabled
        self.call("link", SetBool, request)

    def wait_state(self, state: str, timeout: float) -> None:
        end = time.time() + timeout
        while time.time() < end:
            if self.status is not None and self.status.state == state:
                return
            time.sleep(0.1)
        last = self.status.state if self.status is not None else None
        raise RuntimeError(f"FTS did not reach {state} (last {last})")


def fault(name: str, repo: Path) -> None:
    """Publishes a fault and waits for the bridge to log it (gz topic -p can be lost)."""
    env = dict(os.environ, GZ_IP="127.0.0.1")
    log = repo / "run/bridge.log"
    applied = f"fts_bridge: fault {name} at"
    for attempt in range(FAULT_TRIES):
        subprocess.run(["gz", "topic", "-t", "/fts/fault", "-m", "gz.msgs.StringMsg",
                        "-p", f'data: "{name}"'], check=True, env=env)
        end = time.time() + FAULT_CONFIRM_S
        while time.time() < end:
            if applied in log.read_text():
                return
            time.sleep(0.05)
        print(f"fault {name} not applied yet, try {attempt + 1}", flush=True)
    raise RuntimeError(f"bridge did not apply fault {name}")


def px4_pid(repo: Path) -> int:
    for line in (repo / "run/pids").read_text().splitlines():
        name, pid = line.split()
        if name == "px4":
            return int(pid)
    raise RuntimeError("px4 not in run/pids")


def run(scenario: str, node: Runner, repo: Path) -> dict:
    px4 = Px4()
    try:
        node.wait_state("SAFE", 90)
        px4.set_param("MIS_TAKEOFF_ALT", TAKEOFF_M)
        home = px4.home_amsl()
        node.trigger("arm")
        node.wait_state("ARMED", 5)
        px4.arm()
        px4.takeoff()
        px4.wait(lambda e, n, rel: rel > TAKEOFF_M - 1.0, 60, "takeoff")
        time.sleep(SETTLE_S)

        if scenario == "fence":
            px4.reposition(home, 150.0, 0.0, TAKEOFF_M)
        elif scenario == "ceiling":
            px4.reposition(home, 0.0, 0.0, 60.0)
        elif scenario == "control":
            fault("cut_motor:0", repo)
        elif scenario == "freeze":
            os.kill(px4_pid(repo), signal.SIGSTOP)
        elif scenario == "link":
            node.set_link(False)
        elif scenario == "gnss":
            fault("gnss_off", repo)
        elif scenario == "manual":
            node.trigger("terminate")
        elif scenario == "near_fence":
            px4.reposition(home, 90.0, 0.0, TAKEOFF_M)
            px4.wait(lambda e, n, rel: e > 85.0, 60, "flight near the fence")
        elif scenario == "gnss_jump":
            fault("gnss_jump", repo)
        elif scenario == "hb_drop":
            fault("hb_drop:5", repo)

        if scenario in FALSE_ALARMS:
            time.sleep(FALSE_ALARM_HOLD_S)
            passed = node.status.state == "ARMED"
            px4.land()
            px4.wait(lambda e, n, rel: rel < 0.5, 90, "landing")
            node.trigger("disarm")
        else:
            node.wait_state("TERMINATED", 120)
            time.sleep(LANDING_WAIT_S)
            passed = node.status.cause == EXPECTED_CAUSE[scenario]
    finally:
        px4.close()

    st = node.status
    status = {"state": st.state, "cause": st.cause, "t_trigger_us": st.t_trigger_us,
              "t_relay_us": st.t_relay_us, "t_chute_us": st.t_chute_us}
    rows = metrics.read_truth(repo / "run/truth.csv")
    # Manual: the TERMINATE time, from the FTS reply to it. Link loss: the last
    # frame sent, which stays fixed once sending stops.
    t_cmd = node.cmd_at_state.get("TERMINATED", -1) if scenario == "manual" else st.t_last_cmd_us
    result = metrics.summarize(scenario, rows, status, t_cmd)
    result["pass"] = passed
    result["status_history"] = node.history
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("scenario", choices=SCENARIOS)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--repo", required=True, type=Path)
    args = ap.parse_args()

    rclpy.init()
    node = Runner()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    try:
        result = run(args.scenario, node, args.repo)
    finally:
        # Stop and join the spin thread before teardown; a thread left in
        # spin() at exit aborts the process.
        executor.shutdown()
        spinner.join()
        node.destroy_node()
        rclpy.try_shutdown()
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "status_history"}, indent=2))
    if not result["pass"]:
        raise SystemExit(1)
