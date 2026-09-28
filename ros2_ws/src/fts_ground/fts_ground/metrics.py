"""Scenario metrics from the bridge's ground-truth CSV and the final FTS status."""

import csv
import math
from dataclasses import dataclass
from pathlib import Path

FENCE_HALF_M = 100.0
CEILING_M = 40.0
# A drone that tips over on landing can rest higher than at takeoff.
LANDED_ABOVE_REST_M = 1.0
LANDED_MAX_VZ_M_S = 0.5
CHUTE_SETTLE_US = 1_000_000
BEFORE_LANDING_US = 200_000


@dataclass(frozen=True)
class Row:
    t_us: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    relay_open: bool
    chute_fired: bool
    cut_motor: int
    gnss_on: bool
    t_hb_us: int


def read_truth(path: Path) -> list[Row]:
    with open(path, newline="") as f:
        rows = [Row(int(r["t_us"]), float(r["x"]), float(r["y"]), float(r["z"]),
                    float(r["vx"]), float(r["vy"]), float(r["vz"]),
                    r["relay_open"] == "1", r["chute_fired"] == "1", int(r["cut_motor"]),
                    r["gnss_on"] == "1", int(r["t_hb_us"]))
                for r in csv.DictReader(f)]
    if not rows:
        raise ValueError(f"no rows in {path}")
    return rows


def _first(rows: list[Row], pred) -> Row | None:
    return next((r for r in rows if pred(r)), None)


def _at(rows: list[Row], t_us: int) -> Row:
    """Last row at or before t_us (the first row if none)."""
    best = rows[0]
    for r in rows:
        if r.t_us > t_us:
            break
        best = r
    return best


def _mean(values: list[float]) -> float | None:
    return round(sum(values) / len(values), 2) if values else None


def fault_time(scenario: str, rows: list[Row], t_cmd_us: int, t_trigger_us: int) -> int | None:
    z0 = rows[0].z
    if scenario == "fence":
        row = _first(rows, lambda r: abs(r.x) > FENCE_HALF_M or abs(r.y) > FENCE_HALF_M)
    elif scenario == "ceiling":
        row = _first(rows, lambda r: r.z - z0 > CEILING_M)
    elif scenario == "control":
        row = _first(rows, lambda r: r.cut_motor >= 0)
    elif scenario == "gnss":
        row = _first(rows, lambda r: not r.gnss_on)
    elif scenario == "freeze":
        return _at(rows, t_trigger_us).t_hb_us
    elif scenario in ("link", "manual"):
        return t_cmd_us if t_cmd_us >= 0 else None
    else:
        return None
    return row.t_us if row is not None else None


def summarize(scenario: str, rows: list[Row], status: dict, t_cmd_us: int) -> dict:
    z0 = rows[0].z
    t_trigger = status["t_trigger_us"]
    t_relay = status["t_relay_us"]
    t_chute = status["t_chute_us"]
    res = {
        "scenario": scenario,
        "state": status["state"],
        "cause": status["cause"],
        "t_trigger_us": t_trigger,
        "t_relay_us": t_relay,
        "t_chute_us": t_chute,
        "max_distance_from_center_m": round(max(max(abs(r.x), abs(r.y)) for r in rows), 1),
        "max_altitude_m": round(max(r.z for r in rows) - z0, 1),
    }
    if t_trigger < 0:
        res["terminated"] = False
        return res
    res["terminated"] = True
    fault = fault_time(scenario, rows, t_cmd_us, t_trigger)
    res["t_fault_us"] = fault
    if fault is not None:
        res["fault_to_trigger_ms"] = round((t_trigger - fault) / 1000.0, 1)
        res["fault_to_relay_ms"] = round((t_relay - fault) / 1000.0, 1)
        res["fault_to_chute_ms"] = round((t_chute - fault) / 1000.0, 1) if t_chute >= 0 else None
    trig = _at(rows, t_trigger)
    res["trigger_altitude_m"] = round(trig.z - z0, 1)
    land = _first(rows, lambda r: r.t_us > t_trigger and r.z < z0 + LANDED_ABOVE_REST_M
                  and abs(r.vz) < LANDED_MAX_VZ_M_S)
    if t_chute >= 0:
        end = land.t_us - BEFORE_LANDING_US if land is not None else rows[-1].t_us
        res["vz_before_chute_m_s"] = _mean([r.vz for r in rows if t_relay <= r.t_us <= t_chute])
        res["vz_under_chute_m_s"] = _mean(
            [r.vz for r in rows if t_chute + CHUTE_SETTLE_US <= r.t_us <= end])
    if land is None:
        res["landed"] = False
        return res
    res["landed"] = True
    res["drift_m"] = round(math.hypot(land.x - trig.x, land.y - trig.y), 1)
    res["landing_xy_m"] = [round(land.x, 1), round(land.y, 1)]
    if scenario == "fence":
        res["beyond_fence_m"] = round(max(abs(land.x), abs(land.y)) - FENCE_HALF_M, 1)
    return res
