import pytest

from fts_ground import metrics

REST_Z = 0.2
STEP_US = 100_000


def fence_run() -> list[metrics.Row]:
    """On the ground to 1 s, then at 20 m flying east at 5 m/s from x = 80 m;
    trigger at 4.6 s, then a 5 m/s fall to the ground at 8.6 s."""
    rows = []
    for t_us in range(0, 12_000_001, STEP_US):
        t = t_us / 1e6
        if t < 1.0:
            x, z, vz = 0.0, REST_Z, 0.0
        elif t < 4.6:
            x, z, vz = 80.0 + 5.0 * t, REST_Z + 20.0, 0.0
        else:
            x = 80.0 + 5.0 * 4.6
            z = max(REST_Z, REST_Z + 20.0 - 5.0 * (t - 4.6))
            vz = -5.0 if z > REST_Z + 1e-9 else 0.0
        rows.append(metrics.Row(t_us, x, 0.0, z, 0.0, 0.0, vz, t >= 4.6, t >= 4.9, -1, True,
                                min(t_us, 3_000_000)))
    return rows


STATUS = {"state": "TERMINATED", "cause": "FENCE", "t_trigger_us": 4_600_000,
          "t_relay_us": 4_600_000, "t_chute_us": 4_900_000}


def test_fence_timing_and_landing():
    res = metrics.summarize("fence", fence_run(), STATUS, -1)
    assert res["terminated"] and res["landed"]
    assert res["t_fault_us"] == 4_100_000
    assert res["fault_to_trigger_ms"] == 500.0
    assert res["fault_to_relay_ms"] == 500.0
    assert res["fault_to_chute_ms"] == 800.0
    assert res["trigger_altitude_m"] == 20.0
    assert res["vz_under_chute_m_s"] == -5.0
    assert res["drift_m"] == 0.0
    assert res["beyond_fence_m"] == 3.0


def test_freeze_uses_last_heartbeat():
    status = dict(STATUS, cause="AP_FREEZE")
    assert metrics.fault_time("freeze", fence_run(), -1, 4_600_000) == 3_000_000
    res = metrics.summarize("freeze", fence_run(), status, -1)
    assert res["fault_to_trigger_ms"] == 1600.0
    assert "beyond_fence_m" not in res


def test_link_uses_command_time():
    assert metrics.fault_time("link", fence_run(), 100_000, 4_600_000) == 100_000
    assert metrics.fault_time("link", fence_run(), -1, 4_600_000) is None


def test_not_terminated():
    status = {"state": "ARMED", "cause": "NONE", "t_trigger_us": -1, "t_relay_us": -1,
              "t_chute_us": -1}
    res = metrics.summarize("near_fence", fence_run(), status, -1)
    assert res["terminated"] is False
    assert res["max_distance_from_center_m"] == pytest.approx(103.0)


def test_read_truth(tmp_path):
    path = tmp_path / "truth.csv"
    path.write_text("t_us,x,y,z,vx,vy,vz,relay_open,chute_fired,cut_motor,gnss_on,t_hb_us\n"
                    "20000,1.000,2.000,0.250,0.000,0.000,-0.100,0,0,-1,1,0\n")
    rows = metrics.read_truth(path)
    assert rows == [metrics.Row(20000, 1.0, 2.0, 0.25, 0.0, 0.0, -0.1, False, False, -1, True, 0)]


def test_landing_tipped_over_higher_than_start():
    """After a hard landing the drone may rest a little higher than at takeoff."""
    rows = [r if r.t_us < 8_600_000 else metrics.Row(r.t_us, r.x, r.y, REST_Z + 0.31, 0.0, 0.0, 0.0,
                                                      True, True, -1, True, r.t_hb_us)
            for r in fence_run()]
    res = metrics.summarize("fence", rows, STATUS, -1)
    assert res["landed"]
    assert res["vz_under_chute_m_s"] == -5.0


def test_short_fall_under_parachute_still_measured():
    """A tumbling drone can land about 2 s after the parachute opens."""
    rows = [r if r.t_us < 6_900_000 else metrics.Row(r.t_us, r.x, r.y, REST_Z, 0.0, 0.0, 0.0,
                                                      True, True, -1, True, r.t_hb_us)
            for r in fence_run()]
    res = metrics.summarize("fence", rows, STATUS, -1)
    assert res["landed"]
    assert res["vz_under_chute_m_s"] == -5.0


def test_false_alarm_passes_only_without_any_termination():
    armed = [{"state": "SAFE"}, {"state": "ARMED"}]
    assert metrics.false_alarm_passed(armed + [{"state": "SAFE"}], "SAFE")
    assert not metrics.false_alarm_passed(armed + [{"state": "TERMINATED"}], "TERMINATED")
    assert not metrics.false_alarm_passed(armed + [{"state": "TERMINATED"}], "SAFE")
    assert not metrics.false_alarm_passed(armed, "ARMED")  # disarm not taken
