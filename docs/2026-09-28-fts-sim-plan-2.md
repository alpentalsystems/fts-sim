# FTS Simulation Plan 2: Integration and Scenarios

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the FTS core from plan 1 as Zephyr firmware on `native_sim`, connect it to a PX4-flown x500 in Gazebo through an I/O bridge, add the parachute, a ROS 2 ground station and a scenario runner, and record every trigger and false-alarm case in a test log.

**Architecture:** Mac is the source of truth; `tools/sync-vm.sh` copies the tree to `~/fts-sim` on the VM, where everything is built and run. Pure logic (app layer, bridge helpers, Python protocol, ground link, metrics) is tested on the Mac first; the Gazebo, Zephyr and ROS 2 parts are checked on the VM with scripted runs.

**Tech Stack:** C11 and Zephyr v4.4.2 (`native_sim/native/64`), C++17 with Gazebo Harmonic (`gz-sim8`, `gz-transport13`, `gz-msgs10`), PX4 v1.15.4 SITL, ROS 2 Humble (`rclpy`, `std_srvs`), Python 3.10 (`pymavlink`, `matplotlib`, `pytest`).

**Spec:** `docs/2026-09-28-fts-sim-design.md`. Facts from the feasibility run: `docs/feasibility.md`.

## Spec deviations (owner to confirm at plan review)

1. The scenario runner commands PX4 over MAVLink (`pymavlink`, UDP 14550) instead of `px4_msgs` through the uXRCE-DDS agent. This avoids building `px4_msgs`; ROS 2 still carries the FTS ground station, its services, the status topic and the rosbag.
2. RViz and PlotJuggler are not used (PlotJuggler is not installed). Graphs are drawn with matplotlib from the bridge's ground-truth CSV; each run also records `/fts/status` with rosbag2.
3. Wind (3 m/s east) acts on the parachute drag only, not on the drone in powered flight.
4. The FTS runs its checks on each IMU frame (100 Hz simulation time) rather than on a kernel timer, so checks follow simulation time exactly.

## Global Constraints

- Public references only; nothing from any former employer's design in code, comments, docs, or commit messages.
- Kernel-style tabs in C and C++; 4 spaces in Python. Comments short, English, plain ASCII.
- C: C11, `-Wall -Wextra -Werror`. C++: C++17, `-Wall -Wextra -Werror`.
- VM: `user@vm-host`, key `~/.ssh/id_ed25519_vm`, repo copy at `~/fts-sim`, PX4 at `~/PX4-Autopilot`, Zephyr workspace at `~/zephyrproject`. Commit only on the Mac.
- Names: world `fts_field`, drone model instance `x500_fts_0`, canopy model `fts_canopy`.
- Gazebo topics: `/fts/imu` (100 Hz), `/fts/navsat` (10 Hz), `/fts/baro` (20 Hz), `/fts/chute` (command), `/fts/chute_state`, `/fts/fault`, `/x500_fts_0/command/motor_speed` (from PX4), `/x500_fts_0/command/motor_speed_gated` (to the motors), `/world/fts_field/clock`, `/world/fts_field/pose/info`.
- FTS demo configuration: origin 47.3979711 N, 8.5461637 E (PX4's default home, also the world origin); fence square +-100 m; ceiling 40 m above the arm point. Other thresholds as in the spec.
- Links: link 1 (FTS uart0) to the bridge; link 2 (FTS uart1) to the ground station. PX4 heartbeat on UDP 14540 at 10 Hz (`px4-mavlink stream -u 14580 -s HEARTBEAT -r 10`).
- Fault names on `/fts/fault`: `cut_motor:N` (N 0..3), `gnss_off`, `gnss_on`, `gnss_jump`, `hb_drop:N`.

## Review Focus

1. **PX4 frozen with SIGSTOP:** no motor messages arrive, but after the relay opens the motors must stop; the bridge publishes zero commands on its own clock (checked in the freeze scenario: the drone falls).
2. **Link 2 not yet opened by the ground station:** the firmware must keep running and pass PBIT (Task 3 smoke test opens link 2 only after PBIT).
3. **Stale PTY names from an earlier run:** `run-sim.sh` must use the PTYs from this run's log only (it deletes old logs first).
4. **Heartbeat stamped slightly ahead of the last IMU time:** the FTS must not treat it as a timeout (plan 1 rebase fix; exercised on every run).
5. **Parachute command repeated or bridge restarted:** the parachute fires once and stays deployed (plugin latches; checked in Task 4).

---

### Task 1: Python frame protocol

**Files:**
- Create: `ros2_ws/src/fts_ground/fts_ground/__init__.py`, `ros2_ws/src/fts_ground/fts_ground/proto.py`, `ros2_ws/src/fts_ground/test/conftest.py`, `ros2_ws/src/fts_ground/test/test_proto.py`
- Modify: `tests/host/test_proto.c`

**Interfaces:**
- Produces (Python module `fts_ground.proto`): constants `SYNC`, `MAX_PAYLOAD`, `MSG_*`, `CMD_*`, `STATES`, `CAUSES`, `UNSET_US`, `STATUS_LEN`; `crc16(data) -> int`; `encode(msg_type, payload) -> bytes`; `encode_imu(t_us, q, gyro, accel)`, `encode_gnss(t_us, lat_e7, lon_e7, alt_mm, fix, sats)`, `encode_baro(t_us, pressure_pa, temp_c)`, `encode_ap_heartbeat(t_us, seq)`, `encode_gs_cmd(seq, cmd)`, `encode_readback(t_us, relay_open, chute_fired)`; `Status` dataclass and `decode_status(payload) -> Status`; `Output` dataclass and `decode_output(payload) -> Output`; `Parser` with `feed(data) -> list[tuple[int, bytes]]` and `crc_errors`.

The byte strings in the tests were produced by the C encoder (`common/proto.c`), so they pin the two implementations to each other.

- [ ] **Step 1: Write the failing test**

Create `ros2_ws/src/fts_ground/test/conftest.py`:

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
```

Create `ros2_ws/src/fts_ground/test/test_proto.py`:

```python
import pytest

from fts_ground import proto

# Produced by common/proto.c.
GS_CMD_FRAME = bytes.fromhex("a5050509000000049023")
STATUS_FRAME = bytes.fromhex(
    "a50823001bb70000000000030100e079af0000000000e079af0000000000"
    "ffffffffffffffffa287")
IMU_FRAME = bytes.fromhex(
    "a50130e8030000000000000000803f000000000000000000000000000000"
    "00000000000000003f0000000000000000c3f51c418f44")


def test_crc16_check_value():
    assert proto.crc16(b"123456789") == 0x29B1


def test_gs_cmd_matches_c_encoder():
    assert proto.encode_gs_cmd(9, proto.CMD_TERMINATE) == GS_CMD_FRAME


def test_imu_matches_c_encoder():
    frame = proto.encode_imu(1000, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.5), (0.0, 0.0, 9.81))
    assert frame == IMU_FRAME


def test_status_from_c_encoder():
    frames = proto.Parser().feed(STATUS_FRAME)
    assert len(frames) == 1
    msg_type, payload = frames[0]
    assert msg_type == proto.MSG_STATUS
    assert proto.decode_status(payload) == proto.Status(
        t_us=12_000_000, state="TERMINATED", cause="FENCE", pbit_fail=0,
        t_trigger_us=11_500_000, t_relay_us=11_500_000, t_chute_us=None)


def test_output_decode():
    assert proto.decode_output(bytes((1, 0))) == proto.Output(relay_open=True, chute_fire=False)


def test_parser_handles_split_noise_and_bad_crc():
    bad = bytearray(GS_CMD_FRAME)
    bad[-1] ^= 0xFF
    data = b"\x00\xa5\x07" + bytes(bad) + GS_CMD_FRAME
    parser = proto.Parser()
    frames = parser.feed(data[:7]) + parser.feed(data[7:])
    assert frames == [(proto.MSG_GS_CMD, GS_CMD_FRAME[3:-2])]
    assert parser.crc_errors >= 1


def test_encode_rejects_long_payload():
    with pytest.raises(ValueError):
        proto.encode(proto.MSG_IMU, bytes(proto.MAX_PAYLOAD + 1))
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest ros2_ws/src/fts_ground/test -q`
Expected: collection error, `ModuleNotFoundError: No module named 'fts_ground'`.

- [ ] **Step 3: Implement**

Create `ros2_ws/src/fts_ground/fts_ground/__init__.py` (empty file).

Create `ros2_ws/src/fts_ground/fts_ground/proto.py`:

```python
"""FTS frame protocol, matching common/include/fts/proto.h."""

import struct
from dataclasses import dataclass

SYNC = 0xA5
MAX_PAYLOAD = 48
OVERHEAD = 5

MSG_IMU = 1
MSG_GNSS = 2
MSG_BARO = 3
MSG_AP_HEARTBEAT = 4
MSG_GS_CMD = 5
MSG_OUTPUT = 6
MSG_READBACK = 7
MSG_STATUS = 8

CMD_PING = 0
CMD_ARM = 1
CMD_DISARM = 2
CMD_TERMINATE_ARM = 3
CMD_TERMINATE = 4

STATES = ("PBIT", "SAFE", "ARMED", "TERMINATED", "FAULT")
CAUSES = ("NONE", "FENCE", "CEILING", "CONTROL", "AP_FREEZE", "LINK", "MANUAL", "GNSS_LOST")

UNSET_US = 0xFFFFFFFFFFFFFFFF

_STATUS = struct.Struct("<QBBBQQQ")
STATUS_LEN = _STATUS.size


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} bytes")
    body = bytes((msg_type, len(payload))) + payload
    return bytes((SYNC,)) + body + struct.pack("<H", crc16(body))


def encode_imu(t_us: int, q, gyro, accel) -> bytes:
    return encode(MSG_IMU, struct.pack("<Q10f", t_us, *q, *gyro, *accel))


def encode_gnss(t_us: int, lat_e7: int, lon_e7: int, alt_mm: int, fix: int, sats: int) -> bytes:
    return encode(MSG_GNSS, struct.pack("<QiiiBB", t_us, lat_e7, lon_e7, alt_mm, fix, sats))


def encode_baro(t_us: int, pressure_pa: float, temp_c: float) -> bytes:
    return encode(MSG_BARO, struct.pack("<Qff", t_us, pressure_pa, temp_c))


def encode_ap_heartbeat(t_us: int, seq: int) -> bytes:
    return encode(MSG_AP_HEARTBEAT, struct.pack("<QI", t_us, seq))


def encode_gs_cmd(seq: int, cmd: int) -> bytes:
    return encode(MSG_GS_CMD, struct.pack("<IB", seq, cmd))


def encode_readback(t_us: int, relay_open: int, chute_fired: int) -> bytes:
    return encode(MSG_READBACK, struct.pack("<QBB", t_us, relay_open, chute_fired))


@dataclass(frozen=True)
class Status:
    t_us: int
    state: str
    cause: str
    pbit_fail: int
    t_trigger_us: int | None
    t_relay_us: int | None
    t_chute_us: int | None


def _time(value: int) -> int | None:
    return None if value == UNSET_US else value


def decode_status(payload: bytes) -> Status:
    t_us, state, cause, pbit_fail, t_trigger, t_relay, t_chute = _STATUS.unpack(payload)
    return Status(t_us, STATES[state], CAUSES[cause], pbit_fail,
                  _time(t_trigger), _time(t_relay), _time(t_chute))


@dataclass(frozen=True)
class Output:
    relay_open: bool
    chute_fire: bool


def decode_output(payload: bytes) -> Output:
    relay_open, chute_fire = struct.unpack("<BB", payload)
    return Output(relay_open != 0, chute_fire != 0)


class Parser:
    """Byte stream to frames. A bad candidate drops only its sync byte."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.crc_errors = 0
        self.len_errors = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buf += data
        frames = []
        while True:
            start = self.buf.find(SYNC)
            if start < 0:
                self.buf.clear()
                return frames
            del self.buf[:start]
            if len(self.buf) < 3:
                return frames
            length = self.buf[2]
            if length > MAX_PAYLOAD:
                self.len_errors += 1
                del self.buf[:1]
                continue
            total = OVERHEAD + length
            if len(self.buf) < total:
                return frames
            body = bytes(self.buf[1:3 + length])
            (rx_crc,) = struct.unpack_from("<H", self.buf, 3 + length)
            if crc16(body) != rx_crc:
                self.crc_errors += 1
                del self.buf[:1]
                continue
            frames.append((body[0], body[2:]))
            del self.buf[:total]
```

Add to `tests/host/test_proto.c`, before `int main(void)`:

```c
static void test_gs_cmd_matches_python_literal(void)
{
	static const uint8_t expected[] = {0xA5, 0x05, 0x05, 0x09, 0x00, 0x00, 0x00, 0x04, 0x90, 0x23};
	struct fts_gs_cmd c = {.seq = 9U, .cmd = FTS_CMD_TERMINATE};
	uint8_t buf[FTS_FRAME_MAX];
	size_t n = fts_encode_gs_cmd(&c, buf, sizeof(buf));

	CHECK(n == sizeof(expected) && memcmp(buf, expected, n) == 0);
}
```

and call `test_gs_cmd_matches_python_literal();` in `main` before `return CHECK_DONE();`. This pins the existing C encoder to the same bytes; it passes at once.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -m pytest ros2_ws/src/fts_ground/test -q && cmake --build build/host >/dev/null && ctest --test-dir build/host | grep "tests passed"`
Expected: `7 passed`, then `100% tests passed, 0 tests failed out of 5`.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/fts_ground tests/host/test_proto.c
git commit -m "feat: add the frame protocol in Python" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: FTS application layer

**Files:**
- Create: `fts/src/logic/app.h`, `fts/src/logic/app.c`, `tests/host/test_app.c`
- Modify: `tests/host/CMakeLists.txt`

**Interfaces:**
- Consumes: `fts.h`, `pbit.h` (plan 1), `fts/proto.h`.
- Produces: `APP_PBIT_WINDOW_US` (1 s), `APP_REPORT_TICKS` (10), `APP_TX_MAX` (128); `struct app_tx { uint8_t link1[APP_TX_MAX]; size_t n1; uint8_t link2[APP_TX_MAX]; size_t n2; }`; `struct app` (fields `fts`, `t_now_us`, `tx_drops`, `rx_unknown`, others internal); `void app_demo_config(struct fts_config *c)`; `void app_init(struct app *a, const struct fts_config *c)`; `void app_on_link1(struct app *a, const struct fts_frame *f, struct app_tx *tx)`; `void app_on_link2(struct app *a, const struct fts_frame *f, struct app_tx *tx)`. The caller zeroes `tx->n1` and `tx->n2`, calls a handler, then sends the bytes.

Behavior: sensor frames update `t_now_us` (the newest sensor time) and feed the FTS; the first sensor frame starts a 1 s PBIT window that counts IMU, GNSS and baro frames, averages the accelerometer norm and uses the last READBACK (none seen counts as a failed readback). Each IMU frame is one FTS tick. OUTPUT goes out on link 1 when the relay or parachute output changes and every 10th tick; STATUS goes out on both links every 10th tick and on a state change. A ground station command is applied at `t_now_us` and answered with STATUS on both links; commands before any sensor frame are ignored.

- [ ] **Step 1: Write the failing test**

Create `tests/host/test_app.c`:

```c
#include <string.h>

#include "app.h"
#include "check.h"

#define MS 1000LL
#define S 1000000LL
#define LAT 473979711
#define LON 85461637

static struct app a;
static struct app_tx tx;
/* What the app sent, collected across calls. */
static int outputs;
static struct fts_output last_out;
static int status1;
static int status2;
static struct fts_status last_st2;

static void reset(void)
{
	struct fts_config c;

	app_demo_config(&c);
	app_init(&a, &c);
	outputs = 0;
	status1 = 0;
	status2 = 0;
	memset(&last_out, 0, sizeof(last_out));
	memset(&last_st2, 0, sizeof(last_st2));
}

static void scan(const uint8_t *buf, size_t n, bool link1)
{
	struct fts_parser p;
	struct fts_frame f;

	fts_parser_init(&p);
	for (size_t i = 0; i < n; i++) {
		if (!fts_parser_feed(&p, buf[i], &f)) {
			continue;
		}
		do {
			if ((f.type == FTS_MSG_OUTPUT) && link1) {
				outputs++;
				CHECK(fts_decode_output(&f, &last_out) == 0);
			} else if ((f.type == FTS_MSG_STATUS) && link1) {
				status1++;
			} else if (f.type == FTS_MSG_STATUS) {
				status2++;
				CHECK(fts_decode_status(&f, &last_st2) == 0);
			}
		} while (fts_parser_next(&p, &f));
	}
}

static struct fts_frame frame_of(const uint8_t *buf, size_t n)
{
	struct fts_parser p;
	struct fts_frame f;

	memset(&f, 0, sizeof(f));
	fts_parser_init(&p);
	for (size_t i = 0; i < n; i++) {
		if (fts_parser_feed(&p, buf[i], &f)) {
			break;
		}
	}
	return f;
}

static void link1_in(const uint8_t *buf, size_t n)
{
	struct fts_frame f = frame_of(buf, n);

	tx.n1 = 0U;
	tx.n2 = 0U;
	app_on_link1(&a, &f, &tx);
	scan(tx.link1, tx.n1, true);
	scan(tx.link2, tx.n2, false);
}

static void command(uint8_t cmd)
{
	struct fts_gs_cmd g = {.seq = 1U, .cmd = cmd};
	uint8_t buf[FTS_FRAME_MAX];
	struct fts_frame f = frame_of(buf, fts_encode_gs_cmd(&g, buf, sizeof(buf)));

	tx.n1 = 0U;
	tx.n2 = 0U;
	app_on_link2(&a, &f, &tx);
	scan(tx.link1, tx.n1, true);
	scan(tx.link2, tx.n2, false);
}

/* Healthy frames from t0 to t1: GNSS, heartbeat, readback 10 Hz; baro 20 Hz; IMU every imu_step. */
static void run(int64_t t0, int64_t t1, int64_t imu_step, bool readback)
{
	uint8_t buf[FTS_FRAME_MAX];

	for (int64_t t = t0; t < t1; t += 10 * MS) {
		uint64_t tu = (uint64_t)t;

		if ((t % (100 * MS)) == 0) {
			struct fts_gnss g = {.t_us = tu, .lat_e7 = LAT, .lon_e7 = LON, .alt_mm = 500,
					     .fix = 3U, .sats = 10U};
			struct fts_ap_heartbeat h = {.t_us = tu, .seq = 0U};
			struct fts_readback r = {.t_us = tu, .relay_open = 0U, .chute_fired = 0U};

			link1_in(buf, fts_encode_gnss(&g, buf, sizeof(buf)));
			link1_in(buf, fts_encode_ap_heartbeat(&h, buf, sizeof(buf)));
			if (readback) {
				link1_in(buf, fts_encode_readback(&r, buf, sizeof(buf)));
			}
		}
		if ((t % (50 * MS)) == 0) {
			struct fts_baro b = {.t_us = tu, .pressure_pa = 101325.0f, .temp_c = 15.0f};

			link1_in(buf, fts_encode_baro(&b, buf, sizeof(buf)));
		}
		if ((t % imu_step) == 0) {
			struct fts_imu i = {.t_us = tu, .q = {1.0f, 0.0f, 0.0f, 0.0f},
					    .gyro = {0.0f, 0.0f, 0.0f}, .accel = {0.0f, 0.0f, 9.81f}};

			link1_in(buf, fts_encode_imu(&i, buf, sizeof(buf)));
		}
	}
}

static void test_pbit_passes_with_healthy_inputs(void)
{
	reset();
	run(0, 1200 * MS, 10 * MS, true);
	CHECK(a.fts.state == FTS_SAFE);
	CHECK(status2 > 0 && last_st2.state == FTS_SAFE);
	CHECK(outputs > 0 && last_out.relay_open == 0U && last_out.chute_fire == 0U);
}

static void test_pbit_fails_without_readback(void)
{
	reset();
	run(0, 1200 * MS, 10 * MS, false);
	CHECK(a.fts.state == FTS_FAULT);
	CHECK((a.fts.pbit_fail & (PBIT_RELAY | PBIT_CHUTE)) == (PBIT_RELAY | PBIT_CHUTE));
	CHECK(last_st2.state == FTS_FAULT && last_st2.pbit_fail == a.fts.pbit_fail);
}

static void test_pbit_fails_on_low_imu_rate(void)
{
	reset();
	run(0, 1200 * MS, 20 * MS, true);
	CHECK(a.fts.state == FTS_FAULT && (a.fts.pbit_fail & PBIT_IMU_RATE) != 0U);
}

static void test_command_before_sensor_time_is_ignored(void)
{
	reset();
	command(FTS_CMD_ARM);
	CHECK(tx.n1 == 0U && tx.n2 == 0U && a.fts.state == FTS_PBIT);
}

static void test_manual_terminate_drives_outputs(void)
{
	reset();
	run(0, 1200 * MS, 10 * MS, true);
	command(FTS_CMD_ARM);
	CHECK(a.fts.state == FTS_ARMED && last_st2.state == FTS_ARMED);
	command(FTS_CMD_TERMINATE_ARM);
	outputs = 0;
	command(FTS_CMD_TERMINATE);
	CHECK(outputs == 1 && last_out.relay_open == 1U && last_out.chute_fire == 0U);
	CHECK(last_st2.state == FTS_TERMINATED && last_st2.cause == FTS_CAUSE_MANUAL);
	run(1200 * MS, 1600 * MS, 10 * MS, true);
	CHECK(last_out.relay_open == 1U && last_out.chute_fire == 1U);
	CHECK(last_st2.t_chute_us == (uint64_t)(1190 * MS + 300 * MS));
}

static void test_status_at_ten_hz(void)
{
	reset();
	run(0, 1200 * MS, 10 * MS, true);
	status1 = 0;
	run(1200 * MS, 2200 * MS, 10 * MS, true);
	CHECK(status1 == 10);
}

int main(void)
{
	test_pbit_passes_with_healthy_inputs();
	test_pbit_fails_without_readback();
	test_pbit_fails_on_low_imu_rate();
	test_command_before_sensor_time_is_ignored();
	test_manual_terminate_drives_outputs();
	test_status_at_ten_hz();
	return CHECK_DONE();
}
```

Append to `tests/host/CMakeLists.txt`:

```cmake
add_executable(test_app test_app.c ${REPO}/fts/src/logic/app.c ${REPO}/fts/src/logic/fts.c
	${REPO}/fts/src/logic/geo.c ${REPO}/fts/src/logic/pbit.c ${REPO}/common/proto.c)
target_include_directories(test_app PRIVATE ${REPO}/fts/src/logic ${REPO}/common/include)
target_link_libraries(test_app m)
add_test(NAME app COMMAND test_app)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S tests/host -B build/host 2>&1 | grep -m2 -i "cannot find\|app.c"`
Expected: CMake reports that `fts/src/logic/app.c` cannot be found.

- [ ] **Step 3: Implement**

Create `fts/src/logic/app.h`:

```c
#ifndef APP_H_
#define APP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fts.h"
#include "fts/proto.h"
#include "pbit.h"

#define APP_PBIT_WINDOW_US 1000000
/* STATUS and periodic OUTPUT every N IMU ticks (10 Hz at 100 Hz IMU). */
#define APP_REPORT_TICKS 10U
#define APP_TX_MAX 128U

/* Bytes to send after one input frame. */
struct app_tx {
	uint8_t link1[APP_TX_MAX];
	size_t n1;
	uint8_t link2[APP_TX_MAX];
	size_t n2;
};

struct app {
	struct fts fts;
	int64_t t_now_us; /* newest sensor time; -1 before the first */
	int64_t pbit_start_us;
	struct pbit_input pbit;
	double accel_sum;
	uint32_t accel_n;
	bool readback_seen;
	struct fts_readback readback;
	uint32_t tick_count;
	bool last_relay;
	bool last_chute;
	enum fts_state last_state;
	uint32_t rx_unknown;
	uint32_t tx_drops;
};

/* Demo site: PX4 default home as origin, +-100 m fence, 40 m ceiling. */
void app_demo_config(struct fts_config *c);
void app_init(struct app *a, const struct fts_config *c);

/* Link 1: sensors, heartbeat and readback in; OUTPUT and STATUS out. */
void app_on_link1(struct app *a, const struct fts_frame *f, struct app_tx *tx);

/* Link 2: ground station commands in; STATUS out. */
void app_on_link2(struct app *a, const struct fts_frame *f, struct app_tx *tx);

#endif /* APP_H_ */
```

Create `fts/src/logic/app.c`:

```c
#include <math.h>
#include <string.h>

#include "app.h"

void app_demo_config(struct fts_config *c)
{
	fts_default_config(c);
	c->origin.lat_e7 = 473979711;
	c->origin.lon_e7 = 85461637;
	c->fence.ceiling_m = 40.0;
}

void app_init(struct app *a, const struct fts_config *c)
{
	memset(a, 0, sizeof(*a));
	fts_init(&a->fts, c);
	a->t_now_us = -1;
	a->pbit_start_us = -1;
	a->last_state = FTS_PBIT;
}

static void put(struct app *a, uint8_t *buf, size_t *n, const uint8_t *frame, size_t len)
{
	if ((len == 0U) || ((*n + len) > APP_TX_MAX)) {
		a->tx_drops++;
		return;
	}
	memcpy(&buf[*n], frame, len);
	*n += len;
}

static void emit_status(struct app *a, struct app_tx *tx)
{
	struct fts_status s = {
		.t_us = (uint64_t)a->t_now_us,
		.state = (uint8_t)a->fts.state,
		.cause = (uint8_t)a->fts.cause,
		.pbit_fail = a->fts.pbit_fail,
		/* -1 (unset) becomes all ones on the wire */
		.t_trigger_us = (uint64_t)a->fts.t_trigger_us,
		.t_relay_us = (uint64_t)a->fts.t_relay_us,
		.t_chute_us = (uint64_t)a->fts.t_chute_us,
	};
	uint8_t f[FTS_FRAME_MAX];
	size_t n = fts_encode_status(&s, f, sizeof(f));

	put(a, tx->link1, &tx->n1, f, n);
	put(a, tx->link2, &tx->n2, f, n);
	a->last_state = a->fts.state;
}

static void emit_outputs(struct app *a, struct app_tx *tx, bool force)
{
	bool relay = fts_relay_open(&a->fts);
	bool chute = fts_chute_fire(&a->fts);
	struct fts_output o;
	uint8_t f[FTS_FRAME_MAX];

	if (!force && (relay == a->last_relay) && (chute == a->last_chute)) {
		return;
	}
	o.relay_open = relay ? 1U : 0U;
	o.chute_fire = chute ? 1U : 0U;
	put(a, tx->link1, &tx->n1, f, fts_encode_output(&o, f, sizeof(f)));
	a->last_relay = relay;
	a->last_chute = chute;
}

static void note_sensor(struct app *a, int64_t t_us)
{
	a->t_now_us = t_us;
	if (a->pbit_start_us < 0) {
		a->pbit_start_us = t_us;
	}
}

static void pbit_check(struct app *a, int64_t t_us)
{
	int64_t window = t_us - a->pbit_start_us;

	if ((a->fts.state != FTS_PBIT) || (window < APP_PBIT_WINDOW_US)) {
		return;
	}
	a->pbit.window_us = window;
	a->pbit.accel_norm = (a->accel_n > 0U) ? (float)(a->accel_sum / (double)a->accel_n) : 0.0f;
	a->pbit.relay_readback_open = !a->readback_seen || (a->readback.relay_open != 0U);
	a->pbit.chute_readback_fired = !a->readback_seen || (a->readback.chute_fired != 0U);
	fts_pbit_done(&a->fts, pbit_eval(&a->pbit), t_us);
}

static void on_imu(struct app *a, const struct fts_imu *m, struct app_tx *tx)
{
	int64_t t = (int64_t)m->t_us;
	bool periodic;

	note_sensor(a, t);
	if (a->fts.state == FTS_PBIT) {
		a->pbit.imu_count++;
		a->accel_sum += sqrt((double)m->accel[0] * m->accel[0] +
				     (double)m->accel[1] * m->accel[1] +
				     (double)m->accel[2] * m->accel[2]);
		a->accel_n++;
	}
	fts_on_imu(&a->fts, t, m->q, m->gyro);
	pbit_check(a, t);
	fts_tick(&a->fts, t);
	a->tick_count++;
	periodic = (a->tick_count % APP_REPORT_TICKS) == 0U;
	emit_outputs(a, tx, periodic);
	if (periodic || (a->fts.state != a->last_state)) {
		emit_status(a, tx);
	}
}

void app_on_link1(struct app *a, const struct fts_frame *f, struct app_tx *tx)
{
	struct fts_imu imu;
	struct fts_gnss g;
	struct fts_baro b;
	struct fts_ap_heartbeat h;
	struct fts_readback r;

	if (fts_decode_imu(f, &imu) == 0) {
		on_imu(a, &imu, tx);
	} else if (fts_decode_gnss(f, &g) == 0) {
		note_sensor(a, (int64_t)g.t_us);
		if (a->fts.state == FTS_PBIT) {
			a->pbit.gnss_count++;
		}
		fts_on_gnss(&a->fts, (int64_t)g.t_us, g.lat_e7, g.lon_e7, g.alt_mm, g.fix);
	} else if (fts_decode_baro(f, &b) == 0) {
		note_sensor(a, (int64_t)b.t_us);
		if (a->fts.state == FTS_PBIT) {
			a->pbit.baro_count++;
		}
		fts_on_baro(&a->fts, (int64_t)b.t_us, b.pressure_pa);
	} else if (fts_decode_ap_heartbeat(f, &h) == 0) {
		fts_on_heartbeat(&a->fts, (int64_t)h.t_us);
	} else if (fts_decode_readback(f, &r) == 0) {
		a->readback = r;
		a->readback_seen = true;
	} else {
		a->rx_unknown++;
	}
}

void app_on_link2(struct app *a, const struct fts_frame *f, struct app_tx *tx)
{
	struct fts_gs_cmd c;

	if (fts_decode_gs_cmd(f, &c) != 0) {
		a->rx_unknown++;
		return;
	}
	if (a->t_now_us < 0) {
		return; /* no simulation time yet */
	}
	(void)fts_on_command(&a->fts, a->t_now_us, c.cmd);
	emit_outputs(a, tx, false);
	emit_status(a, tx);
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cmake -S tests/host -B build/host >/dev/null && cmake --build build/host >/dev/null && ctest --test-dir build/host --output-on-failure`
Expected: `100% tests passed, 0 tests failed out of 6`.

- [ ] **Step 5: Commit**

```bash
git add fts/src/logic/app.h fts/src/logic/app.c tests/host
git commit -m "feat: add the FTS application layer" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: FTS firmware on native_sim

**Files:**
- Create: `tools/sync-vm.sh`, `tools/setup-vm.sh`, `tools/build-vm.sh`, `fts/CMakeLists.txt`, `fts/prj.conf`, `fts/app.overlay`, `fts/src/main.c`, `tests/sim/fw_smoke.py`

**Interfaces:**
- Consumes: `app.h` (Task 2), `fts_ground.proto` (Task 1).
- Produces: `build/fts/zephyr/zephyr.exe` on the VM. At start it prints `uart connected to pseudotty: <path>` (link 1) and `uart_1 connected to pseudotty: <path>` (link 2), then `FTS started`, and one line per state change (`FTS <old> -> <new> ...`).

- [ ] **Step 1: Scripts and the failing smoke test**

Create `tools/sync-vm.sh` (Mac side):

```bash
#!/usr/bin/env bash
# Copies the working tree to ~/fts-sim on the VM.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
VM=${FTS_VM:-user@vm-host}
KEY=${FTS_VM_KEY:-$HOME/.ssh/id_ed25519_vm}
rsync -a --delete \
	--exclude build/ --exclude run/ --exclude runs/ --exclude .superpowers/ \
	--exclude ros2_ws/build/ --exclude ros2_ws/install/ --exclude ros2_ws/log/ \
	--exclude .pytest_cache/ --exclude __pycache__/ \
	-e "ssh -i $KEY" "$REPO/" "$VM:fts-sim/"
```

Create `tools/setup-vm.sh` (VM side, run once):

```bash
#!/usr/bin/env bash
# One-time VM setup: Zephyr v4.4.2 workspace for native_sim, no modules.
set -euo pipefail
sudo apt-get install -y --no-install-recommends device-tree-compiler gperf
python3 -m pip install --user west
export PATH=$HOME/.local/bin:$PATH
if [ ! -d "$HOME/zephyrproject/zephyr" ]; then
	west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.4.2 "$HOME/zephyrproject"
fi
cd "$HOME/zephyrproject"
west config manifest.project-filter -- '-.*'
west update --narrow -o=--depth=1
python3 -m pip install --user -r zephyr/scripts/requirements-base.txt
```

Create `tools/build-vm.sh` (VM side):

```bash
#!/usr/bin/env bash
# Builds everything on the VM.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
export ZEPHYR_BASE=${ZEPHYR_BASE:-$HOME/zephyrproject/zephyr}
export ZEPHYR_TOOLCHAIN_VARIANT=host
export PATH=$HOME/.local/bin:$PATH

# FTS firmware
cmake -GNinja -S "$REPO/fts" -B "$REPO/build/fts" -DBOARD=native_sim/native/64
ninja -C "$REPO/build/fts"
```

Run `chmod +x tools/*.sh`.

Create `tests/sim/fw_smoke.py`:

```python
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
```

- [ ] **Step 2: Set up the VM and watch the smoke test fail**

Run on the Mac: `tools/sync-vm.sh && ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && tools/setup-vm.sh > /tmp/setup-vm.log 2>&1; tail -2 /tmp/setup-vm.log; ls ~/zephyrproject/zephyr/VERSION && python3 tests/sim/fw_smoke.py'`
Expected: setup ends without error and `VERSION` exists; the smoke test fails with `FileNotFoundError` for `build/fts/zephyr/zephyr.exe`.

- [ ] **Step 3: Implement the firmware**

Create `fts/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fts_firmware)

target_include_directories(app PRIVATE src/logic ../common/include)
target_sources(app PRIVATE
	src/main.c
	src/logic/app.c
	src/logic/fts.c
	src/logic/geo.c
	src/logic/pbit.c
	../common/proto.c
)
```

Create `fts/prj.conf`:

```
CONFIG_SERIAL=y
CONFIG_CONSOLE=y
# Console on the host stdout; both UARTs stay free for the binary links.
CONFIG_UART_CONSOLE=n
CONFIG_POSIX_ARCH_CONSOLE=y
CONFIG_PRINTK=y
CONFIG_MAIN_STACK_SIZE=8192
```

Create `fts/app.overlay`:

```dts
/* Link 2 (ground station) on the second native PTY UART. */
&uart1 {
	status = "okay";
};
```

Create `fts/src/main.c`:

```c
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "app.h"
#include "fts/proto.h"

/* Link 1: bridge (sensors, heartbeat, outputs). Link 2: ground station. */
static const struct device *const link1 = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct device *const link2 = DEVICE_DT_GET(DT_NODELABEL(uart1));

static const char *const state_names[] = {"PBIT", "SAFE", "ARMED", "TERMINATED", "FAULT"};
static const char *const cause_names[] = {"NONE", "FENCE", "CEILING", "CONTROL",
					  "AP_FREEZE", "LINK", "MANUAL", "GNSS_LOST"};

static struct app app;
static struct app_tx tx;
static struct fts_parser parser1;
static struct fts_parser parser2;

typedef void (*handler_t)(struct app *a, const struct fts_frame *f, struct app_tx *tx);

static void send(const struct device *dev, const uint8_t *buf, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		uart_poll_out(dev, buf[i]);
	}
}

static void flush(void)
{
	send(link1, tx.link1, tx.n1);
	send(link2, tx.link2, tx.n2);
	tx.n1 = 0U;
	tx.n2 = 0U;
}

static void drain(const struct device *dev, struct fts_parser *p, handler_t on_frame)
{
	unsigned char c;
	struct fts_frame f;

	while (uart_poll_in(dev, &c) == 0) {
		if (!fts_parser_feed(p, c, &f)) {
			continue;
		}
		do {
			on_frame(&app, &f, &tx);
			flush();
		} while (fts_parser_next(p, &f));
	}
}

static void report(enum fts_state *last)
{
	const struct fts *s = &app.fts;

	if (s->state == *last) {
		return;
	}
	printk("FTS %s -> %s", state_names[*last], state_names[s->state]);
	if (s->state == FTS_TERMINATED) {
		printk(" cause %s t_trigger_us %lld", cause_names[s->cause], (long long)s->t_trigger_us);
	} else if (s->state == FTS_FAULT) {
		printk(" pbit_fail 0x%02x", s->pbit_fail);
	}
	printk("\n");
	*last = s->state;
}

int main(void)
{
	struct fts_config cfg;
	enum fts_state last = FTS_PBIT;

	if (!device_is_ready(link1) || !device_is_ready(link2)) {
		printk("FTS links not ready\n");
		return -1;
	}
	app_demo_config(&cfg);
	app_init(&app, &cfg);
	fts_parser_init(&parser1);
	fts_parser_init(&parser2);
	printk("FTS started\n");
	for (;;) {
		drain(link1, &parser1, app_on_link1);
		drain(link2, &parser2, app_on_link2);
		report(&last);
		k_sleep(K_MSEC(1));
	}
	return 0;
}
```

- [ ] **Step 4: Build and run the smoke test**

Run on the Mac: `tools/sync-vm.sh && ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && tools/build-vm.sh > /tmp/build.log 2>&1; grep -iE "warning|error" /tmp/build.log | head; python3 tests/sim/fw_smoke.py'`
Expected: no warnings or errors; `link 1 after PBIT: Status(... state='SAFE' ...)`, `link 2 after ARM: Status(... state='ARMED' ...)`, `firmware smoke test passed`.

- [ ] **Step 5: Commit**

```bash
git add tools fts/CMakeLists.txt fts/prj.conf fts/app.overlay fts/src/main.c tests/sim/fw_smoke.py
git commit -m "feat: run the FTS as Zephyr firmware on native_sim" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: Parachute plugin

**Files:**
- Create: `sim/plugins/CMakeLists.txt`, `sim/plugins/parachute.cpp`, `sim/plugins/test_parachute.cpp`, `sim/plugins/test_drop.sdf`
- Modify: `tools/build-vm.sh`

**Interfaces:**
- Produces: Gazebo system plugin library `FtsParachute` (`build/sim/libFtsParachute.so`), class `fts::Parachute`, attached to a model with SDF parameters `link_name`, `canopy_model`, `cd`, `area`, `air_density`, `wind`, `topic`, `state_topic`. After a `gz.msgs.Boolean` with `data: true` on `topic`, it applies `-0.5 * rho * Cd * A * |v_rel| * v_rel` (with `v_rel = v - wind`) to the link every step, keeps the canopy model 2 m above the link, and publishes the deployed state on `state_topic` on change and once per simulated second. Once fired it stays fired.

- [ ] **Step 1: Write the failing test**

Create `sim/plugins/test_drop.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="drop">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <model name="box">
      <pose>0 0 200 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>2.0</mass>
          <inertia><ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.05</iyy><iyz>0</iyz><izz>0.05</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.4 0.4 0.2</size></box></geometry>
        </collision>
      </link>
      <plugin filename="FtsParachute" name="fts::Parachute">
        <link_name>link</link_name>
        <canopy_model>canopy</canopy_model>
        <cd>1.5</cd>
        <area>0.85</area>
        <air_density>1.225</air_density>
        <wind>0 0 0</wind>
        <topic>/test/chute</topic>
        <state_topic>/test/chute_state</state_topic>
      </plugin>
    </model>
    <model name="canopy">
      <pose>0 0 -20 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

Create `sim/plugins/test_parachute.cpp`:

```cpp
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include "check.h"

namespace {

constexpr double kMass = 2.0;
constexpr double kGravity = 9.8;
constexpr double kCd = 1.5;
constexpr double kArea = 0.85;
constexpr double kRho = 1.225;

void sleep_ms(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace

int main()
{
	gz::sim::ServerConfig config;
	config.SetSdfFile(TEST_WORLD);
	gz::sim::TestFixture fixture(config);
	gz::sim::Link box;
	gz::sim::Link canopy;
	double box_vz = 0.0;
	double box_z = 0.0;
	double canopy_z = 0.0;

	fixture
		.OnConfigure([&](const gz::sim::Entity &world, const std::shared_ptr<const sdf::Element> &,
				 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) {
			gz::sim::World w(world);
			box = gz::sim::Link(gz::sim::Model(w.ModelByName(ecm, "box")).LinkByName(ecm, "link"));
			canopy = gz::sim::Link(
				gz::sim::Model(w.ModelByName(ecm, "canopy")).LinkByName(ecm, "link"));
			box.EnableVelocityChecks(ecm, true);
		})
		.OnPostUpdate([&](const gz::sim::UpdateInfo &, const gz::sim::EntityComponentManager &ecm) {
			const auto v = box.WorldLinearVelocity(ecm);
			const auto p = box.WorldPose(ecm);
			const auto c = canopy.WorldPose(ecm);
			if (v) {
				box_vz = v->Z();
			}
			if (p) {
				box_z = p->Pos().Z();
			}
			if (c) {
				canopy_z = c->Pos().Z();
			}
		})
		.Finalize();

	std::atomic<bool> deployed{false};
	gz::transport::Node node;
	CHECK(node.Subscribe<gz::msgs::Boolean>(
		"/test/chute_state", [&](const gz::msgs::Boolean &m) { deployed = m.data(); }));
	auto pub = node.Advertise<gz::msgs::Boolean>("/test/chute");

	/* 2 s of free fall: no drag before the command. */
	fixture.Server()->Run(true, 500, false);
	std::printf("free fall vz %.2f\n", box_vz);
	CHECK(std::fabs(box_vz + 2.0 * kGravity) < 0.5);

	for (int i = 0; (i < 50) && !pub.HasConnections(); i++) {
		sleep_ms(100);
	}
	CHECK(pub.HasConnections());
	gz::msgs::Boolean fire;
	fire.set_data(true);
	CHECK(pub.Publish(fire));
	CHECK(pub.Publish(fire)); /* a repeated command changes nothing */
	sleep_ms(200);

	/* 10 s under the canopy: close to terminal speed, canopy 2 m above. */
	fixture.Server()->Run(true, 2500, false);
	const double terminal = std::sqrt(2.0 * kMass * kGravity / (kRho * kCd * kArea));
	std::printf("canopy vz %.2f expected %.2f, canopy above box %.2f m\n", box_vz, -terminal,
		    canopy_z - box_z);
	CHECK(std::fabs(box_vz + terminal) < 0.3);
	CHECK(std::fabs(canopy_z - box_z - 2.0) < 0.3);

	for (int i = 0; (i < 20) && !deployed; i++) {
		sleep_ms(100);
	}
	CHECK(deployed);
	return CHECK_DONE();
}
```

Create `sim/plugins/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(fts_sim_plugins CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(FtsParachute SHARED parachute.cpp)
target_link_libraries(FtsParachute PRIVATE
	gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)

enable_testing()
add_executable(test_parachute test_parachute.cpp)
target_include_directories(test_parachute PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/host)
target_compile_definitions(test_parachute PRIVATE TEST_WORLD="${CMAKE_CURRENT_SOURCE_DIR}/test_drop.sdf")
target_link_libraries(test_parachute PRIVATE
	gz-sim8::gz-sim8 gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
add_test(NAME parachute COMMAND test_parachute)
set_tests_properties(parachute PROPERTIES
	ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR};GZ_IP=127.0.0.1")
```

Append to `tools/build-vm.sh`:

```bash

# Gazebo plugins
cmake -GNinja -S "$REPO/sim/plugins" -B "$REPO/build/sim"
ninja -C "$REPO/build/sim"
```

- [ ] **Step 2: Run the test to verify it fails**

Run on the Mac: `tools/sync-vm.sh && ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && cmake -GNinja -S sim/plugins -B build/sim 2>&1 | grep -m2 -i "cannot find\|parachute.cpp"'`
Expected: CMake reports that `parachute.cpp` cannot be found.

- [ ] **Step 3: Implement**

Create `sim/plugins/parachute.cpp`:

```cpp
#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace fts {

/*
 * Parachute. After a true message on <topic>, applies the drag
 * 0.5 * rho * Cd * A * |v| * v against the link's velocity relative to
 * <wind>, and keeps <canopy_model> 2 m above the link. Publishes the
 * deployed state on <state_topic> on change and once per simulated second.
 */
class Parachute : public gz::sim::System,
		  public gz::sim::ISystemConfigure,
		  public gz::sim::ISystemPreUpdate {
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
	{
		const gz::sim::Model model(entity);
		const std::string link_name = sdf->Get<std::string>("link_name", "base_link").first;

		link_ = gz::sim::Link(model.LinkByName(ecm, link_name));
		if (!link_.Valid(ecm)) {
			gzerr << "Parachute: no link [" << link_name << "]\n";
			return;
		}
		link_.EnableVelocityChecks(ecm, true);
		canopy_name_ = sdf->Get<std::string>("canopy_model", "fts_canopy").first;
		cd_ = sdf->Get<double>("cd", 1.5).first;
		area_ = sdf->Get<double>("area", 0.85).first;
		rho_ = sdf->Get<double>("air_density", 1.225).first;
		wind_ = sdf->Get<gz::math::Vector3d>("wind", gz::math::Vector3d::Zero).first;
		const std::string topic = sdf->Get<std::string>("topic", "/fts/chute").first;
		const std::string state_topic =
			sdf->Get<std::string>("state_topic", "/fts/chute_state").first;
		if (!node_.Subscribe(topic, &Parachute::OnCommand, this)) {
			gzerr << "Parachute: cannot subscribe to [" << topic << "]\n";
			return;
		}
		state_pub_ = node_.Advertise<gz::msgs::Boolean>(state_topic);
		configured_ = true;
	}

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
	{
		if (!configured_ || info.paused) {
			return;
		}
		const bool fired = fired_.load();
		const auto now = info.simTime;

		if ((fired != published_) || (now < last_pub_) || (now - last_pub_ >= std::chrono::seconds(1))) {
			gz::msgs::Boolean m;
			m.set_data(fired);
			state_pub_.Publish(m);
			published_ = fired;
			last_pub_ = now;
		}
		if (!fired) {
			return;
		}
		const auto v = link_.WorldLinearVelocity(ecm);
		const auto pose = link_.WorldPose(ecm);
		if (!v || !pose) {
			return;
		}
		const gz::math::Vector3d rel = *v - wind_;
		link_.AddWorldForce(ecm, rel * (-0.5 * rho_ * cd_ * area_ * rel.Length()));
		if (canopy_ == gz::sim::kNullEntity) {
			canopy_ = gz::sim::World(gz::sim::worldEntity(ecm)).ModelByName(ecm, canopy_name_);
		}
		if (canopy_ != gz::sim::kNullEntity) {
			gz::sim::Model(canopy_).SetWorldPoseCmd(
				ecm, gz::math::Pose3d(pose->Pos() + gz::math::Vector3d(0.0, 0.0, 2.0),
						      gz::math::Quaterniond::Identity));
		}
	}

private:
	void OnCommand(const gz::msgs::Boolean &msg)
	{
		if (msg.data()) {
			fired_ = true;
		}
	}

	gz::sim::Link link_;
	gz::sim::Entity canopy_{gz::sim::kNullEntity};
	std::string canopy_name_;
	double cd_{1.5};
	double area_{0.85};
	double rho_{1.225};
	gz::math::Vector3d wind_;
	gz::transport::Node node_;
	gz::transport::Node::Publisher state_pub_;
	std::atomic<bool> fired_{false};
	bool published_{false};
	std::chrono::steady_clock::duration last_pub_{};
	bool configured_{false};
};

} // namespace fts

GZ_ADD_PLUGIN(fts::Parachute, gz::sim::System, fts::Parachute::ISystemConfigure,
	      fts::Parachute::ISystemPreUpdate)
```

- [ ] **Step 4: Build and run the test**

Run on the Mac: `tools/sync-vm.sh && ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && tools/build-vm.sh > /tmp/build.log 2>&1; grep -iE "warning|error" /tmp/build.log | head; ctest --test-dir build/sim --output-on-failure | tail -8'`
Expected: no warnings or errors; `free fall vz -19.6` (within 0.5), `canopy vz -5.0x expected -5.01`, canopy about 2.0 m above the box, `100% tests passed, 0 tests failed out of 1`.

- [ ] **Step 5: Commit**

```bash
git add sim/plugins tools/build-vm.sh
git commit -m "feat: add a Gazebo parachute plugin" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: Drone model and test field

**Files:**
- Create: `sim/models/x500_fts/model.config`, `sim/models/x500_fts/model.sdf`, `sim/models/fts_canopy/model.config`, `sim/models/fts_canopy/model.sdf`, `sim/models/fence_post/model.config`, `sim/models/fence_post/model.sdf`, `sim/worlds/fts_field.sdf`

**Interfaces:**
- Consumes: `FtsParachute` (Task 4); PX4's `x500_base` model (via `GZ_SIM_RESOURCE_PATH`).
- Produces: world `fts_field` with the drone `x500_fts_0` at the origin, the canopy `fts_canopy`, and eight fence posts at the +-100 m square. The drone publishes `/fts/imu` (100 Hz), `/fts/navsat` (10 Hz), `/fts/baro` (20 Hz); its motors listen on `/x500_fts_0/command/motor_speed_gated`; its parachute listens on `/fts/chute` with a 3 m/s east wind.

- [ ] **Step 1: Write the model and world files**

Create `sim/models/x500_fts/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>x500_fts</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>PX4 x500 with its own FTS sensors, gated motor commands and a parachute.</description>
</model>
```

Create `sim/models/x500_fts/model.sdf`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="x500_fts">
    <include merge="true">
      <uri>model://x500_base</uri>
    </include>
    <!-- FTS sensors on their own board, independent of PX4's sensors. -->
    <link name="fts_board">
      <pose>0 0 0.05 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia><ixx>1e-5</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz></inertia>
      </inertial>
      <sensor name="fts_imu" type="imu">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <topic>/fts/imu</topic>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.0003</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.0003</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.0003</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.003</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.003</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.003</stddev></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="fts_navsat" type="navsat">
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <topic>/fts/navsat</topic>
      </sensor>
      <sensor name="fts_baro" type="air_pressure">
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <topic>/fts/baro</topic>
        <air_pressure>
          <pressure><noise type="gaussian"><mean>0</mean><stddev>0.5</stddev></noise></pressure>
        </air_pressure>
      </sensor>
    </link>
    <joint name="fts_board_joint" type="fixed">
      <parent>base_link</parent>
      <child>fts_board</child>
    </joint>
    <!-- Same motors as PX4's x500, but on the gated command topic. -->
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_0_joint</jointName>
      <linkName>rotor_0</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed_gated</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_1_joint</jointName>
      <linkName>rotor_1</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed_gated</commandSubTopic>
      <motorNumber>1</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_2_joint</jointName>
      <linkName>rotor_2</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed_gated</commandSubTopic>
      <motorNumber>2</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_3_joint</jointName>
      <linkName>rotor_3</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed_gated</commandSubTopic>
      <motorNumber>3</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <!-- Parachute: about 5 m/s descent for 2 kg; 3 m/s wind from the west. -->
    <plugin filename="FtsParachute" name="fts::Parachute">
      <link_name>base_link</link_name>
      <canopy_model>fts_canopy</canopy_model>
      <cd>1.5</cd>
      <area>0.85</area>
      <air_density>1.225</air_density>
      <wind>3 0 0</wind>
      <topic>/fts/chute</topic>
      <state_topic>/fts/chute_state</state_topic>
    </plugin>
  </model>
</sdf>
```

Create `sim/models/fts_canopy/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>fts_canopy</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Parachute canopy shown above the drone after deployment (visual only).</description>
</model>
```

Create `sim/models/fts_canopy/model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="fts_canopy">
    <link name="link">
      <gravity>false</gravity>
      <inertial>
        <mass>0.1</mass>
        <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia>
      </inertial>
      <visual name="canopy">
        <geometry><sphere><radius>0.52</radius></sphere></geometry>
        <pose>0 0 0 0 0 0</pose>
        <material>
          <ambient>1 0.4 0 1</ambient>
          <diffuse>1 0.4 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

Create `sim/models/fence_post/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>fence_post</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Geofence marker post.</description>
</model>
```

Create `sim/models/fence_post/model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="fence_post">
    <static>true</static>
    <link name="link">
      <pose>0 0 2 0 0 0</pose>
      <visual name="post">
        <geometry><cylinder><radius>0.15</radius><length>4</length></cylinder></geometry>
        <material>
          <ambient>0.9 0.1 0.1 1</ambient>
          <diffuse>0.9 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

Create `sim/worlds/fts_field.sdf`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="fts_field">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system"/>
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
    <plugin name="gz::sim::systems::AirPressure" filename="gz-sim-air-pressure-system"/>
    <plugin name="gz::sim::systems::ApplyLinkWrench" filename="gz-sim-apply-link-wrench-system"/>
    <plugin name="gz::sim::systems::NavSat" filename="gz-sim-navsat-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.8 0.9 1</background>
      <grid>false</grid>
    </scene>
    <gravity>0 0 -9.8</gravity>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
          <material>
            <ambient>0.45 0.6 0.35 1</ambient>
            <diffuse>0.45 0.6 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <include><uri>model://fence_post</uri><name>post_ne</name><pose>100 100 0 0 0 0</pose></include>
    <include><uri>model://fence_post</uri><name>post_n</name><pose>0 100 0 0 0 0</pose></include>
    <include><uri>model://fence_post</uri><name>post_nw</name><pose>-100 100 0 0 0 0</pose></include>
    <include><uri>model://fence_post</uri><name>post_w</name><pose>-100 0 0 0 0 0</pose></include>
    <include><uri>model://fence_post</uri><name>post_sw</name><pose>-100 -100 0 0 0 0</pose></include>
    <include><uri>model://fence_post</uri><name>post_s</name><pose>0 -100 0 0 0 0</pose></include>
    <include><uri>model://fence_post</uri><name>post_se</name><pose>100 -100 0 0 0 0</pose></include>
    <include><uri>model://fence_post</uri><name>post_e</name><pose>100 0 0 0 0 0</pose></include>
    <include><uri>model://fts_canopy</uri><name>fts_canopy</name><pose>0 0 -20 0 0 0</pose></include>
    <include><uri>model://x500_fts</uri><name>x500_fts_0</name><pose>0 0 0.25 0 0 0</pose></include>
  </world>
</sdf>
```

- [ ] **Step 2: Check the world on the VM**

Run on the Mac:

```bash
tools/sync-vm.sh && ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim &&
export GZ_IP=127.0.0.1 GZ_SIM_RESOURCE_PATH=$PWD/sim/models:$PWD/sim/worlds:$HOME/PX4-Autopilot/Tools/simulation/gz/models GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/sim &&
(gz sim -r -s sim/worlds/fts_field.sdf > /tmp/gz-world.log 2>&1 & echo $! > /tmp/gz-world.pid) && sleep 12 &&
gz topic -l | grep -E "^/fts/|motor_speed_gated|/world/fts_field/(clock|pose/info)" &&
echo imu $(timeout 5 gz topic -e -t /fts/imu | grep -c "^header") navsat $(timeout 5 gz topic -e -t /fts/navsat | grep -c "^header") baro $(timeout 5 gz topic -e -t /fts/baro | grep -c "^header") &&
gz topic -i -t /x500_fts_0/command/motor_speed_gated | head -4; gz topic -i -t /fts/chute | head -4;
grep -iE "error|fail" /tmp/gz-world.log | head -5; kill $(cat /tmp/gz-world.pid)'
```

Expected: topics `/fts/imu`, `/fts/navsat`, `/fts/baro`, `/fts/chute` (subscriber), `/x500_fts_0/command/motor_speed_gated` (subscriber), `/world/fts_field/clock`, `/world/fts_field/pose/info`; counts over 5 s about `imu 500 navsat 50 baro 100` (within 20%); no errors in the log. If a sensor topic is missing, check that the world lists the matching sensor system.

- [ ] **Step 3: Commit**

```bash
git add sim/models sim/worlds
git commit -m "feat: add the drone model with FTS sensors and the test field" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: I/O bridge

**Files:**
- Modify: `common/include/fts/proto.h`, `tools/build-vm.sh`
- Create: `bridge/CMakeLists.txt`, `bridge/src/mavlink_hb.h`, `bridge/src/mavlink_hb.cpp`, `bridge/src/gate.h`, `bridge/src/gate.cpp`, `bridge/src/convert.h`, `bridge/src/convert.cpp`, `bridge/src/pty.h`, `bridge/src/pty.cpp`, `bridge/src/main.cpp`, `bridge/tests/test_mavlink_hb.cpp`, `bridge/tests/test_gate.cpp`, `bridge/tests/test_convert.cpp`, `tests/sim/bridge_check.py`

**Interfaces:**
- Consumes: `fts/proto.h` (plan 1), the world and topics from Task 5.
- Produces:
  - `bool mav_has_heartbeat(const uint8_t *buf, size_t n, uint8_t sysid, uint8_t compid)` (MAVLink v2 HEARTBEAT with a valid CRC);
  - `std::vector<double> gate_apply(const std::vector<double> &in, bool relay_open, int cut)` (relay open zeroes all; `cut` in range zeroes that motor; `-1` none);
  - `uint64_t stamp_to_us(int64_t sec, int32_t nsec)`, `int32_t deg_to_e7(double deg)`, `int32_t m_to_mm(double m)` (both round to nearest);
  - `int pty_open_raw(const std::string &path)` (throws `std::runtime_error`);
  - executable `fts_bridge --link PTY --truth FILE [--model x500_fts_0] [--world fts_field] [--mav-port 14540]`. It sends IMU, GNSS, BARO frames from `/fts/*`, AP_HEARTBEAT for each PX4 heartbeat on the UDP port, READBACK at 10 Hz simulation time; it gates `/<model>/command/motor_speed` to `.../motor_speed_gated` and, while the relay is open, publishes zero commands every 10 ms simulation time on its own; it publishes `/fts/chute` once when OUTPUT asks for the parachute; it applies faults from `/fts/fault`; it writes the ground-truth CSV `t_us,x,y,z,vx,vy,vz,relay_open,chute_fired,cut_motor,gnss_on,t_hb_us` at 50 Hz simulation time.

- [ ] **Step 1: Structural change (separate commit): C++ guards in the protocol header**

In `common/include/fts/proto.h`, after `#include <stdint.h>` add:

```c

#ifdef __cplusplus
extern "C" {
#endif
```

and before `#endif /* FTS_PROTO_H_ */` add:

```c
#ifdef __cplusplus
}
#endif

```

Run: `cmake --build build/host >/dev/null && ctest --test-dir build/host | grep "tests passed"`
Expected: `100% tests passed, 0 tests failed out of 6`.

```bash
git add common/include/fts/proto.h
git commit -m "refactor: allow the protocol header in C++" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

- [ ] **Step 2: Write the failing tests**

Create `bridge/tests/test_mavlink_hb.cpp`:

```cpp
#include <vector>

#include "check.h"
#include "mavlink_hb.h"

/* HEARTBEAT from sysid 1, compid 1 (pymavlink, MAVLink v2). */
static const uint8_t kHeartbeat[] = {0xfd, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
				     0x03, 0x01, 0x00, 0x02, 0x0c, 0x9d, 0x04, 0x03, 0x4e, 0x8f};

int main()
{
	std::vector<uint8_t> bad(kHeartbeat, kHeartbeat + sizeof(kHeartbeat));
	std::vector<uint8_t> noisy = {0x00, 0xfd, 0x01};

	CHECK(mav_has_heartbeat(kHeartbeat, sizeof(kHeartbeat), 1, 1));
	CHECK(!mav_has_heartbeat(kHeartbeat, sizeof(kHeartbeat), 1, 2));
	CHECK(!mav_has_heartbeat(kHeartbeat, sizeof(kHeartbeat) - 1U, 1, 1));
	bad[12] ^= 0x01U;
	CHECK(!mav_has_heartbeat(bad.data(), bad.size(), 1, 1));
	/* a false start byte before the packet must not hide it */
	noisy.insert(noisy.end(), kHeartbeat, kHeartbeat + sizeof(kHeartbeat));
	CHECK(mav_has_heartbeat(noisy.data(), noisy.size(), 1, 1));
	return CHECK_DONE();
}
```

Create `bridge/tests/test_gate.cpp`:

```cpp
#include <vector>

#include "check.h"
#include "gate.h"

int main()
{
	const std::vector<double> in = {100.0, 200.0, 300.0, 400.0};
	const std::vector<double> cut = gate_apply(in, false, 2);

	CHECK(gate_apply(in, false, -1) == in);
	CHECK(gate_apply(in, true, -1) == std::vector<double>(4, 0.0));
	CHECK(gate_apply(in, true, 1) == std::vector<double>(4, 0.0));
	CHECK(cut[0] == 100.0 && cut[1] == 200.0 && cut[2] == 0.0 && cut[3] == 400.0);
	return CHECK_DONE();
}
```

Create `bridge/tests/test_convert.cpp`:

```cpp
#include "check.h"
#include "convert.h"

int main()
{
	CHECK(stamp_to_us(12, 345678000) == 12345678ULL);
	CHECK(stamp_to_us(0, 999) == 0ULL);
	CHECK(deg_to_e7(47.3979711) == 473979711);
	CHECK(deg_to_e7(-8.54616374) == -85461637);
	CHECK(m_to_mm(488.0) == 488000);
	CHECK(m_to_mm(-1.2345) == -1235);
	return CHECK_DONE();
}
```

Create `bridge/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(fts_bridge C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_library(bridge_core STATIC src/mavlink_hb.cpp src/gate.cpp src/convert.cpp ${REPO}/common/proto.c)
target_include_directories(bridge_core PUBLIC src ${REPO}/common/include)

enable_testing()
foreach(name mavlink_hb gate convert)
	add_executable(test_${name} tests/test_${name}.cpp)
	target_include_directories(test_${name} PRIVATE ${REPO}/tests/host)
	target_link_libraries(test_${name} PRIVATE bridge_core)
	add_test(NAME ${name} COMMAND test_${name})
endforeach()

option(FTS_BRIDGE_GZ "Build the Gazebo bridge executable" ON)
if(FTS_BRIDGE_GZ)
	find_package(Threads REQUIRED)
	find_package(gz-transport13 REQUIRED)
	find_package(gz-msgs10 REQUIRED)
	add_executable(fts_bridge src/main.cpp src/pty.cpp)
	target_link_libraries(fts_bridge PRIVATE bridge_core Threads::Threads
		gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
endif()
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cmake -S bridge -B build/bridge-host -DFTS_BRIDGE_GZ=OFF 2>&1 | grep -m2 -i "cannot find"`
Expected: CMake reports that `src/mavlink_hb.cpp` cannot be found.

- [ ] **Step 4: Implement the helpers**

Create `bridge/src/mavlink_hb.h`:

```cpp
#ifndef BRIDGE_MAVLINK_HB_H_
#define BRIDGE_MAVLINK_HB_H_

#include <cstddef>
#include <cstdint>

/* True if buf holds a MAVLink v2 HEARTBEAT from sysid/compid with a valid CRC. */
bool mav_has_heartbeat(const uint8_t *buf, size_t n, uint8_t sysid, uint8_t compid);

#endif /* BRIDGE_MAVLINK_HB_H_ */
```

Create `bridge/src/mavlink_hb.cpp`:

```cpp
#include "mavlink_hb.h"

namespace {

constexpr uint8_t kStx = 0xFD;
constexpr size_t kHeader = 10;   /* stx, len, incompat, compat, seq, sysid, compid, msgid x3 */
constexpr size_t kSignature = 13;
constexpr uint8_t kSigned = 0x01;
constexpr uint8_t kHeartbeatCrcExtra = 50;

uint16_t x25(uint16_t crc, uint8_t b)
{
	uint8_t t = static_cast<uint8_t>(b ^ static_cast<uint8_t>(crc & 0xFFU));

	t = static_cast<uint8_t>(t ^ static_cast<uint8_t>(t << 4));
	return static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(t) << 8) ^
				     (static_cast<uint16_t>(t) << 3) ^ (t >> 4));
}

} // namespace

bool mav_has_heartbeat(const uint8_t *buf, size_t n, uint8_t sysid, uint8_t compid)
{
	for (size_t i = 0; i + kHeader + 2U <= n; i++) {
		if (buf[i] != kStx) {
			continue;
		}
		const size_t len = buf[i + 1];
		const size_t total = kHeader + len + 2U + (((buf[i + 2] & kSigned) != 0U) ? kSignature : 0U);
		const uint32_t msgid = buf[i + 7] | (buf[i + 8] << 8) | (static_cast<uint32_t>(buf[i + 9]) << 16);

		if ((i + total > n) || (msgid != 0U) || (buf[i + 5] != sysid) || (buf[i + 6] != compid)) {
			continue;
		}
		uint16_t crc = 0xFFFF;
		for (size_t k = i + 1; k < i + kHeader + len; k++) {
			crc = x25(crc, buf[k]);
		}
		crc = x25(crc, kHeartbeatCrcExtra);
		const uint16_t rx = static_cast<uint16_t>(buf[i + kHeader + len] | (buf[i + kHeader + len + 1] << 8));
		if (crc == rx) {
			return true;
		}
	}
	return false;
}
```

Create `bridge/src/gate.h`:

```cpp
#ifndef BRIDGE_GATE_H_
#define BRIDGE_GATE_H_

#include <vector>

/* Motor gate: relay open zeroes all motors; cut (0-based, -1 none) zeroes one motor. */
std::vector<double> gate_apply(const std::vector<double> &in, bool relay_open, int cut);

#endif /* BRIDGE_GATE_H_ */
```

Create `bridge/src/gate.cpp`:

```cpp
#include "gate.h"

std::vector<double> gate_apply(const std::vector<double> &in, bool relay_open, int cut)
{
	if (relay_open) {
		return std::vector<double>(in.size(), 0.0);
	}
	std::vector<double> out = in;
	if ((cut >= 0) && (static_cast<size_t>(cut) < out.size())) {
		out[static_cast<size_t>(cut)] = 0.0;
	}
	return out;
}
```

Create `bridge/src/convert.h`:

```cpp
#ifndef BRIDGE_CONVERT_H_
#define BRIDGE_CONVERT_H_

#include <cstdint>

uint64_t stamp_to_us(int64_t sec, int32_t nsec);
int32_t deg_to_e7(double deg);
int32_t m_to_mm(double m);

#endif /* BRIDGE_CONVERT_H_ */
```

Create `bridge/src/convert.cpp`:

```cpp
#include <cmath>

#include "convert.h"

uint64_t stamp_to_us(int64_t sec, int32_t nsec)
{
	return static_cast<uint64_t>(sec) * 1000000ULL + static_cast<uint64_t>(nsec / 1000);
}

int32_t deg_to_e7(double deg)
{
	return static_cast<int32_t>(std::llround(deg * 1e7));
}

int32_t m_to_mm(double m)
{
	return static_cast<int32_t>(std::llround(m * 1000.0));
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cmake -S bridge -B build/bridge-host -DFTS_BRIDGE_GZ=OFF >/dev/null && cmake --build build/bridge-host >/dev/null && ctest --test-dir build/bridge-host --output-on-failure`
Expected: `100% tests passed, 0 tests failed out of 3`.

- [ ] **Step 6: Implement the executable and its check**

Create `bridge/src/pty.h`:

```cpp
#ifndef BRIDGE_PTY_H_
#define BRIDGE_PTY_H_

#include <string>

/* Opens a PTY slave in raw mode; throws std::runtime_error on failure. */
int pty_open_raw(const std::string &path);

#endif /* BRIDGE_PTY_H_ */
```

Create `bridge/src/pty.cpp`:

```cpp
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "pty.h"

int pty_open_raw(const std::string &path)
{
	const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY);
	if (fd < 0) {
		throw std::runtime_error("cannot open " + path + ": " + std::strerror(errno));
	}
	termios t{};
	if (::tcgetattr(fd, &t) != 0) {
		const int e = errno;
		::close(fd);
		throw std::runtime_error("tcgetattr " + path + ": " + std::strerror(e));
	}
	::cfmakeraw(&t);
	if (::tcsetattr(fd, TCSANOW, &t) != 0) {
		const int e = errno;
		::close(fd);
		throw std::runtime_error("tcsetattr " + path + ": " + std::strerror(e));
	}
	return fd;
}
```

Create `bridge/src/main.cpp`:

```cpp
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/WaitHelpers.hh>

#include "convert.h"
#include "fts/proto.h"
#include "gate.h"
#include "mavlink_hb.h"
#include "pty.h"

namespace {

constexpr uint64_t kReadbackPeriodUs = 100000;
constexpr uint64_t kZeroPeriodUs = 10000;
constexpr uint64_t kTruthPeriodUs = 20000;
constexpr double kGnssJumpDeg = 0.02; /* about 2.2 km north */
constexpr int kMotors = 4;
constexpr uint8_t kPx4SysId = 1;
constexpr uint8_t kPx4CompId = 1;

struct Args {
	std::string link;
	std::string truth;
	std::string model = "x500_fts_0";
	std::string world = "fts_field";
	int mav_port = 14540;
};

Args parse_args(int argc, char **argv)
{
	Args a;

	for (int i = 1; i < argc; i += 2) {
		const std::string key = argv[i];
		if (i + 1 >= argc) {
			throw std::runtime_error("missing value for " + key);
		}
		const std::string value = argv[i + 1];
		if (key == "--link") {
			a.link = value;
		} else if (key == "--truth") {
			a.truth = value;
		} else if (key == "--model") {
			a.model = value;
		} else if (key == "--world") {
			a.world = value;
		} else if (key == "--mav-port") {
			a.mav_port = std::stoi(value);
		} else {
			throw std::runtime_error("unknown option " + key);
		}
	}
	if (a.link.empty() || a.truth.empty()) {
		throw std::runtime_error("usage: fts_bridge --link PTY --truth FILE [--model NAME] "
					 "[--world NAME] [--mav-port N]");
	}
	return a;
}

uint64_t stamp_of(const gz::msgs::Header &h)
{
	return stamp_to_us(h.stamp().sec(), h.stamp().nsec());
}

/* Next due time for a periodic action; restarts if the clock went back. */
bool due(uint64_t t, uint64_t &next, uint64_t period)
{
	if ((t < next) && (next - t <= period)) {
		return false;
	}
	next = t + period;
	return true;
}

class Bridge {
public:
	explicit Bridge(const Args &a) : args_(a)
	{
		fd_ = pty_open_raw(a.link);
		truth_ = std::fopen(a.truth.c_str(), "w");
		if (truth_ == nullptr) {
			throw std::runtime_error("cannot open " + a.truth + ": " + std::strerror(errno));
		}
		std::fprintf(truth_, "t_us,x,y,z,vx,vy,vz,relay_open,chute_fired,cut_motor,gnss_on,t_hb_us\n");
		udp_ = open_udp(a.mav_port);
		gated_pub_ = node_.Advertise<gz::msgs::Actuators>("/" + a.model + "/command/motor_speed_gated");
		chute_pub_ = node_.Advertise<gz::msgs::Boolean>("/fts/chute");
		const std::string world = "/world/" + a.world;
		sub(world + "/clock", &Bridge::on_clock);
		sub(world + "/pose/info", &Bridge::on_pose);
		sub("/fts/imu", &Bridge::on_imu);
		sub("/fts/navsat", &Bridge::on_navsat);
		sub("/fts/baro", &Bridge::on_baro);
		sub("/fts/fault", &Bridge::on_fault);
		sub("/fts/chute_state", &Bridge::on_chute_state);
		sub("/" + a.model + "/command/motor_speed", &Bridge::on_motor);
	}

	void start()
	{
		std::thread(&Bridge::link_reader, this).detach();
		std::thread(&Bridge::mav_reader, this).detach();
	}

	void close_truth()
	{
		std::lock_guard<std::mutex> lk(truth_mu_);
		std::fclose(truth_);
		truth_ = nullptr;
	}

private:
	template <typename M> void sub(const std::string &topic, void (Bridge::*cb)(const M &))
	{
		if (!node_.Subscribe(topic, cb, this)) {
			throw std::runtime_error("cannot subscribe to " + topic);
		}
	}

	static int open_udp(int port)
	{
		const int s = ::socket(AF_INET, SOCK_DGRAM, 0);
		if (s < 0) {
			throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
		}
		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(static_cast<uint16_t>(port));
		addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		if (::bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
			throw std::runtime_error("bind UDP " + std::to_string(port) + ": " + std::strerror(errno));
		}
		return s;
	}

	void send(const uint8_t *buf, size_t n)
	{
		if (n == 0U) {
			throw std::logic_error("frame encoding failed");
		}
		std::lock_guard<std::mutex> lk(tx_mu_);
		size_t off = 0;
		while (off < n) {
			const ssize_t w = ::write(fd_, buf + off, n - off);
			if (w < 0) {
				if (errno == EINTR) {
					continue;
				}
				std::cerr << "fts_bridge: link write failed: " << std::strerror(errno) << "\n";
				std::exit(1);
			}
			off += static_cast<size_t>(w);
		}
	}

	void publish_motors(const std::vector<double> &v)
	{
		gz::msgs::Actuators out;
		for (double x : v) {
			out.add_velocity(x);
		}
		gated_pub_.Publish(out);
	}

	void on_clock(const gz::msgs::Clock &m)
	{
		const uint64_t t = stamp_to_us(m.sim().sec(), m.sim().nsec());
		std::lock_guard<std::mutex> lk(clock_mu_);

		t_us_ = t;
		if (due(t, next_readback_us_, kReadbackPeriodUs)) {
			fts_readback r{};
			uint8_t buf[FTS_FRAME_MAX];
			r.t_us = t;
			r.relay_open = relay_open_ ? 1U : 0U;
			r.chute_fired = chute_state_ ? 1U : 0U;
			send(buf, fts_encode_readback(&r, buf, sizeof(buf)));
		}
		if (relay_open_ && due(t, next_zero_us_, kZeroPeriodUs)) {
			publish_motors(std::vector<double>(kMotors, 0.0));
		}
	}

	void on_imu(const gz::msgs::IMU &m)
	{
		fts_imu f{};
		uint8_t buf[FTS_FRAME_MAX];

		f.t_us = stamp_of(m.header());
		f.q[0] = static_cast<float>(m.orientation().w());
		f.q[1] = static_cast<float>(m.orientation().x());
		f.q[2] = static_cast<float>(m.orientation().y());
		f.q[3] = static_cast<float>(m.orientation().z());
		f.gyro[0] = static_cast<float>(m.angular_velocity().x());
		f.gyro[1] = static_cast<float>(m.angular_velocity().y());
		f.gyro[2] = static_cast<float>(m.angular_velocity().z());
		f.accel[0] = static_cast<float>(m.linear_acceleration().x());
		f.accel[1] = static_cast<float>(m.linear_acceleration().y());
		f.accel[2] = static_cast<float>(m.linear_acceleration().z());
		send(buf, fts_encode_imu(&f, buf, sizeof(buf)));
	}

	void on_navsat(const gz::msgs::NavSat &m)
	{
		fts_gnss f{};
		uint8_t buf[FTS_FRAME_MAX];
		double lat = m.latitude_deg();

		if (!gnss_on_) {
			return;
		}
		if (gnss_jump_.exchange(false)) {
			lat += kGnssJumpDeg;
		}
		f.t_us = stamp_of(m.header());
		f.lat_e7 = deg_to_e7(lat);
		f.lon_e7 = deg_to_e7(m.longitude_deg());
		f.alt_mm = m_to_mm(m.altitude());
		f.fix = 3U;
		f.sats = 10U;
		send(buf, fts_encode_gnss(&f, buf, sizeof(buf)));
	}

	void on_baro(const gz::msgs::FluidPressure &m)
	{
		fts_baro f{};
		uint8_t buf[FTS_FRAME_MAX];

		f.t_us = stamp_of(m.header());
		f.pressure_pa = static_cast<float>(m.pressure());
		f.temp_c = 15.0f;
		send(buf, fts_encode_baro(&f, buf, sizeof(buf)));
	}

	void on_motor(const gz::msgs::Actuators &m)
	{
		const std::vector<double> in(m.velocity().begin(), m.velocity().end());
		gz::msgs::Actuators out;

		*out.mutable_header() = m.header();
		for (double v : gate_apply(in, relay_open_, cut_)) {
			out.add_velocity(v);
		}
		gated_pub_.Publish(out);
	}

	static int parse_count(const std::string &s, size_t prefix)
	{
		size_t used = 0;
		const int v = std::stoi(s.substr(prefix), &used);
		if (used != s.size() - prefix) {
			throw std::invalid_argument(s);
		}
		return v;
	}

	void on_fault(const gz::msgs::StringMsg &m)
	{
		const std::string &s = m.data();
		try {
			if (s.rfind("cut_motor:", 0) == 0) {
				const int n = parse_count(s, 10);
				if ((n < 0) || (n >= kMotors)) {
					throw std::invalid_argument(s);
				}
				cut_ = n;
			} else if (s == "gnss_off") {
				gnss_on_ = false;
			} else if (s == "gnss_on") {
				gnss_on_ = true;
			} else if (s == "gnss_jump") {
				gnss_jump_ = true;
			} else if (s.rfind("hb_drop:", 0) == 0) {
				hb_drop_ = parse_count(s, 8);
			} else {
				throw std::invalid_argument(s);
			}
		} catch (const std::exception &) {
			std::cerr << "fts_bridge: unknown fault [" << s << "]\n";
			return;
		}
		std::cerr << "fts_bridge: fault " << s << " at t_us " << t_us_.load() << "\n";
	}

	void on_chute_state(const gz::msgs::Boolean &m)
	{
		chute_state_ = m.data();
	}

	void on_pose(const gz::msgs::Pose_V &m)
	{
		const uint64_t t = stamp_of(m.header());

		for (const auto &p : m.pose()) {
			if (p.name() == args_.model) {
				write_truth(t, p.position().x(), p.position().y(), p.position().z());
				return;
			}
		}
	}

	void write_truth(uint64_t t, double x, double y, double z)
	{
		std::lock_guard<std::mutex> lk(truth_mu_);
		double vx = 0.0;
		double vy = 0.0;
		double vz = 0.0;

		if (truth_ == nullptr) {
			return;
		}
		if (have_prev_ && (t > prev_t_) && (t - prev_t_ < kTruthPeriodUs)) {
			return;
		}
		if (have_prev_ && (t > prev_t_)) {
			const double dt = static_cast<double>(t - prev_t_) / 1e6;
			vx = (x - prev_x_) / dt;
			vy = (y - prev_y_) / dt;
			vz = (z - prev_z_) / dt;
		}
		std::fprintf(truth_, "%llu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%llu\n",
			     static_cast<unsigned long long>(t), x, y, z, vx, vy, vz, relay_open_ ? 1 : 0,
			     chute_state_ ? 1 : 0, cut_.load(), gnss_on_ ? 1 : 0,
			     static_cast<unsigned long long>(t_hb_us_.load()));
		std::fflush(truth_);
		prev_t_ = t;
		prev_x_ = x;
		prev_y_ = y;
		prev_z_ = z;
		have_prev_ = true;
	}

	void on_frame(const fts_frame &f)
	{
		fts_output o;

		if (fts_decode_output(&f, &o) != 0) {
			return; /* STATUS also arrives on link 1; the bridge ignores it */
		}
		const bool relay = o.relay_open != 0U;
		if (relay && !relay_open_) {
			std::cerr << "fts_bridge: relay open at t_us " << t_us_.load() << "\n";
			relay_open_ = true;
			publish_motors(std::vector<double>(kMotors, 0.0));
		}
		relay_open_ = relay;
		if ((o.chute_fire != 0U) && !chute_cmd_) {
			gz::msgs::Boolean b;
			b.set_data(true);
			chute_pub_.Publish(b);
			chute_cmd_ = true;
			std::cerr << "fts_bridge: parachute fired at t_us " << t_us_.load() << "\n";
		}
	}

	void link_reader()
	{
		fts_parser p;
		fts_frame f;
		uint8_t buf[256];

		fts_parser_init(&p);
		for (;;) {
			const ssize_t n = ::read(fd_, buf, sizeof(buf));
			if (n < 0) {
				if (errno == EINTR) {
					continue;
				}
				std::cerr << "fts_bridge: link read failed: " << std::strerror(errno) << "\n";
				std::exit(1);
			}
			if (n == 0) {
				std::cerr << "fts_bridge: link closed\n";
				std::exit(1);
			}
			for (ssize_t i = 0; i < n; i++) {
				if (!fts_parser_feed(&p, buf[i], &f)) {
					continue;
				}
				do {
					on_frame(f);
				} while (fts_parser_next(&p, &f));
			}
		}
	}

	void mav_reader()
	{
		uint8_t buf[2048];
		uint32_t seq = 0;

		for (;;) {
			const ssize_t n = ::recv(udp_, buf, sizeof(buf), 0);
			if (n < 0) {
				if (errno == EINTR) {
					continue;
				}
				std::cerr << "fts_bridge: UDP receive failed: " << std::strerror(errno) << "\n";
				std::exit(1);
			}
			if (!mav_has_heartbeat(buf, static_cast<size_t>(n), kPx4SysId, kPx4CompId)) {
				continue;
			}
			const int drop = hb_drop_.load();
			if (drop > 0) {
				hb_drop_ = drop - 1;
				continue;
			}
			fts_ap_heartbeat h{};
			uint8_t out[FTS_FRAME_MAX];
			h.t_us = t_us_.load();
			h.seq = seq++;
			t_hb_us_ = h.t_us;
			send(out, fts_encode_ap_heartbeat(&h, out, sizeof(out)));
		}
	}

	Args args_;
	int fd_ = -1;
	int udp_ = -1;
	std::FILE *truth_ = nullptr;
	gz::transport::Node node_;
	gz::transport::Node::Publisher gated_pub_;
	gz::transport::Node::Publisher chute_pub_;
	std::mutex tx_mu_;
	std::mutex clock_mu_;
	std::mutex truth_mu_;
	std::atomic<uint64_t> t_us_{0};
	std::atomic<uint64_t> t_hb_us_{0};
	std::atomic<bool> relay_open_{false};
	std::atomic<bool> chute_cmd_{false};
	std::atomic<bool> chute_state_{false};
	std::atomic<bool> gnss_on_{true};
	std::atomic<bool> gnss_jump_{false};
	std::atomic<int> cut_{-1};
	std::atomic<int> hb_drop_{0};
	uint64_t next_readback_us_ = 0;
	uint64_t next_zero_us_ = 0;
	bool have_prev_ = false;
	uint64_t prev_t_ = 0;
	double prev_x_ = 0.0;
	double prev_y_ = 0.0;
	double prev_z_ = 0.0;
};

} // namespace

int main(int argc, char **argv)
{
	try {
		Bridge bridge(parse_args(argc, argv));
		bridge.start();
		std::cerr << "fts_bridge: running\n";
		gz::transport::waitForShutdown();
		bridge.close_truth();
	} catch (const std::exception &e) {
		std::cerr << "fts_bridge: " << e.what() << "\n";
		return 1;
	}
	/* Reader threads are blocked in read(); end the process without unwinding them. */
	std::_Exit(0);
}
```

Append to `tools/build-vm.sh`:

```bash

# I/O bridge
cmake -GNinja -S "$REPO/bridge" -B "$REPO/build/bridge"
ninja -C "$REPO/build/bridge"
ctest --test-dir "$REPO/build/bridge" --output-on-failure
```

Create `tests/sim/bridge_check.py`:

```python
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
```

- [ ] **Step 7: Build and check on the VM**

Run on the Mac:

```bash
tools/sync-vm.sh && ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && tools/build-vm.sh > /tmp/build.log 2>&1; grep -iE "warning|error|tests passed" /tmp/build.log | head;
export GZ_IP=127.0.0.1 GZ_SIM_RESOURCE_PATH=$PWD/sim/models:$PWD/sim/worlds:$HOME/PX4-Autopilot/Tools/simulation/gz/models GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/sim &&
(gz sim -r -s sim/worlds/fts_field.sdf > /tmp/gz-world.log 2>&1 & echo $! > /tmp/gz-world.pid) && sleep 12 && python3 tests/sim/bridge_check.py; kill $(cat /tmp/gz-world.pid)'
```

Expected: no warnings or errors; `100% tests passed, 0 tests failed out of 3`; rates about imu 100, gnss 10, baro 20, readback 10; truth rows over 100; `bridge check passed`.

- [ ] **Step 8: Commit**

```bash
git add bridge tools/build-vm.sh tests/sim/bridge_check.py
git commit -m "feat: add the FTS I/O bridge for Gazebo" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: ROS 2 ground station

**Files:**
- Create: `ros2_ws/src/fts_interfaces/package.xml`, `ros2_ws/src/fts_interfaces/CMakeLists.txt`, `ros2_ws/src/fts_interfaces/msg/FtsStatus.msg`, `ros2_ws/src/fts_ground/package.xml`, `ros2_ws/src/fts_ground/setup.py`, `ros2_ws/src/fts_ground/setup.cfg`, `ros2_ws/src/fts_ground/resource/fts_ground`, `ros2_ws/src/fts_ground/fts_ground/link.py`, `ros2_ws/src/fts_ground/fts_ground/ground_station.py`, `ros2_ws/src/fts_ground/test/test_link.py`
- Modify: `.gitignore`, `tools/build-vm.sh`

**Interfaces:**
- Consumes: `fts_ground.proto` (Task 1).
- Produces:
  - message `fts_interfaces/msg/FtsStatus` (`t_us`, `state`, `cause`, `pbit_fail`, `t_trigger_us`, `t_relay_us`, `t_chute_us`, `t_last_cmd_us`, `link_enabled`; times are -1 when unset);
  - `fts_ground.link.GroundLink` with `enabled`, `seq`, `last_status`, `t_last_cmd_us`, `command(cmd) -> bytes`, `terminate() -> bytes`, `on_bytes(data) -> list[proto.Status]`;
  - node `fts_ground_station` (executable `ground_station`, parameter `link` = PTY path): publishes `/fts/status`; services `/fts_ground_station/arm`, `/disarm`, `/terminate` (`std_srvs/srv/Trigger`), `/fts_ground_station/link` (`std_srvs/srv/SetBool`, false stops all sending); sends PING at 2 Hz.

`t_last_cmd_us` is the FTS time of the newest STATUS when the last command was sent; the runner uses it as the fault time for the link-loss and manual scenarios.

- [ ] **Step 1: Write the failing test**

Create `ros2_ws/src/fts_ground/test/test_link.py`:

```python
from fts_ground import proto
from fts_ground.link import GroundLink

# STATUS from the C encoder: t_us 12 s, TERMINATED, FENCE (see test_proto.py).
STATUS_FRAME = bytes.fromhex(
    "a50823001bb70000000000030100e079af0000000000e079af0000000000"
    "ffffffffffffffffa287")


def frames(data: bytes) -> list[tuple[int, int, int]]:
    """(type, seq, cmd) for each GS_CMD frame in data."""
    out = []
    for msg_type, payload in proto.Parser().feed(data):
        assert msg_type == proto.MSG_GS_CMD
        out.append((msg_type, int.from_bytes(payload[:4], "little"), payload[4]))
    return out


def test_commands_have_increasing_sequence_numbers():
    link = GroundLink()
    data = link.command(proto.CMD_PING) + link.command(proto.CMD_ARM)
    assert frames(data) == [(proto.MSG_GS_CMD, 1, proto.CMD_PING), (proto.MSG_GS_CMD, 2, proto.CMD_ARM)]


def test_terminate_sends_both_steps_in_order():
    link = GroundLink()
    assert [f[2] for f in frames(link.terminate())] == [proto.CMD_TERMINATE_ARM, proto.CMD_TERMINATE]


def test_disabled_link_sends_nothing():
    link = GroundLink()
    link.enabled = False
    assert link.command(proto.CMD_PING) == b""
    assert link.terminate() == b""
    assert link.t_last_cmd_us is None


def test_status_split_across_reads():
    link = GroundLink()
    assert link.on_bytes(STATUS_FRAME[:10]) == []
    statuses = link.on_bytes(STATUS_FRAME[10:])
    assert len(statuses) == 1 and statuses[0].state == "TERMINATED"
    assert link.last_status == statuses[0]


def test_command_time_is_newest_status_time():
    link = GroundLink()
    link.command(proto.CMD_PING)
    assert link.t_last_cmd_us is None
    link.on_bytes(STATUS_FRAME)
    link.command(proto.CMD_PING)
    assert link.t_last_cmd_us == 12_000_000
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest ros2_ws/src/fts_ground/test -q`
Expected: collection error, `ModuleNotFoundError: No module named 'fts_ground.link'`.

- [ ] **Step 3: Implement**

Create `ros2_ws/src/fts_ground/fts_ground/link.py`:

```python
"""Ground station side of link 2, without ROS."""

from fts_ground import proto


class GroundLink:
    def __init__(self) -> None:
        self.parser = proto.Parser()
        self.seq = 0
        self.enabled = True
        self.last_status: proto.Status | None = None
        self.t_last_cmd_us: int | None = None

    def command(self, cmd: int) -> bytes:
        """Frame for one command, or nothing while the link is disabled."""
        if not self.enabled:
            return b""
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        if self.last_status is not None:
            self.t_last_cmd_us = self.last_status.t_us
        return proto.encode_gs_cmd(self.seq, cmd)

    def terminate(self) -> bytes:
        return self.command(proto.CMD_TERMINATE_ARM) + self.command(proto.CMD_TERMINATE)

    def on_bytes(self, data: bytes) -> list[proto.Status]:
        statuses = []
        for msg_type, payload in self.parser.feed(data):
            if msg_type == proto.MSG_STATUS and len(payload) == proto.STATUS_LEN:
                self.last_status = proto.decode_status(payload)
                statuses.append(self.last_status)
        return statuses
```

Create `ros2_ws/src/fts_ground/fts_ground/ground_station.py`:

```python
"""FTS ground station: link 2 over a PTY, as a ROS 2 topic and services."""

import os
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from fts_ground import proto
from fts_ground.link import GroundLink
from fts_interfaces.msg import FtsStatus

PING_PERIOD_S = 0.5
POLL_PERIOD_S = 0.01


def _us(value: int | None) -> int:
    return -1 if value is None else value


class GroundStation(Node):
    def __init__(self) -> None:
        super().__init__("fts_ground_station")
        path = self.declare_parameter("link", "").get_parameter_value().string_value
        if not path:
            raise RuntimeError("parameter 'link' (PTY path) is required")
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        self.link = GroundLink()
        self.state = ""
        self.pub = self.create_publisher(FtsStatus, "fts/status", 10)
        self.create_timer(POLL_PERIOD_S, self.poll)
        self.create_timer(PING_PERIOD_S, lambda: self.write(self.link.command(proto.CMD_PING)))
        self.create_service(Trigger, "~/arm", self.command_service(proto.CMD_ARM))
        self.create_service(Trigger, "~/disarm", self.command_service(proto.CMD_DISARM))
        self.create_service(Trigger, "~/terminate", self.on_terminate)
        self.create_service(SetBool, "~/link", self.on_link)
        self.get_logger().info(f"link 2 on {path}")

    def write(self, data: bytes) -> None:
        if data:
            os.write(self.fd, data)

    def reply(self, resp):
        resp.success = self.link.enabled
        resp.message = "sent" if self.link.enabled else "link disabled"
        return resp

    def command_service(self, cmd: int):
        def handle(_req, resp):
            self.write(self.link.command(cmd))
            return self.reply(resp)
        return handle

    def on_terminate(self, _req, resp):
        self.write(self.link.terminate())
        return self.reply(resp)

    def on_link(self, req, resp):
        self.link.enabled = req.data
        resp.success = True
        resp.message = "link enabled" if req.data else "link disabled"
        self.get_logger().info(resp.message)
        return resp

    def poll(self) -> None:
        while True:
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                return
            if not data:
                raise RuntimeError("link 2 closed")
            for st in self.link.on_bytes(data):
                if st.state != self.state:
                    self.get_logger().info(f"FTS {st.state} cause {st.cause}")
                    self.state = st.state
                self.pub.publish(self.to_msg(st))

    def to_msg(self, st: proto.Status) -> FtsStatus:
        msg = FtsStatus()
        msg.t_us = st.t_us
        msg.state = st.state
        msg.cause = st.cause
        msg.pbit_fail = st.pbit_fail
        msg.t_trigger_us = _us(st.t_trigger_us)
        msg.t_relay_us = _us(st.t_relay_us)
        msg.t_chute_us = _us(st.t_chute_us)
        msg.t_last_cmd_us = _us(self.link.t_last_cmd_us)
        msg.link_enabled = self.link.enabled
        return msg


def main() -> None:
    rclpy.init()
    node = GroundStation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

Create `ros2_ws/src/fts_interfaces/msg/FtsStatus.msg`:

```
# FTS status as reported on link 2. Times are simulation time in
# microseconds; -1 means not set.
uint64 t_us
string state
string cause
uint8 pbit_fail
int64 t_trigger_us
int64 t_relay_us
int64 t_chute_us
# FTS time of the newest status when the last command was sent
int64 t_last_cmd_us
bool link_enabled
```

Create `ros2_ws/src/fts_interfaces/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(fts_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/FtsStatus.msg")

ament_package()
```

Create `ros2_ws/src/fts_interfaces/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>fts_interfaces</name>
  <version>0.1.0</version>
  <description>Messages for the simulated flight termination system.</description>
  <maintainer email="contact@alpentalsystems.com">Alpental Systems</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Create `ros2_ws/src/fts_ground/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>fts_ground</name>
  <version>0.1.0</version>
  <description>Ground station and scenario runner for the simulated flight termination system.</description>
  <maintainer email="contact@alpentalsystems.com">Alpental Systems</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>fts_interfaces</exec_depend>
  <test_depend>python3-pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Create `ros2_ws/src/fts_ground/setup.py`:

```python
from setuptools import setup

PACKAGE = "fts_ground"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alpental Systems",
    maintainer_email="contact@alpentalsystems.com",
    description="Ground station and scenario runner for the simulated flight termination system.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ground_station = fts_ground.ground_station:main",
        ],
    },
)
```

Create `ros2_ws/src/fts_ground/setup.cfg`:

```
[develop]
script_dir=$base/lib/fts_ground
[install]
install_scripts=$base/lib/fts_ground
```

Create `ros2_ws/src/fts_ground/resource/fts_ground` (empty file).

Append to `.gitignore`:

```
ros2_ws/install/
ros2_ws/log/
.pytest_cache/
```

Append to `tools/build-vm.sh`:

```bash

# ROS 2 packages
set +u
source /opt/ros/humble/setup.bash
set -u
(cd "$REPO/ros2_ws" && colcon build --symlink-install)
```

- [ ] **Step 4: Run the tests; build on the VM**

Run: `python3 -m pytest ros2_ws/src/fts_ground/test -q`
Expected: `12 passed`.

Run on the Mac: `tools/sync-vm.sh && ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && tools/build-vm.sh > /tmp/build.log 2>&1; tail -3 /tmp/build.log; source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash && ros2 interface show fts_interfaces/msg/FtsStatus | head -3 && ls ros2_ws/install/fts_ground/lib/fts_ground/'`
Expected: `Summary: 2 packages finished`; the message definition prints; `ground_station` is listed.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src .gitignore tools/build-vm.sh
git commit -m "feat: add the ROS 2 ground station" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 8: Launch scripts and bring-up on the ground

**Files:**
- Create: `tools/run-sim.sh`, `tools/stop-sim.sh`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: everything from Tasks 3 to 7.
- Produces: `tools/run-sim.sh [--gui]` starts Gazebo, the firmware, the bridge, PX4 (attached to `x500_fts_0`, heartbeat at 10 Hz) and the ground station; logs go to `run/<name>.log`, ground truth to `run/truth.csv`, process IDs to `run/pids` (`<name> <pid>` per line), PTY paths to `run/ptys`. `tools/stop-sim.sh` stops them (it sends SIGCONT first, for a frozen PX4).

- [ ] **Step 1: Write the scripts**

Create `tools/run-sim.sh`:

```bash
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
```

Create `tools/stop-sim.sh`:

```bash
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
```

Run `chmod +x tools/run-sim.sh tools/stop-sim.sh`.

Append to `.gitignore`:

```
run/
runs/
```

- [ ] **Step 2: Bring up and check on the ground**

Run on the Mac:

```bash
tools/sync-vm.sh && ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && tools/run-sim.sh && sleep 5 &&
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash &&
grep "^FTS" run/fts.log && ros2 topic echo --once /fts/status | grep -E "state|cause" &&
ros2 service call /fts_ground_station/arm std_srvs/srv/Trigger > /dev/null && sleep 1 && ros2 topic echo --once /fts/status | grep state'
```

Expected: `FTS started`, `FTS PBIT -> SAFE`; status `state: SAFE`, `cause: NONE`; after the ARM call `state: ARMED` (this proves GNSS, baro, heartbeat and link 2 all reach the FTS).

Then terminate on the ground and stop:

```bash
ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash &&
ros2 service call /fts_ground_station/terminate std_srvs/srv/Trigger > /dev/null && sleep 2 &&
ros2 topic echo --once /fts/status | grep -E "state|cause|t_relay|t_chute"; grep -E "relay open|parachute fired" run/bridge.log;
GZ_IP=127.0.0.1 gz topic -e -n 1 -t /fts/chute_state; tail -2 run/truth.csv; tools/stop-sim.sh'
```

Expected: `state: TERMINATED`, `cause: MANUAL`, `t_chute_us` 300000 above `t_relay_us`; the bridge log shows `relay open` and `parachute fired`; `/fts/chute_state` is `data: true`; the last truth rows have `relay_open` 1 and `chute_fired` 1; all processes stopped.

- [ ] **Step 3: Commit**

```bash
git add tools/run-sim.sh tools/stop-sim.sh .gitignore
git commit -m "feat: add scripts to start and stop the whole simulation" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 9: Metrics and scenario runner

**Files:**
- Create: `ros2_ws/src/fts_ground/fts_ground/metrics.py`, `ros2_ws/src/fts_ground/test/test_metrics.py`, `ros2_ws/src/fts_ground/fts_ground/runner.py`, `tools/run-scenarios.sh`, `tools/plot_run.py`, `tools/summarize_runs.py`
- Modify: `ros2_ws/src/fts_ground/setup.py`, `ros2_ws/src/fts_ground/package.xml`

**Interfaces:**
- Consumes: `/fts/status` and the ground station services (Task 7); `run/truth.csv`, `run/pids` (Task 8); `/fts/fault` (Task 6).
- Produces:
  - `fts_ground.metrics`: `FENCE_HALF_M` (100), `CEILING_M` (40), `Row`, `read_truth(path) -> list[Row]`, `fault_time(scenario, rows, t_cmd_us, t_trigger_us) -> int | None`, `summarize(scenario, rows, status: dict, t_cmd_us: int) -> dict`;
  - executable `runner SCENARIO --out DIR --repo DIR` with scenarios `fence`, `ceiling`, `control`, `freeze`, `link`, `gnss`, `manual` (expect termination with the matching cause) and `near_fence`, `gnss_jump`, `hb_drop` (expect no termination); writes `DIR/result.json` and exits 1 if the scenario fails;
  - `tools/run-scenarios.sh [--gui] SCENARIO...` (fresh simulation per scenario; results, logs, bag and plots in `runs/<scenario>/`);
  - `tools/plot_run.py RUN_DIR` (`altitude.png`, `track.png`); `tools/summarize_runs.py [RUNS_DIR]` (markdown table).

Fault time per scenario: `fence` first truth row outside the fence; `ceiling` first row more than 40 m above the first row; `control` first row with a cut motor; `gnss` first row with GNSS off; `freeze` the last heartbeat time recorded at the trigger; `link` and `manual` the ground station's `t_last_cmd_us`.

- [ ] **Step 1: Write the failing test**

Create `ros2_ws/src/fts_ground/test/test_metrics.py`:

```python
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
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest ros2_ws/src/fts_ground/test -q`
Expected: collection error, `cannot import name 'metrics' from 'fts_ground'`.

- [ ] **Step 3: Implement the metrics**

Create `ros2_ws/src/fts_ground/fts_ground/metrics.py`:

```python
"""Scenario metrics from the bridge's ground-truth CSV and the final FTS status."""

import csv
import math
from dataclasses import dataclass
from pathlib import Path

FENCE_HALF_M = 100.0
CEILING_M = 40.0
LANDED_ABOVE_REST_M = 0.3
LANDED_MAX_VZ_M_S = 0.5
CHUTE_SETTLE_US = 3_000_000
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
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -m pytest ros2_ws/src/fts_ground/test -q`
Expected: `17 passed`.

- [ ] **Step 5: Runner and scripts**

Create `ros2_ws/src/fts_ground/fts_ground/runner.py`:

```python
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

    def _ack(self, cmd: int) -> int:
        end = time.time() + 5.0
        while time.time() < end:
            ack = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
            if ack is not None and ack.command == cmd:
                return ack.result
        raise RuntimeError(f"no ack for command {cmd}")

    def command(self, cmd: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)
        result = self._ack(cmd)
        if result != MAV.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"command {cmd} rejected: result {result}")

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
        self.m.mav.command_int_send(self.m.target_system, self.m.target_component,
                                    MAV.MAV_FRAME_GLOBAL, MAV.MAV_CMD_DO_REPOSITION, 0, 0,
                                    REPOSITION_SPEED_M_S, 1.0, 0, NAN,
                                    int(lat * 1e7), int(lon * 1e7), home_amsl + rel_alt)
        result = self._ack(MAV.MAV_CMD_DO_REPOSITION)
        if result != MAV.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"reposition rejected: result {result}")


class Runner(Node):
    def __init__(self) -> None:
        super().__init__("fts_runner")
        self.status: FtsStatus | None = None
        self.history: list[dict] = []
        self.create_subscription(FtsStatus, "/fts/status", self.on_status, 10)

    def on_status(self, msg: FtsStatus) -> None:
        if self.status is None or msg.state != self.status.state:
            self.history.append({"t_us": msg.t_us, "state": msg.state, "cause": msg.cause})
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


def fault(name: str) -> None:
    env = dict(os.environ, GZ_IP="127.0.0.1")
    subprocess.run(["gz", "topic", "-t", "/fts/fault", "-m", "gz.msgs.StringMsg",
                    "-p", f'data: "{name}"'], check=True, env=env)


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
            fault("cut_motor:0")
        elif scenario == "freeze":
            os.kill(px4_pid(repo), signal.SIGSTOP)
        elif scenario == "link":
            node.set_link(False)
        elif scenario == "gnss":
            fault("gnss_off")
        elif scenario == "manual":
            node.trigger("terminate")
        elif scenario == "near_fence":
            px4.reposition(home, 90.0, 0.0, TAKEOFF_M)
            px4.wait(lambda e, n, rel: e > 85.0, 60, "flight near the fence")
        elif scenario == "gnss_jump":
            fault("gnss_jump")
        elif scenario == "hb_drop":
            fault("hb_drop:5")

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
    result = metrics.summarize(scenario, rows, status, st.t_last_cmd_us)
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
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        result = run(args.scenario, node, args.repo)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "status_history"}, indent=2))
    if not result["pass"]:
        raise SystemExit(1)
```

In `ros2_ws/src/fts_ground/setup.py`, change the console scripts to:

```python
        "console_scripts": [
            "ground_station = fts_ground.ground_station:main",
            "runner = fts_ground.runner:main",
        ],
```

In `ros2_ws/src/fts_ground/package.xml`, after `<exec_depend>fts_interfaces</exec_depend>` add:

```xml
  <exec_depend>python3-pymavlink</exec_depend>
```

Create `tools/run-scenarios.sh`:

```bash
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
		python3 "$REPO/tools/plot_run.py" "$out"
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
```

The runner's exit status passes through `tee` because of `set -o pipefail`.

Create `tools/plot_run.py`:

```python
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
```

Create `tools/summarize_runs.py`:

```python
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
```

Run `chmod +x tools/run-scenarios.sh tools/plot_run.py tools/summarize_runs.py`.

- [ ] **Step 6: First scenario on the VM**

Run on the Mac: `tools/sync-vm.sh && ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && tools/build-vm.sh > /tmp/build.log 2>&1; tail -1 /tmp/build.log; tools/run-scenarios.sh manual 2>&1 | tail -25; ls runs/manual'`
Expected: the runner prints `FTS ARMED`, then `FTS TERMINATED cause MANUAL`; the JSON shows `"pass": true`, `"fault_to_trigger_ms"` between 0 and 150, `"fault_to_chute_ms"` about 300 more than `"fault_to_relay_ms"`, `"landed": true`, `"vz_under_chute_m_s"` between -6 and -4; `scenario manual passed`; `runs/manual` holds `result.json`, `truth.csv`, the logs, `bag`, `altitude.png`, `track.png`.

- [ ] **Step 7: Commit**

```bash
git add ros2_ws/src/fts_ground tools
git commit -m "feat: add the scenario runner and run metrics" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 10: Scenario runs and test log

**Files:**
- Create: `tools/fetch-runs.sh`, `docs/test-log.md`, `docs/img/` (selected plots)
- Modify: `README.md`

- [ ] **Step 1: Run every scenario**

Run on the Mac: `ssh -i ~/.ssh/id_ed25519_vm user@vm-host 'cd fts-sim && tools/run-scenarios.sh fence ceiling control freeze link gnss manual near_fence gnss_jump hb_drop > /tmp/scenarios.log 2>&1; grep -E "^scenario|^failed" /tmp/scenarios.log; python3 tools/summarize_runs.py'`

Expected: every line reads `scenario <name> passed`, and the table shows, per scenario:

| Scenario | Cause | Fault to trigger |
|---|---|---|
| fence | FENCE | 500 to 700 ms |
| ceiling | CEILING | 500 to 700 ms |
| control | CONTROL | 500 to 1500 ms (the drone must first tilt past 60 degrees) |
| freeze | AP_FREEZE | 1000 to 1150 ms |
| link | LINK | 5000 to 5600 ms (PING every 0.5 s) |
| gnss | GNSS_LOST | 1000 to 1150 ms |
| manual | MANUAL | 0 to 150 ms |
| near_fence, gnss_jump, hb_drop | NONE (not terminated) | - |

For every terminated run, fault to relay equals fault to trigger, fault to parachute is 300 ms more, the drone lands, and the descent under the parachute is between -6 and -4 m/s. In `freeze`, the drone must fall after the trigger (Review Focus 1). If a scenario fails, find the cause from `runs/<name>/*.log` with superpowers:systematic-debugging before changing any threshold.

- [ ] **Step 2: Fetch the runs and write the test log**

Create `tools/fetch-runs.sh`:

```bash
#!/usr/bin/env bash
# Copies runs/ from the VM to the Mac.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
VM=${FTS_VM:-user@vm-host}
KEY=${FTS_VM_KEY:-$HOME/.ssh/id_ed25519_vm}
rsync -a --delete -e "ssh -i $KEY" "$VM:fts-sim/runs/" "$REPO/runs/"
```

Run `chmod +x tools/fetch-runs.sh && tools/fetch-runs.sh && python3 tools/summarize_runs.py && mkdir -p docs/img && for s in fence control freeze; do cp runs/$s/altitude.png docs/img/$s-altitude.png; done && cp runs/fence/track.png docs/img/fence-track.png`.

Create `docs/test-log.md` with these sections, filled from the runs:

1. **Setup:** date; VM (Ubuntu 22.04 aarch64, 8 cores); PX4 v1.15.4; Gazebo 8.15; Zephyr v4.4.2 `native_sim/native/64`; FTS demo configuration (fence +-100 m, ceiling 40 m, thresholds from the spec); parachute (Cd 1.5, area 0.85 m2, 2 kg, wind 3 m/s east).
2. **How to reproduce:** `tools/sync-vm.sh`, `tools/build-vm.sh`, `tools/run-scenarios.sh <names>`, `tools/fetch-runs.sh`, `tools/summarize_runs.py`.
3. **Results:** the table printed by `tools/summarize_runs.py`, pasted as is.
4. **Notes per scenario:** one or two sentences each on what happened, taken from `runs/<name>/runner.log`, `fts.log` and `bridge.log` (for example the tilt build-up in `control`, the fall in `freeze`).
5. **Plots:** `img/fence-altitude.png`, `img/fence-track.png`, `img/control-altitude.png`, `img/freeze-altitude.png`.
6. **Limits:** simulation only; the FTS's own IMU and barometer faults are out of scope; wind acts on the parachute only.

- [ ] **Step 3: README**

Replace the `## Host tests` section of `README.md` with:

```markdown
## Host tests (Mac or VM)

```sh
cmake -S tests/host -B build/host && cmake --build build/host && ctest --test-dir build/host
cmake -S bridge -B build/bridge-host -DFTS_BRIDGE_GZ=OFF && cmake --build build/bridge-host && ctest --test-dir build/bridge-host
python3 -m pytest ros2_ws/src/fts_ground/test
```

## Simulation (VM with PX4, Gazebo 8, ROS 2 Humble)

```sh
tools/sync-vm.sh                       # Mac: copy the tree to ~/fts-sim on the VM
tools/setup-vm.sh                      # VM, once: Zephyr workspace
tools/build-vm.sh                      # VM: firmware, plugin, bridge, ROS 2 packages
tools/run-scenarios.sh fence manual    # VM: one fresh simulation per scenario
```

Results: [docs/test-log.md](docs/test-log.md).
```

- [ ] **Step 4: Screenshots (owner)**

The owner runs `tools/run-scenarios.sh --gui fence` and `tools/run-scenarios.sh --gui control` from a terminal on the VM desktop and takes screenshots of the Gazebo window: the drone near the fence posts, and the canopy during the descent. Save them as `docs/img/fence-gazebo.png` and `docs/img/control-gazebo.png`, then add them to section 5 of the test log.

- [ ] **Step 5: Commit**

```bash
git add tools/fetch-runs.sh docs/test-log.md docs/img README.md
git commit -m "docs: add the scenario test log" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

## After this plan

The Korean blog post (with `summary_en`), the public repo under `alpentalsystems`, and a `part1` tag follow the same steps as the redundant controller series, after the owner has reviewed the test log and screenshots.
