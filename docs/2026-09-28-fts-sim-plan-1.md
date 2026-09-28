# FTS Simulation Plan 1: Feasibility and Core Logic

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Confirm that PX4 SITL flies in Gazebo on the owner's ARM64 Ubuntu VM, and build the host-tested core of the FTS: the frame protocol, fence geometry, triggers, state machine, and self-test.

**Architecture:** Pure C modules with no platform dependencies, tested on the host with CMake/CTest, the same pattern as the redundant controller series. Plan 2 (written after Task 0) adds the Zephyr firmware glue, the Gazebo I/O bridge, the drone model and parachute, the ROS 2 nodes, and the scenario tests, using the facts Task 0 records.

**Tech Stack:** C11 (`-Wall -Wextra -Werror`), CMake/CTest; PX4 SITL, Gazebo `gz sim` 8, ROS 2 Humble on the VM for Task 0.

**Spec:** `docs/2026-09-28-fts-sim-design.md`

## Global Constraints

- Public references only; nothing from any former employer's design in code, comments, docs, or commit messages.
- Times are simulation time in microseconds (`uint64_t` on the wire, `int64_t` in logic).
- Thresholds (spec): fence and ceiling confirmation 0.5 s; tilt 60° or rate 300°/s for 0.5 s; heartbeat timeout 1.0 s; link timeout 5 s; manual terminate window 3 s; parachute 0.3 s after the relay.
- Frame format: sync `0xA5`, type, len, payload (little-endian), CRC-16/CCITT-FALSE over type, len, and payload, low byte first; max payload 48.
- Kernel-style tabs; comments short, English, plain ASCII.
- VM: `user@vm-host`, key `~/.ssh/id_ed25519_vm`. `sudo` only if passwordless sudo is set up; otherwise the owner runs those commands.

## Review Focus

1. **Time going backwards or jumping** (Gazebo pause, reset): confirmation timers must not underflow or fire early.
2. **Fence edge cases:** point exactly on an edge or vertex, concave polygons, and very large coordinates must give a stable inside/outside answer.
3. **Trigger before PBIT or while SAFE:** no termination outside `ARMED`.
4. **Manual terminate replay:** a stale `TERMINATE_ARM` (older than 3 s) followed by `TERMINATE` must not terminate.
5. **Latching:** once `TERMINATED`, no input (ARM, DISARM, recovery of all conditions) may close the relay or reset the parachute.

---

### Task 0: Feasibility check on the VM (throwaway)

**Output:** findings recorded in `docs/feasibility.md`; nothing built here is kept in the repo.

- [ ] **Step 1: Prerequisites**

Check on the VM: `sudo -n true` (passwordless sudo), free disk (`df -h /`), and `gz sim --versions`. If sudo needs a password, ask the owner to run the install commands in Step 2.

- [ ] **Step 2: Get and build PX4 SITL**

On the VM:
```bash
cd ~ && git clone --recursive --depth 1 --shallow-submodules -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git
bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
cd PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500
```
Record: build success or the first error, build time, disk used. If `v1.15.4` does not work with the installed `gz sim` 8, try the newest v1.16 tag and record which one works.

- [ ] **Step 3: Fly**

In the PX4 shell: `commander takeoff`, wait 10 s, `commander land`. Record: real-time factor (`gz topic -e -t /stats` or the GUI), whether the takeoff reached about 2.5 m, CPU load (`top`).

- [ ] **Step 4: Facts plan 2 needs**

Record:
- the Gazebo topic PX4 publishes motor commands on (`gz topic -l | grep -i motor`) and its message type (`gz topic -i -t <topic>`);
- the world name and the x500 model name;
- whether PX4's MAVLink heartbeat is visible on UDP 14550 or 14540 (`sudo tcpdump -i lo udp port 14550 -c 5` or a Python socket);
- the uXRCE-DDS agent: install (`sudo snap install micro-xrce-dds-agent --edge`, or build from source) and whether `ros2 topic list` shows `/fmu/out/...` topics after `MicroXRCEAgent udp4 -p 8888`.

- [ ] **Step 5: Decide**

Write `docs/feasibility.md` with the findings and one of: **go** (PX4 SITL works; plan 2 uses the recorded facts), **fallback** (switch to ArduPilot SITL; only the heartbeat source and flight commands change), or **stop** (report to the owner). Commit:

```bash
git add docs/feasibility.md
git commit -m "docs: record PX4 SITL feasibility on the VM" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 1: Repository scaffold and host test harness

**Files:**
- Create: `.gitignore`, `README.md`, `tests/host/CMakeLists.txt`, `tests/host/check.h`, `tests/host/test_smoke.c`

- [ ] **Step 1: Files**

`.gitignore`:
```
build/
__pycache__/
```

`README.md`:
```markdown
# Simulated drone flight termination system

A flight termination system (FTS) for a small drone that watches the flight
independently of the autopilot and, on a fault, cuts motor power and deploys
a parachute. Everything runs in simulation: PX4 SITL and Gazebo for the
drone, the FTS as Zephyr firmware on native_sim, ROS 2 for the ground
station and tests.

Design: [docs/2026-09-28-fts-sim-design.md](docs/2026-09-28-fts-sim-design.md)

## Host tests

```sh
cmake -S tests/host -B build/host && cmake --build build/host && ctest --test-dir build/host
```
```

`tests/host/check.h`:
```c
#ifndef CHECK_H_
#define CHECK_H_

#include <stdio.h>

static int failures;

#define CHECK(cond)                                                            \
	do {                                                                   \
		if (!(cond)) {                                                 \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                            \
		}                                                              \
	} while (0)

#define CHECK_DONE()                                                           \
	((failures == 0) ? (printf("all checks passed\n"), 0)                 \
			 : (printf("%d check(s) failed\n", failures), 1))

#endif /* CHECK_H_ */
```

`tests/host/test_smoke.c`:
```c
#include "check.h"

int main(void)
{
	CHECK(1 + 1 == 2);
	return CHECK_DONE();
}
```

`tests/host/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.20)
project(fts_host_tests C)

enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_executable(test_smoke test_smoke.c)
add_test(NAME smoke COMMAND test_smoke)
```

- [ ] **Step 2: Run**

Run: `cmake -S tests/host -B build/host >/dev/null && cmake --build build/host >/dev/null && ctest --test-dir build/host`
Expected: `100% tests passed, 0 tests failed out of 1`.

- [ ] **Step 3: Commit**

```bash
git add .gitignore README.md tests/host
git commit -m "build: add host test harness" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: Frame protocol

**Files:**
- Create: `common/include/fts/proto.h`, `common/proto.c`, `tests/host/test_proto.c`
- Modify: `tests/host/CMakeLists.txt`

**Interfaces:**
- Produces: `FTS_MSG_*` (IMU 1, GNSS 2, BARO 3, AP_HEARTBEAT 4, GS_CMD 5, OUTPUT 6, READBACK 7, STATUS 8), `FTS_CMD_*` (PING 0, ARM 1, DISARM 2, TERMINATE_ARM 3, TERMINATE 4); `fts_crc16`, `fts_frame_encode`, `struct fts_parser` with `fts_parser_init/feed/next`; message structs and `fts_encode_<msg>` / `fts_decode_<msg>` (0 or -1) for each message.

Payload layouts (little-endian, floats as IEEE-754 binary32):

| Message | Payload |
|---|---|
| IMU (48) | t_us u64, q w,x,y,z f32, gyro x,y,z f32 (rad/s), accel x,y,z f32 (m/s²) |
| GNSS (22) | t_us u64, lat_e7 i32, lon_e7 i32, alt_mm i32, fix u8, sats u8 |
| BARO (16) | t_us u64, pressure_pa f32, temp_c f32 |
| AP_HEARTBEAT (12) | t_us u64, seq u32 |
| GS_CMD (5) | seq u32, cmd u8 |
| OUTPUT (2) | relay_open u8, chute_fire u8 |
| READBACK (10) | t_us u64, relay_open u8, chute_fired u8 |
| STATUS (35) | t_us u64, state u8, cause u8, pbit_fail u8, t_trigger_us u64, t_relay_us u64, t_chute_us u64 |

- [ ] **Step 1: Write the failing test**

Create `tests/host/test_proto.c`:

```c
#include <string.h>

#include "check.h"
#include "fts/proto.h"

static size_t feed_all(struct fts_parser *p, const uint8_t *buf, size_t n,
		       struct fts_frame *frames, size_t max_frames)
{
	size_t count = 0U;

	for (size_t i = 0; i < n; i++) {
		struct fts_frame f;

		if (!fts_parser_feed(p, buf[i], &f)) {
			continue;
		}
		do {
			if (count < max_frames) {
				frames[count++] = f;
			}
		} while (fts_parser_next(p, &f));
	}
	return count;
}

static bool decode_one(const uint8_t *buf, size_t n, struct fts_frame *f)
{
	struct fts_parser p;

	fts_parser_init(&p);
	return feed_all(&p, buf, n, f, 1U) == 1U;
}

static void test_crc16_check_value(void)
{
	const char *s = "123456789";

	CHECK(fts_crc16((const uint8_t *)s, strlen(s)) == 0x29B1U);
}

static void test_imu_roundtrip(void)
{
	struct fts_imu in = {.t_us = 0x0102030405060708ULL, .q = {1.0f, 0.0f, 0.0f, 0.0f},
			     .gyro = {0.1f, -0.2f, 0.3f}, .accel = {0.0f, 0.0f, -9.81f}};
	struct fts_imu out;
	uint8_t buf[FTS_FRAME_MAX];
	struct fts_frame f;
	size_t n = fts_encode_imu(&in, buf, sizeof(buf));

	memset(&out, 0, sizeof(out));
	CHECK(n == FTS_FRAME_OVERHEAD + 48U);
	CHECK(buf[3] == 0x08U); /* little-endian t_us */
	CHECK(decode_one(buf, n, &f));
	CHECK(fts_decode_imu(&f, &out) == 0);
	CHECK(out.t_us == in.t_us && out.q[0] == 1.0f && out.gyro[1] == -0.2f);
	CHECK(out.accel[2] == -9.81f);
}

static void test_gnss_baro_heartbeat_roundtrip(void)
{
	struct fts_gnss g = {.t_us = 5U, .lat_e7 = 375665000, .lon_e7 = 1269780000,
			     .alt_mm = -1234, .fix = 3U, .sats = 12U};
	struct fts_baro b = {.t_us = 6U, .pressure_pa = 101325.0f, .temp_c = 15.0f};
	struct fts_ap_heartbeat h = {.t_us = 7U, .seq = 0xDEADBEEFU};
	struct fts_gnss g2;
	struct fts_baro b2;
	struct fts_ap_heartbeat h2;
	uint8_t buf[FTS_FRAME_MAX];
	struct fts_frame f;
	size_t n;

	n = fts_encode_gnss(&g, buf, sizeof(buf));
	CHECK(n == FTS_FRAME_OVERHEAD + 22U && decode_one(buf, n, &f) && fts_decode_gnss(&f, &g2) == 0);
	CHECK(g2.lat_e7 == 375665000 && g2.lon_e7 == 1269780000 && g2.alt_mm == -1234);
	CHECK(g2.fix == 3U && g2.sats == 12U);
	n = fts_encode_baro(&b, buf, sizeof(buf));
	CHECK(n == FTS_FRAME_OVERHEAD + 16U && decode_one(buf, n, &f) && fts_decode_baro(&f, &b2) == 0);
	CHECK(b2.pressure_pa == 101325.0f && b2.temp_c == 15.0f);
	n = fts_encode_ap_heartbeat(&h, buf, sizeof(buf));
	CHECK(n == FTS_FRAME_OVERHEAD + 12U && decode_one(buf, n, &f));
	CHECK(fts_decode_ap_heartbeat(&f, &h2) == 0 && h2.seq == 0xDEADBEEFU && h2.t_us == 7U);
}

static void test_command_output_readback_status_roundtrip(void)
{
	struct fts_gs_cmd c = {.seq = 9U, .cmd = FTS_CMD_TERMINATE};
	struct fts_output o = {.relay_open = 1U, .chute_fire = 0U};
	struct fts_readback r = {.t_us = 11U, .relay_open = 1U, .chute_fired = 1U};
	struct fts_status s = {.t_us = 12U, .state = 3U, .cause = 2U, .pbit_fail = 0x40U,
			       .t_trigger_us = 100U, .t_relay_us = 101U, .t_chute_us = 400U};
	struct fts_gs_cmd c2;
	struct fts_output o2;
	struct fts_readback r2;
	struct fts_status s2;
	uint8_t buf[FTS_FRAME_MAX];
	struct fts_frame f;
	size_t n;

	n = fts_encode_gs_cmd(&c, buf, sizeof(buf));
	CHECK(n == FTS_FRAME_OVERHEAD + 5U && decode_one(buf, n, &f) && fts_decode_gs_cmd(&f, &c2) == 0);
	CHECK(c2.seq == 9U && c2.cmd == FTS_CMD_TERMINATE);
	n = fts_encode_output(&o, buf, sizeof(buf));
	CHECK(n == FTS_FRAME_OVERHEAD + 2U && decode_one(buf, n, &f) && fts_decode_output(&f, &o2) == 0);
	CHECK(o2.relay_open == 1U && o2.chute_fire == 0U);
	n = fts_encode_readback(&r, buf, sizeof(buf));
	CHECK(n == FTS_FRAME_OVERHEAD + 10U && decode_one(buf, n, &f) && fts_decode_readback(&f, &r2) == 0);
	CHECK(r2.t_us == 11U && r2.chute_fired == 1U);
	n = fts_encode_status(&s, buf, sizeof(buf));
	CHECK(n == FTS_FRAME_OVERHEAD + 35U && decode_one(buf, n, &f) && fts_decode_status(&f, &s2) == 0);
	CHECK(s2.state == 3U && s2.cause == 2U && s2.pbit_fail == 0x40U);
	CHECK(s2.t_trigger_us == 100U && s2.t_relay_us == 101U && s2.t_chute_us == 400U);
}

static void test_decode_rejects_wrong_type_or_len(void)
{
	struct fts_frame f = {.type = FTS_MSG_GNSS, .len = 21U};
	struct fts_gnss g;
	struct fts_imu i;

	CHECK(fts_decode_gnss(&f, &g) == -1);
	f.len = 22U;
	CHECK(fts_decode_imu(&f, &i) == -1);
}

static void test_parser_resyncs_after_noise_and_bad_crc(void)
{
	struct fts_ap_heartbeat h = {.t_us = 1U, .seq = 1U};
	uint8_t buf[128] = {0x3CU, FTS_SYNC, 0x04U, 0x30U};
	size_t n = 4U;
	struct fts_parser p;
	struct fts_frame frames[4];

	for (uint32_t i = 0; i < 3U; i++) {
		h.seq = i;
		n += fts_encode_ap_heartbeat(&h, &buf[n], sizeof(buf) - n);
	}
	buf[n - 1U] ^= 0xFFU; /* corrupt the last frame's CRC */
	fts_parser_init(&p);
	CHECK(feed_all(&p, buf, n, frames, 4U) == 2U);
	CHECK(p.crc_errors >= 1U);
	CHECK(frames[1].payload[8] == 1U);
}

int main(void)
{
	test_crc16_check_value();
	test_imu_roundtrip();
	test_gnss_baro_heartbeat_roundtrip();
	test_command_output_readback_status_roundtrip();
	test_decode_rejects_wrong_type_or_len();
	test_parser_resyncs_after_noise_and_bad_crc();
	return CHECK_DONE();
}
```

Append to `tests/host/CMakeLists.txt`:

```cmake
add_executable(test_proto test_proto.c ${REPO}/common/proto.c)
target_include_directories(test_proto PRIVATE ${REPO}/common/include)
add_test(NAME proto COMMAND test_proto)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S tests/host -B build/host 2>&1 | grep -m2 -i "cannot find\|proto.c"`
Expected: CMake reports that `common/proto.c` cannot be found.

- [ ] **Step 3: Implement**

Create `common/include/fts/proto.h`:

```c
#ifndef FTS_PROTO_H_
#define FTS_PROTO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FTS_SYNC 0xA5U
#define FTS_MAX_PAYLOAD 48U
/* sync, type, len, crc (2) */
#define FTS_FRAME_OVERHEAD 5U
#define FTS_FRAME_MAX (FTS_FRAME_OVERHEAD + FTS_MAX_PAYLOAD)

#define FTS_MSG_IMU 1U
#define FTS_MSG_GNSS 2U
#define FTS_MSG_BARO 3U
#define FTS_MSG_AP_HEARTBEAT 4U
#define FTS_MSG_GS_CMD 5U
#define FTS_MSG_OUTPUT 6U
#define FTS_MSG_READBACK 7U
#define FTS_MSG_STATUS 8U

#define FTS_CMD_PING 0U
#define FTS_CMD_ARM 1U
#define FTS_CMD_DISARM 2U
#define FTS_CMD_TERMINATE_ARM 3U
#define FTS_CMD_TERMINATE 4U

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection. */
uint16_t fts_crc16(const uint8_t *data, size_t len);

struct fts_frame {
	uint8_t type;
	uint8_t len;
	uint8_t payload[FTS_MAX_PAYLOAD];
};

/* Returns the frame length, or 0 if the payload or buffer is invalid. */
size_t fts_frame_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out,
			size_t out_size);

#define FTS_PARSER_QUEUE 4U

struct fts_parser {
	uint8_t buf[FTS_FRAME_MAX];
	uint8_t n;
	struct fts_frame queue[FTS_PARSER_QUEUE];
	uint8_t q_head;
	uint8_t q_count;
	uint32_t crc_errors;
	uint32_t len_errors;
	uint32_t queue_overflows;
};

void fts_parser_init(struct fts_parser *p);

/*
 * Feeds one byte; returns true when *out holds a valid frame. A bad
 * candidate frame drops only its sync byte, so frames after a false sync are
 * still found. Call fts_parser_next() until false for further frames.
 */
bool fts_parser_feed(struct fts_parser *p, uint8_t byte, struct fts_frame *out);
bool fts_parser_next(struct fts_parser *p, struct fts_frame *out);

struct fts_imu {
	uint64_t t_us;
	float q[4]; /* w, x, y, z: body to world */
	float gyro[3];
	float accel[3];
};

struct fts_gnss {
	uint64_t t_us;
	int32_t lat_e7;
	int32_t lon_e7;
	int32_t alt_mm;
	uint8_t fix; /* 0 none, 3 3D */
	uint8_t sats;
};

struct fts_baro {
	uint64_t t_us;
	float pressure_pa;
	float temp_c;
};

struct fts_ap_heartbeat {
	uint64_t t_us;
	uint32_t seq;
};

struct fts_gs_cmd {
	uint32_t seq;
	uint8_t cmd;
};

struct fts_output {
	uint8_t relay_open;
	uint8_t chute_fire;
};

struct fts_readback {
	uint64_t t_us;
	uint8_t relay_open;
	uint8_t chute_fired;
};

struct fts_status {
	uint64_t t_us;
	uint8_t state;
	uint8_t cause;
	uint8_t pbit_fail;
	uint64_t t_trigger_us;
	uint64_t t_relay_us;
	uint64_t t_chute_us;
};

size_t fts_encode_imu(const struct fts_imu *m, uint8_t *out, size_t out_size);
size_t fts_encode_gnss(const struct fts_gnss *m, uint8_t *out, size_t out_size);
size_t fts_encode_baro(const struct fts_baro *m, uint8_t *out, size_t out_size);
size_t fts_encode_ap_heartbeat(const struct fts_ap_heartbeat *m, uint8_t *out, size_t out_size);
size_t fts_encode_gs_cmd(const struct fts_gs_cmd *m, uint8_t *out, size_t out_size);
size_t fts_encode_output(const struct fts_output *m, uint8_t *out, size_t out_size);
size_t fts_encode_readback(const struct fts_readback *m, uint8_t *out, size_t out_size);
size_t fts_encode_status(const struct fts_status *m, uint8_t *out, size_t out_size);

/* Return 0 on success, -1 if the frame type or length does not match. */
int fts_decode_imu(const struct fts_frame *f, struct fts_imu *m);
int fts_decode_gnss(const struct fts_frame *f, struct fts_gnss *m);
int fts_decode_baro(const struct fts_frame *f, struct fts_baro *m);
int fts_decode_ap_heartbeat(const struct fts_frame *f, struct fts_ap_heartbeat *m);
int fts_decode_gs_cmd(const struct fts_frame *f, struct fts_gs_cmd *m);
int fts_decode_output(const struct fts_frame *f, struct fts_output *m);
int fts_decode_readback(const struct fts_frame *f, struct fts_readback *m);
int fts_decode_status(const struct fts_frame *f, struct fts_status *m);

#endif /* FTS_PROTO_H_ */
```

Create `common/proto.c`:

```c
#include <string.h>

#include "fts/proto.h"

static uint16_t crc_update(uint16_t crc, uint8_t byte)
{
	crc ^= (uint16_t)((uint16_t)byte << 8);
	for (int bit = 0; bit < 8; bit++) {
		if ((crc & 0x8000U) != 0U) {
			crc = (uint16_t)((crc << 1) ^ 0x1021U);
		} else {
			crc = (uint16_t)(crc << 1);
		}
	}
	return crc;
}

uint16_t fts_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFFU;

	for (size_t i = 0; i < len; i++) {
		crc = crc_update(crc, data[i]);
	}
	return crc;
}

size_t fts_frame_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out,
			size_t out_size)
{
	size_t total = FTS_FRAME_OVERHEAD + (size_t)len;
	uint16_t crc;

	if ((len > FTS_MAX_PAYLOAD) || (out_size < total) || ((len > 0U) && (payload == NULL))) {
		return 0U;
	}
	out[0] = FTS_SYNC;
	out[1] = type;
	out[2] = len;
	if (len > 0U) {
		memcpy(&out[3], payload, len);
	}
	crc = fts_crc16(&out[1], (size_t)len + 2U);
	out[3U + len] = (uint8_t)(crc & 0xFFU);
	out[4U + len] = (uint8_t)(crc >> 8);
	return total;
}

static void drop(struct fts_parser *p, uint8_t count)
{
	memmove(p->buf, &p->buf[count], (size_t)(p->n - count));
	p->n = (uint8_t)(p->n - count);
}

static void enqueue(struct fts_parser *p, uint8_t type, uint8_t len, const uint8_t *payload)
{
	struct fts_frame *f;

	if (p->q_count == FTS_PARSER_QUEUE) {
		p->queue_overflows++;
		return;
	}
	f = &p->queue[(p->q_head + p->q_count) % FTS_PARSER_QUEUE];
	f->type = type;
	f->len = len;
	memcpy(f->payload, payload, len);
	p->q_count++;
}

static void scan(struct fts_parser *p)
{
	for (;;) {
		uint8_t skip = 0U;
		uint8_t len;
		uint8_t total;
		uint16_t rx;

		while ((skip < p->n) && (p->buf[skip] != FTS_SYNC)) {
			skip++;
		}
		if (skip > 0U) {
			drop(p, skip);
		}
		if (p->n < 3U) {
			return;
		}
		len = p->buf[2];
		if (len > FTS_MAX_PAYLOAD) {
			p->len_errors++;
			drop(p, 1U);
			continue;
		}
		total = (uint8_t)(FTS_FRAME_OVERHEAD + len);
		if (p->n < total) {
			return;
		}
		rx = (uint16_t)((uint16_t)p->buf[3U + len] | ((uint16_t)p->buf[4U + len] << 8));
		if (fts_crc16(&p->buf[1], (size_t)len + 2U) != rx) {
			p->crc_errors++;
			drop(p, 1U);
			continue;
		}
		enqueue(p, p->buf[1], len, &p->buf[3]);
		drop(p, total);
	}
}

void fts_parser_init(struct fts_parser *p)
{
	memset(p, 0, sizeof(*p));
}

bool fts_parser_next(struct fts_parser *p, struct fts_frame *out)
{
	if (p->q_count == 0U) {
		return false;
	}
	*out = p->queue[p->q_head];
	p->q_head = (uint8_t)((p->q_head + 1U) % FTS_PARSER_QUEUE);
	p->q_count--;
	return true;
}

bool fts_parser_feed(struct fts_parser *p, uint8_t byte, struct fts_frame *out)
{
	/* scan() leaves at most FTS_FRAME_MAX - 1 bytes, so there is room for one more. */
	p->buf[p->n++] = byte;
	scan(p);
	return fts_parser_next(p, out);
}

/* Little-endian writer and reader over a payload buffer. */
struct wr {
	uint8_t *b;
	size_t n;
};

static void put_u8(struct wr *w, uint8_t v)
{
	w->b[w->n++] = v;
}

static void put_u32(struct wr *w, uint32_t v)
{
	for (int i = 0; i < 4; i++) {
		w->b[w->n++] = (uint8_t)(v >> (8 * i));
	}
}

static void put_u64(struct wr *w, uint64_t v)
{
	for (int i = 0; i < 8; i++) {
		w->b[w->n++] = (uint8_t)(v >> (8 * i));
	}
}

static void put_f32(struct wr *w, float v)
{
	uint32_t u;

	memcpy(&u, &v, sizeof(u));
	put_u32(w, u);
}

struct rd {
	const uint8_t *b;
	size_t n;
};

static uint8_t get_u8(struct rd *r)
{
	return r->b[r->n++];
}

static uint32_t get_u32(struct rd *r)
{
	uint32_t v = 0U;

	for (int i = 0; i < 4; i++) {
		v |= (uint32_t)r->b[r->n++] << (8 * i);
	}
	return v;
}

static uint64_t get_u64(struct rd *r)
{
	uint64_t v = 0U;

	for (int i = 0; i < 8; i++) {
		v |= (uint64_t)r->b[r->n++] << (8 * i);
	}
	return v;
}

static float get_f32(struct rd *r)
{
	uint32_t u = get_u32(r);
	float v;

	memcpy(&v, &u, sizeof(v));
	return v;
}

static int check(const struct fts_frame *f, uint8_t type, uint8_t len)
{
	return ((f->type == type) && (f->len == len)) ? 0 : -1;
}

size_t fts_encode_imu(const struct fts_imu *m, uint8_t *out, size_t out_size)
{
	uint8_t p[48];
	struct wr w = {p, 0U};

	put_u64(&w, m->t_us);
	for (int i = 0; i < 4; i++) {
		put_f32(&w, m->q[i]);
	}
	for (int i = 0; i < 3; i++) {
		put_f32(&w, m->gyro[i]);
	}
	for (int i = 0; i < 3; i++) {
		put_f32(&w, m->accel[i]);
	}
	return fts_frame_encode(FTS_MSG_IMU, p, (uint8_t)w.n, out, out_size);
}

int fts_decode_imu(const struct fts_frame *f, struct fts_imu *m)
{
	struct rd r = {f->payload, 0U};

	if (check(f, FTS_MSG_IMU, 48U) != 0) {
		return -1;
	}
	m->t_us = get_u64(&r);
	for (int i = 0; i < 4; i++) {
		m->q[i] = get_f32(&r);
	}
	for (int i = 0; i < 3; i++) {
		m->gyro[i] = get_f32(&r);
	}
	for (int i = 0; i < 3; i++) {
		m->accel[i] = get_f32(&r);
	}
	return 0;
}

size_t fts_encode_gnss(const struct fts_gnss *m, uint8_t *out, size_t out_size)
{
	uint8_t p[22];
	struct wr w = {p, 0U};

	put_u64(&w, m->t_us);
	put_u32(&w, (uint32_t)m->lat_e7);
	put_u32(&w, (uint32_t)m->lon_e7);
	put_u32(&w, (uint32_t)m->alt_mm);
	put_u8(&w, m->fix);
	put_u8(&w, m->sats);
	return fts_frame_encode(FTS_MSG_GNSS, p, (uint8_t)w.n, out, out_size);
}

int fts_decode_gnss(const struct fts_frame *f, struct fts_gnss *m)
{
	struct rd r = {f->payload, 0U};

	if (check(f, FTS_MSG_GNSS, 22U) != 0) {
		return -1;
	}
	m->t_us = get_u64(&r);
	m->lat_e7 = (int32_t)get_u32(&r);
	m->lon_e7 = (int32_t)get_u32(&r);
	m->alt_mm = (int32_t)get_u32(&r);
	m->fix = get_u8(&r);
	m->sats = get_u8(&r);
	return 0;
}

size_t fts_encode_baro(const struct fts_baro *m, uint8_t *out, size_t out_size)
{
	uint8_t p[16];
	struct wr w = {p, 0U};

	put_u64(&w, m->t_us);
	put_f32(&w, m->pressure_pa);
	put_f32(&w, m->temp_c);
	return fts_frame_encode(FTS_MSG_BARO, p, (uint8_t)w.n, out, out_size);
}

int fts_decode_baro(const struct fts_frame *f, struct fts_baro *m)
{
	struct rd r = {f->payload, 0U};

	if (check(f, FTS_MSG_BARO, 16U) != 0) {
		return -1;
	}
	m->t_us = get_u64(&r);
	m->pressure_pa = get_f32(&r);
	m->temp_c = get_f32(&r);
	return 0;
}

size_t fts_encode_ap_heartbeat(const struct fts_ap_heartbeat *m, uint8_t *out, size_t out_size)
{
	uint8_t p[12];
	struct wr w = {p, 0U};

	put_u64(&w, m->t_us);
	put_u32(&w, m->seq);
	return fts_frame_encode(FTS_MSG_AP_HEARTBEAT, p, (uint8_t)w.n, out, out_size);
}

int fts_decode_ap_heartbeat(const struct fts_frame *f, struct fts_ap_heartbeat *m)
{
	struct rd r = {f->payload, 0U};

	if (check(f, FTS_MSG_AP_HEARTBEAT, 12U) != 0) {
		return -1;
	}
	m->t_us = get_u64(&r);
	m->seq = get_u32(&r);
	return 0;
}

size_t fts_encode_gs_cmd(const struct fts_gs_cmd *m, uint8_t *out, size_t out_size)
{
	uint8_t p[5];
	struct wr w = {p, 0U};

	put_u32(&w, m->seq);
	put_u8(&w, m->cmd);
	return fts_frame_encode(FTS_MSG_GS_CMD, p, (uint8_t)w.n, out, out_size);
}

int fts_decode_gs_cmd(const struct fts_frame *f, struct fts_gs_cmd *m)
{
	struct rd r = {f->payload, 0U};

	if (check(f, FTS_MSG_GS_CMD, 5U) != 0) {
		return -1;
	}
	m->seq = get_u32(&r);
	m->cmd = get_u8(&r);
	return 0;
}

size_t fts_encode_output(const struct fts_output *m, uint8_t *out, size_t out_size)
{
	uint8_t p[2] = {m->relay_open, m->chute_fire};

	return fts_frame_encode(FTS_MSG_OUTPUT, p, sizeof(p), out, out_size);
}

int fts_decode_output(const struct fts_frame *f, struct fts_output *m)
{
	if (check(f, FTS_MSG_OUTPUT, 2U) != 0) {
		return -1;
	}
	m->relay_open = f->payload[0];
	m->chute_fire = f->payload[1];
	return 0;
}

size_t fts_encode_readback(const struct fts_readback *m, uint8_t *out, size_t out_size)
{
	uint8_t p[10];
	struct wr w = {p, 0U};

	put_u64(&w, m->t_us);
	put_u8(&w, m->relay_open);
	put_u8(&w, m->chute_fired);
	return fts_frame_encode(FTS_MSG_READBACK, p, (uint8_t)w.n, out, out_size);
}

int fts_decode_readback(const struct fts_frame *f, struct fts_readback *m)
{
	struct rd r = {f->payload, 0U};

	if (check(f, FTS_MSG_READBACK, 10U) != 0) {
		return -1;
	}
	m->t_us = get_u64(&r);
	m->relay_open = get_u8(&r);
	m->chute_fired = get_u8(&r);
	return 0;
}

size_t fts_encode_status(const struct fts_status *m, uint8_t *out, size_t out_size)
{
	uint8_t p[35];
	struct wr w = {p, 0U};

	put_u64(&w, m->t_us);
	put_u8(&w, m->state);
	put_u8(&w, m->cause);
	put_u8(&w, m->pbit_fail);
	put_u64(&w, m->t_trigger_us);
	put_u64(&w, m->t_relay_us);
	put_u64(&w, m->t_chute_us);
	return fts_frame_encode(FTS_MSG_STATUS, p, (uint8_t)w.n, out, out_size);
}

int fts_decode_status(const struct fts_frame *f, struct fts_status *m)
{
	struct rd r = {f->payload, 0U};

	if (check(f, FTS_MSG_STATUS, 35U) != 0) {
		return -1;
	}
	m->t_us = get_u64(&r);
	m->state = get_u8(&r);
	m->cause = get_u8(&r);
	m->pbit_fail = get_u8(&r);
	m->t_trigger_us = get_u64(&r);
	m->t_relay_us = get_u64(&r);
	m->t_chute_us = get_u64(&r);
	return 0;
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cmake -S tests/host -B build/host >/dev/null && cmake --build build/host >/dev/null && ctest --test-dir build/host --output-on-failure`
Expected: `100% tests passed, 0 tests failed out of 2`.

- [ ] **Step 5: Commit**

```bash
git add common tests/host
git commit -m "feat: add the FTS frame protocol" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: Fence geometry

**Files:**
- Create: `fts/src/logic/geo.h`, `fts/src/logic/geo.c`, `tests/host/test_geo.c`
- Modify: `tests/host/CMakeLists.txt`

**Interfaces:**
- Produces: `GEO_MAX_VERTICES` (16); `struct geo_origin { int32_t lat_e7; int32_t lon_e7; }`; `struct geo_fence { struct geo_point v[GEO_MAX_VERTICES]; int n; double ceiling_m; }` with `struct geo_point { double e; double n; }` in meters east/north of the origin; `void geo_to_local(const struct geo_origin *o, int32_t lat_e7, int32_t lon_e7, struct geo_point *p)`; `bool geo_fence_valid(const struct geo_fence *f)`; `bool geo_inside(const struct geo_fence *f, struct geo_point p)` (points on an edge count as inside).

- [ ] **Step 1: Write the failing test**

Create `tests/host/test_geo.c`:

```c
#include <math.h>

#include "check.h"
#include "geo.h"

static struct geo_fence square(void)
{
	struct geo_fence f = {.v = {{-100, -100}, {100, -100}, {100, 100}, {-100, 100}},
			      .n = 4, .ceiling_m = 120.0};

	return f;
}

static void test_local_conversion(void)
{
	struct geo_origin o = {.lat_e7 = 375665000, .lon_e7 = 1269780000};
	struct geo_point p;

	geo_to_local(&o, 375665000, 1269780000, &p);
	CHECK(fabs(p.e) < 1e-6 && fabs(p.n) < 1e-6);
	geo_to_local(&o, 375665000 + 8993, 1269780000, &p); /* about 100 m north */
	CHECK(fabs(p.n - 100.0) < 0.5 && fabs(p.e) < 1e-6);
	geo_to_local(&o, 375665000, 1269780000 + 11339, &p); /* about 100 m east at 37.57 N */
	CHECK(fabs(p.e - 100.0) < 0.5 && fabs(p.n) < 1e-6);
}

static void test_inside_outside(void)
{
	struct geo_fence f = square();

	CHECK(geo_inside(&f, (struct geo_point){0, 0}));
	CHECK(geo_inside(&f, (struct geo_point){99.9, -99.9}));
	CHECK(!geo_inside(&f, (struct geo_point){100.1, 0}));
	CHECK(!geo_inside(&f, (struct geo_point){0, -150}));
}

static void test_edges_and_vertices_count_inside(void)
{
	struct geo_fence f = square();

	CHECK(geo_inside(&f, (struct geo_point){100, 0}));
	CHECK(geo_inside(&f, (struct geo_point){0, -100}));
	CHECK(geo_inside(&f, (struct geo_point){100, 100}));
	CHECK(geo_inside(&f, (struct geo_point){-100, -100}));
}

static void test_concave_polygon(void)
{
	/* U shape: the notch (0..50 east, 0..100 north) is outside. */
	struct geo_fence f = {.v = {{-100, -100}, {100, -100}, {100, 100}, {50, 100},
				    {50, 0}, {0, 0}, {0, 100}, {-100, 100}},
			      .n = 8, .ceiling_m = 50.0};

	CHECK(geo_inside(&f, (struct geo_point){-50, 50}));
	CHECK(geo_inside(&f, (struct geo_point){75, 50}));
	CHECK(!geo_inside(&f, (struct geo_point){25, 50}));
	CHECK(geo_inside(&f, (struct geo_point){25, -50}));
}

static void test_fence_validity(void)
{
	struct geo_fence f = square();

	CHECK(geo_fence_valid(&f));
	f.n = 2;
	CHECK(!geo_fence_valid(&f));
	f = square();
	f.ceiling_m = 0.0;
	CHECK(!geo_fence_valid(&f));
	f = square();
	f.n = GEO_MAX_VERTICES + 1;
	CHECK(!geo_fence_valid(&f));
}

int main(void)
{
	test_local_conversion();
	test_inside_outside();
	test_edges_and_vertices_count_inside();
	test_concave_polygon();
	test_fence_validity();
	return CHECK_DONE();
}
```

Append to `tests/host/CMakeLists.txt`:

```cmake
add_executable(test_geo test_geo.c ${REPO}/fts/src/logic/geo.c)
target_include_directories(test_geo PRIVATE ${REPO}/fts/src/logic)
target_link_libraries(test_geo m)
add_test(NAME geo COMMAND test_geo)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S tests/host -B build/host 2>&1 | grep -m2 -i "cannot find\|geo.c"`
Expected: CMake reports that `fts/src/logic/geo.c` cannot be found.

- [ ] **Step 3: Implement**

Create `fts/src/logic/geo.h`:

```c
#ifndef GEO_H_
#define GEO_H_

#include <stdbool.h>
#include <stdint.h>

#define GEO_MAX_VERTICES 16

struct geo_origin {
	int32_t lat_e7;
	int32_t lon_e7;
};

/* Meters east and north of the origin. */
struct geo_point {
	double e;
	double n;
};

struct geo_fence {
	struct geo_point v[GEO_MAX_VERTICES];
	int n;
	double ceiling_m; /* above the origin */
};

/* Local flat-earth conversion; accurate to well under 1 m within a few km. */
void geo_to_local(const struct geo_origin *o, int32_t lat_e7, int32_t lon_e7,
		  struct geo_point *p);

/* At least 3 vertices, at most GEO_MAX_VERTICES, ceiling above ground. */
bool geo_fence_valid(const struct geo_fence *f);

/* Points on an edge or vertex count as inside. */
bool geo_inside(const struct geo_fence *f, struct geo_point p);

#endif /* GEO_H_ */
```

Create `fts/src/logic/geo.c`:

```c
#include <math.h>

#include "geo.h"

#define EARTH_RADIUS_M 6371000.0
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
#define EDGE_EPS_M 1e-6

void geo_to_local(const struct geo_origin *o, int32_t lat_e7, int32_t lon_e7,
		  struct geo_point *p)
{
	double lat0 = (double)o->lat_e7 * 1e-7 * DEG_TO_RAD;
	double dlat = (double)(lat_e7 - o->lat_e7) * 1e-7 * DEG_TO_RAD;
	double dlon = (double)(lon_e7 - o->lon_e7) * 1e-7 * DEG_TO_RAD;

	p->n = dlat * EARTH_RADIUS_M;
	p->e = dlon * EARTH_RADIUS_M * cos(lat0);
}

bool geo_fence_valid(const struct geo_fence *f)
{
	return (f->n >= 3) && (f->n <= GEO_MAX_VERTICES) && (f->ceiling_m > 0.0);
}

static bool on_segment(struct geo_point a, struct geo_point b, struct geo_point p)
{
	double cross = (b.e - a.e) * (p.n - a.n) - (b.n - a.n) * (p.e - a.e);

	if (fabs(cross) > EDGE_EPS_M * (fabs(b.e - a.e) + fabs(b.n - a.n) + 1.0)) {
		return false;
	}
	return (p.e >= fmin(a.e, b.e) - EDGE_EPS_M) && (p.e <= fmax(a.e, b.e) + EDGE_EPS_M) &&
	       (p.n >= fmin(a.n, b.n) - EDGE_EPS_M) && (p.n <= fmax(a.n, b.n) + EDGE_EPS_M);
}

bool geo_inside(const struct geo_fence *f, struct geo_point p)
{
	bool inside = false;

	for (int i = 0, j = f->n - 1; i < f->n; j = i++) {
		struct geo_point a = f->v[i];
		struct geo_point b = f->v[j];

		if (on_segment(a, b, p)) {
			return true;
		}
		if (((a.n > p.n) != (b.n > p.n)) &&
		    (p.e < (b.e - a.e) * (p.n - a.n) / (b.n - a.n) + a.e)) {
			inside = !inside;
		}
	}
	return inside;
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cmake -S tests/host -B build/host >/dev/null && cmake --build build/host >/dev/null && ctest --test-dir build/host --output-on-failure`
Expected: `100% tests passed, 0 tests failed out of 3`.

- [ ] **Step 5: Commit**

```bash
git add fts/src/logic/geo.h fts/src/logic/geo.c tests/host
git commit -m "feat: add fence geometry" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: FTS state machine, triggers, and termination sequence

**Files:**
- Create: `fts/src/logic/fts.h`, `fts/src/logic/fts.c`, `tests/host/test_fts.c`
- Modify: `tests/host/CMakeLists.txt`

**Interfaces:**
- Consumes: `geo.h` (Task 3); `FTS_CMD_*` from `fts/proto.h` (Task 2).
- Produces:
  - `enum fts_state { FTS_PBIT, FTS_SAFE, FTS_ARMED, FTS_TERMINATED, FTS_FAULT }`;
  - `enum fts_cause { FTS_CAUSE_NONE, FTS_CAUSE_FENCE, FTS_CAUSE_CEILING, FTS_CAUSE_CONTROL, FTS_CAUSE_AP_FREEZE, FTS_CAUSE_LINK, FTS_CAUSE_MANUAL }`;
  - `struct fts_config` (thresholds, fence, origin), `fts_default_config(struct fts_config *c)`;
  - `struct fts` context; `fts_init`, `fts_pbit_done(struct fts *s, uint8_t fail_mask, int64_t t_us)`;
  - inputs: `fts_on_imu(struct fts *s, int64_t t_us, const float q[4], const float gyro[3])`, `fts_on_gnss(struct fts *s, int64_t t_us, int32_t lat_e7, int32_t lon_e7, int32_t alt_mm, uint8_t fix)`, `fts_on_baro(struct fts *s, int64_t t_us, float pressure_pa)`, `fts_on_heartbeat(struct fts *s, int64_t t_us)`, `fts_on_command(struct fts *s, int64_t t_us, uint8_t cmd)` (returns true if accepted);
  - `fts_tick(struct fts *s, int64_t t_us)` evaluates triggers and the sequence;
  - outputs: `bool fts_relay_open(const struct fts *s)`, `bool fts_chute_fire(const struct fts *s)`; `s->state`, `s->cause`, `s->t_trigger_us`, `s->t_relay_us`, `s->t_chute_us` (-1 until set).

Behavior (spec): triggers only in `FTS_ARMED`; each condition must hold continuously for its confirmation time; time going backwards restarts confirmation windows (never fires early); `ARM` needs `FTS_SAFE`, a 3D GNSS fix, the last GNSS position inside the fence, a heartbeat within the timeout, and records the baro reference pressure; manual terminate needs `TERMINATE_ARM` then `TERMINATE` within the window; `FTS_TERMINATED` latches the relay open and the parachute fired; the parachute fires `chute_delay_us` after the relay opens.

- [ ] **Step 1: Write the failing test**

Create `tests/host/test_fts.c`:

```c
#include <math.h>

#include "check.h"
#include "fts.h"
#include "fts/proto.h"

#define MS 1000LL
#define S 1000000LL

static const float LEVEL[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static const float STILL[3] = {0.0f, 0.0f, 0.0f};

/* Origin is also the fence center; fence is +-100 m, ceiling 120 m. */
static void setup(struct fts *s, struct fts_config *c)
{
	fts_default_config(c);
	c->origin.lat_e7 = 375665000;
	c->origin.lon_e7 = 1269780000;
	fts_init(s, c);
	fts_pbit_done(s, 0U, 0);
}

/* Feeds healthy inputs at 100 Hz from t0 to t1 at the given local position. */
static void healthy(struct fts *s, int64_t t0, int64_t t1, int32_t dlat_e7)
{
	for (int64_t t = t0; t < t1; t += 10 * MS) {
		fts_on_imu(s, t, LEVEL, STILL);
		if ((t % (100 * MS)) == 0) {
			fts_on_gnss(s, t, 375665000 + dlat_e7, 1269780000, 30000, 3U);
			fts_on_heartbeat(s, t);
			fts_on_baro(s, t, 101325.0f);
			(void)fts_on_command(s, t, FTS_CMD_PING);
		}
		fts_tick(s, t);
	}
}

static void armed(struct fts *s, struct fts_config *c)
{
	setup(s, c);
	healthy(s, 0, 1 * S, 0);
	CHECK(fts_on_command(s, 1 * S, FTS_CMD_ARM));
	CHECK(s->state == FTS_ARMED);
}

static void test_pbit_pass_and_fail(void)
{
	struct fts_config c;
	struct fts s;

	fts_default_config(&c);
	fts_init(&s, &c);
	CHECK(s.state == FTS_PBIT);
	fts_pbit_done(&s, 0x04U, 0);
	CHECK(s.state == FTS_FAULT && s.pbit_fail == 0x04U);
	CHECK(!fts_on_command(&s, 0, FTS_CMD_ARM));
	setup(&s, &c);
	CHECK(s.state == FTS_SAFE);
}

static void test_arm_needs_fix_heartbeat_and_inside(void)
{
	struct fts_config c;
	struct fts s;

	setup(&s, &c);
	CHECK(!fts_on_command(&s, 0, FTS_CMD_ARM)); /* no GNSS yet */
	healthy(&s, 0, 1 * S, 20000000);            /* about 2.2 km north: outside */
	CHECK(!fts_on_command(&s, 1 * S, FTS_CMD_ARM));
	healthy(&s, 1 * S, 2 * S, 0);
	CHECK(fts_on_command(&s, 2 * S, FTS_CMD_ARM));
	CHECK(fts_on_command(&s, 3 * S, FTS_CMD_DISARM) && s.state == FTS_SAFE);
}

static void test_no_termination_while_safe(void)
{
	struct fts_config c;
	struct fts s;

	setup(&s, &c);
	healthy(&s, 0, 1 * S, 20000000); /* outside the fence, not armed */
	fts_tick(&s, 10 * S);           /* no heartbeat, no link for 9 s */
	CHECK(s.state == FTS_SAFE && !fts_relay_open(&s));
}

static void test_fence_breach_confirmed_after_half_second(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	healthy(&s, 1 * S, 1 * S + 400 * MS, 20000000);
	CHECK(s.state == FTS_ARMED);
	healthy(&s, 1 * S + 400 * MS, 2 * S, 20000000);
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_FENCE);
	CHECK(fts_relay_open(&s) && s.t_relay_us == s.t_trigger_us);
}

static void test_single_bad_gnss_sample_does_not_terminate(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	healthy(&s, 1 * S, 2 * S, 0);
	fts_on_gnss(&s, 2 * S, 375665000 + 20000000, 1269780000, 30000, 3U);
	fts_tick(&s, 2 * S);
	healthy(&s, 2 * S + 100 * MS, 4 * S, 0);
	CHECK(s.state == FTS_ARMED);
}

static void test_ceiling_from_baro(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	/* About 130 m above the reference: 101325 Pa -> about 99783 Pa. */
	for (int64_t t = 1 * S; t < 2 * S; t += 10 * MS) {
		fts_on_imu(&s, t, LEVEL, STILL);
		if ((t % (100 * MS)) == 0) {
			fts_on_gnss(&s, t, 375665000, 1269780000, 30000, 3U);
			fts_on_heartbeat(&s, t);
			fts_on_baro(&s, t, 99783.0f);
			(void)fts_on_command(&s, t, FTS_CMD_PING);
		}
		fts_tick(&s, t);
	}
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_CEILING);
}

static void test_loss_of_control_tilt(void)
{
	struct fts_config c;
	struct fts s;
	/* 90 degrees about x: w = cos(45), x = sin(45). */
	const float flipped[4] = {0.7071068f, 0.7071068f, 0.0f, 0.0f};

	armed(&s, &c);
	for (int64_t t = 1 * S; t <= 1 * S + 600 * MS; t += 10 * MS) {
		fts_on_imu(&s, t, flipped, STILL);
		fts_on_heartbeat(&s, t);
		fts_tick(&s, t);
	}
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_CONTROL);
}

static void test_loss_of_control_rate(void)
{
	struct fts_config c;
	struct fts s;
	const float spin[3] = {0.0f, 0.0f, 6.0f}; /* about 344 deg/s */

	armed(&s, &c);
	for (int64_t t = 1 * S; t <= 1 * S + 600 * MS; t += 10 * MS) {
		fts_on_imu(&s, t, LEVEL, spin);
		fts_on_heartbeat(&s, t);
		fts_tick(&s, t);
	}
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_CONTROL);
}

static void test_autopilot_freeze(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	fts_tick(&s, 1 * S + 900 * MS + 100 * MS - 1); /* last heartbeat at 0.9 s */
	CHECK(s.state == FTS_ARMED);
	fts_tick(&s, 2 * S + 1);
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_AP_FREEZE);
}

static void test_single_missed_heartbeat_does_not_terminate(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	fts_tick(&s, 1 * S + 500 * MS);
	fts_on_heartbeat(&s, 1 * S + 500 * MS);
	healthy(&s, 1 * S + 500 * MS, 3 * S, 0);
	CHECK(s.state == FTS_ARMED);
}

static void test_link_loss(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	for (int64_t t = 1 * S; t <= 7 * S; t += 100 * MS) {
		fts_on_imu(&s, t, LEVEL, STILL);
		fts_on_gnss(&s, t, 375665000, 1269780000, 30000, 3U);
		fts_on_heartbeat(&s, t);
		fts_tick(&s, t);
		if (t < 5 * S + 900 * MS) {
			CHECK(s.state == FTS_ARMED);
		}
	}
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_LINK);
}

static void test_manual_terminate_needs_both_steps_in_window(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	CHECK(!fts_on_command(&s, 1 * S, FTS_CMD_TERMINATE)); /* no TERMINATE_ARM */
	CHECK(fts_on_command(&s, 1 * S, FTS_CMD_TERMINATE_ARM));
	healthy(&s, 1 * S, 4 * S + 100 * MS, 0);
	CHECK(!fts_on_command(&s, 4 * S + 100 * MS, FTS_CMD_TERMINATE)); /* stale */
	CHECK(s.state == FTS_ARMED);
	CHECK(fts_on_command(&s, 5 * S, FTS_CMD_TERMINATE_ARM));
	CHECK(fts_on_command(&s, 6 * S, FTS_CMD_TERMINATE));
	fts_tick(&s, 6 * S);
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_MANUAL);
}

static void test_parachute_after_delay_and_latched(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	(void)fts_on_command(&s, 2 * S, FTS_CMD_TERMINATE_ARM);
	(void)fts_on_command(&s, 2 * S, FTS_CMD_TERMINATE);
	fts_tick(&s, 2 * S);
	CHECK(fts_relay_open(&s) && !fts_chute_fire(&s));
	fts_tick(&s, 2 * S + 299 * MS);
	CHECK(!fts_chute_fire(&s));
	fts_tick(&s, 2 * S + 300 * MS);
	CHECK(fts_chute_fire(&s) && s.t_chute_us == 2 * S + 300 * MS);
	CHECK(!fts_on_command(&s, 3 * S, FTS_CMD_DISARM));
	CHECK(!fts_on_command(&s, 3 * S, FTS_CMD_ARM));
	healthy(&s, 3 * S, 5 * S, 0);
	CHECK(s.state == FTS_TERMINATED && fts_relay_open(&s) && fts_chute_fire(&s));
}

static void test_time_going_backwards_restarts_confirmation(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	healthy(&s, 1 * S, 1 * S + 400 * MS, 20000000); /* 0.4 s outside */
	healthy(&s, 1 * S, 1 * S + 400 * MS, 20000000); /* clock jumped back */
	CHECK(s.state == FTS_ARMED);
}

int main(void)
{
	test_pbit_pass_and_fail();
	test_arm_needs_fix_heartbeat_and_inside();
	test_no_termination_while_safe();
	test_fence_breach_confirmed_after_half_second();
	test_single_bad_gnss_sample_does_not_terminate();
	test_ceiling_from_baro();
	test_loss_of_control_tilt();
	test_loss_of_control_rate();
	test_autopilot_freeze();
	test_single_missed_heartbeat_does_not_terminate();
	test_link_loss();
	test_manual_terminate_needs_both_steps_in_window();
	test_parachute_after_delay_and_latched();
	test_time_going_backwards_restarts_confirmation();
	return CHECK_DONE();
}
```

Append to `tests/host/CMakeLists.txt`:

```cmake
add_executable(test_fts test_fts.c ${REPO}/fts/src/logic/fts.c ${REPO}/fts/src/logic/geo.c)
target_include_directories(test_fts PRIVATE ${REPO}/fts/src/logic ${REPO}/common/include)
target_link_libraries(test_fts m)
add_test(NAME fts COMMAND test_fts)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S tests/host -B build/host 2>&1 | grep -m2 -i "cannot find\|fts.c"`
Expected: CMake reports that `fts/src/logic/fts.c` cannot be found.

- [ ] **Step 3: Implement**

Create `fts/src/logic/fts.h`:

```c
#ifndef FTS_H_
#define FTS_H_

#include <stdbool.h>
#include <stdint.h>

#include "geo.h"

enum fts_state { FTS_PBIT, FTS_SAFE, FTS_ARMED, FTS_TERMINATED, FTS_FAULT };

enum fts_cause {
	FTS_CAUSE_NONE,
	FTS_CAUSE_FENCE,
	FTS_CAUSE_CEILING,
	FTS_CAUSE_CONTROL,
	FTS_CAUSE_AP_FREEZE,
	FTS_CAUSE_LINK,
	FTS_CAUSE_MANUAL,
};

struct fts_config {
	struct geo_origin origin;
	struct geo_fence fence;
	int64_t fence_confirm_us;
	int64_t control_confirm_us;
	float tilt_limit_rad;
	float rate_limit_rad_s;
	int64_t heartbeat_timeout_us;
	int64_t link_timeout_us;
	int64_t manual_window_us;
	int64_t chute_delay_us;
};

/* Tracks how long a condition has held without a break; -1 when it does not hold. */
struct fts_hold {
	int64_t since_us;
	int64_t last_us;
};

struct fts {
	struct fts_config cfg;
	enum fts_state state;
	enum fts_cause cause;
	uint8_t pbit_fail;
	/* latest inputs; -1 means never received */
	int64_t t_gnss_us;
	struct geo_point pos;
	double gnss_alt_m;
	uint8_t gnss_fix;
	int64_t t_baro_us;
	float pressure_pa;
	float ref_pressure_pa;
	double ref_gnss_alt_m;
	int64_t t_heartbeat_us;
	int64_t t_link_us;
	int64_t t_terminate_arm_us;
	bool manual_request;
	struct fts_hold hold_fence;
	struct fts_hold hold_ceiling;
	struct fts_hold hold_control;
	/* termination record; -1 until set */
	int64_t t_trigger_us;
	int64_t t_relay_us;
	int64_t t_chute_us;
};

void fts_default_config(struct fts_config *c);
void fts_init(struct fts *s, const struct fts_config *c);
void fts_pbit_done(struct fts *s, uint8_t fail_mask, int64_t t_us);

void fts_on_imu(struct fts *s, int64_t t_us, const float q[4], const float gyro[3]);
void fts_on_gnss(struct fts *s, int64_t t_us, int32_t lat_e7, int32_t lon_e7, int32_t alt_mm,
		 uint8_t fix);
void fts_on_baro(struct fts *s, int64_t t_us, float pressure_pa);
void fts_on_heartbeat(struct fts *s, int64_t t_us);

/* Returns true if the command was accepted. Any valid command refreshes the link. */
bool fts_on_command(struct fts *s, int64_t t_us, uint8_t cmd);

void fts_tick(struct fts *s, int64_t t_us);

bool fts_relay_open(const struct fts *s);
bool fts_chute_fire(const struct fts *s);

#endif /* FTS_H_ */
```

Create `fts/src/logic/fts.c`:

```c
#include <math.h>
#include <string.h>

#include "fts.h"
#include "fts/proto.h"

#define DEG_TO_RAD (3.14159265358979323846 / 180.0)

void fts_default_config(struct fts_config *c)
{
	memset(c, 0, sizeof(*c));
	c->fence.v[0] = (struct geo_point){-100.0, -100.0};
	c->fence.v[1] = (struct geo_point){100.0, -100.0};
	c->fence.v[2] = (struct geo_point){100.0, 100.0};
	c->fence.v[3] = (struct geo_point){-100.0, 100.0};
	c->fence.n = 4;
	c->fence.ceiling_m = 120.0;
	c->fence_confirm_us = 500000;
	c->control_confirm_us = 500000;
	c->tilt_limit_rad = (float)(60.0 * DEG_TO_RAD);
	c->rate_limit_rad_s = (float)(300.0 * DEG_TO_RAD);
	c->heartbeat_timeout_us = 1000000;
	c->link_timeout_us = 5000000;
	c->manual_window_us = 3000000;
	c->chute_delay_us = 300000;
}

static void hold_reset(struct fts_hold *h)
{
	h->since_us = -1;
	h->last_us = -1;
}

/* Feeds one sample of a condition; a clock going backwards restarts the window. */
static void hold_sample(struct fts_hold *h, int64_t t_us, bool cond)
{
	if (!cond || ((h->last_us >= 0) && (t_us < h->last_us))) {
		hold_reset(h);
	}
	if (cond && (h->since_us < 0)) {
		h->since_us = t_us;
	}
	h->last_us = t_us;
}

static bool hold_confirmed(const struct fts_hold *h, int64_t confirm_us)
{
	return (h->since_us >= 0) && ((h->last_us - h->since_us) >= confirm_us);
}

void fts_init(struct fts *s, const struct fts_config *c)
{
	memset(s, 0, sizeof(*s));
	s->cfg = *c;
	s->state = FTS_PBIT;
	s->cause = FTS_CAUSE_NONE;
	s->t_gnss_us = -1;
	s->t_baro_us = -1;
	s->t_heartbeat_us = -1;
	s->t_link_us = -1;
	s->t_terminate_arm_us = -1;
	hold_reset(&s->hold_fence);
	hold_reset(&s->hold_ceiling);
	hold_reset(&s->hold_control);
	s->t_trigger_us = -1;
	s->t_relay_us = -1;
	s->t_chute_us = -1;
}

void fts_pbit_done(struct fts *s, uint8_t fail_mask, int64_t t_us)
{
	(void)t_us;
	if (s->state != FTS_PBIT) {
		return;
	}
	s->pbit_fail = fail_mask;
	if (!geo_fence_valid(&s->cfg.fence)) {
		s->pbit_fail |= 0x40U;
	}
	s->state = (s->pbit_fail == 0U) ? FTS_SAFE : FTS_FAULT;
}

void fts_on_imu(struct fts *s, int64_t t_us, const float q[4], const float gyro[3])
{
	/* Tilt between body z and world z: cos(tilt) = 1 - 2(x^2 + y^2). */
	float c = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
	float tilt = acosf(fmaxf(-1.0f, fminf(1.0f, c)));
	float rate = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);

	hold_sample(&s->hold_control, t_us,
		    (tilt > s->cfg.tilt_limit_rad) || (rate > s->cfg.rate_limit_rad_s));
}

void fts_on_gnss(struct fts *s, int64_t t_us, int32_t lat_e7, int32_t lon_e7, int32_t alt_mm,
		 uint8_t fix)
{
	s->t_gnss_us = t_us;
	s->gnss_fix = fix;
	s->gnss_alt_m = (double)alt_mm / 1000.0;
	if (fix >= 3U) {
		geo_to_local(&s->cfg.origin, lat_e7, lon_e7, &s->pos);
		hold_sample(&s->hold_fence, t_us, !geo_inside(&s->cfg.fence, s->pos));
	}
}

void fts_on_baro(struct fts *s, int64_t t_us, float pressure_pa)
{
	s->t_baro_us = t_us;
	s->pressure_pa = pressure_pa;
}

void fts_on_heartbeat(struct fts *s, int64_t t_us)
{
	s->t_heartbeat_us = t_us;
}

static bool recent(int64_t t_event, int64_t t_now, int64_t window)
{
	return (t_event >= 0) && (t_now >= t_event) && ((t_now - t_event) <= window);
}

bool fts_on_command(struct fts *s, int64_t t_us, uint8_t cmd)
{
	if (s->state == FTS_ARMED || s->state == FTS_SAFE) {
		s->t_link_us = t_us;
	}
	switch (cmd) {
	case FTS_CMD_PING:
		return (s->state == FTS_ARMED) || (s->state == FTS_SAFE);
	case FTS_CMD_ARM:
		if ((s->state != FTS_SAFE) || (s->gnss_fix < 3U) || (s->t_gnss_us < 0) ||
		    !geo_inside(&s->cfg.fence, s->pos) ||
		    !recent(s->t_heartbeat_us, t_us, s->cfg.heartbeat_timeout_us) ||
		    (s->t_baro_us < 0)) {
			return false;
		}
		s->ref_pressure_pa = s->pressure_pa;
		s->ref_gnss_alt_m = s->gnss_alt_m;
		hold_reset(&s->hold_fence);
		hold_reset(&s->hold_ceiling);
		hold_reset(&s->hold_control);
		s->t_terminate_arm_us = -1;
		s->state = FTS_ARMED;
		return true;
	case FTS_CMD_DISARM:
		if (s->state != FTS_ARMED) {
			return false;
		}
		s->state = FTS_SAFE;
		return true;
	case FTS_CMD_TERMINATE_ARM:
		if (s->state != FTS_ARMED) {
			return false;
		}
		s->t_terminate_arm_us = t_us;
		return true;
	case FTS_CMD_TERMINATE:
		if ((s->state != FTS_ARMED) ||
		    !recent(s->t_terminate_arm_us, t_us, s->cfg.manual_window_us)) {
			return false;
		}
		s->manual_request = true;
		return true;
	default:
		return false;
	}
}

/* Pressure altitude above the reference, standard atmosphere. */
static double baro_alt_m(float p, float p0)
{
	return 44330.0 * (1.0 - pow((double)p / (double)p0, 1.0 / 5.255));
}

static void terminate(struct fts *s, enum fts_cause cause, int64_t t_us)
{
	s->state = FTS_TERMINATED;
	s->cause = cause;
	s->t_trigger_us = t_us;
	s->t_relay_us = t_us;
}

void fts_tick(struct fts *s, int64_t t_us)
{
	if (s->state == FTS_TERMINATED) {
		if ((s->t_chute_us < 0) && ((t_us - s->t_relay_us) >= s->cfg.chute_delay_us)) {
			s->t_chute_us = t_us;
		}
		return;
	}
	if (s->state != FTS_ARMED) {
		return;
	}
	if (s->t_baro_us >= 0) {
		double alt = baro_alt_m(s->pressure_pa, s->ref_pressure_pa);
		double gnss_rel = s->gnss_alt_m - s->ref_gnss_alt_m;

		hold_sample(&s->hold_ceiling, t_us, fmax(alt, gnss_rel) > s->cfg.fence.ceiling_m);
	}
	if (s->manual_request) {
		terminate(s, FTS_CAUSE_MANUAL, t_us);
	} else if (hold_confirmed(&s->hold_fence, s->cfg.fence_confirm_us)) {
		terminate(s, FTS_CAUSE_FENCE, t_us);
	} else if (hold_confirmed(&s->hold_ceiling, s->cfg.fence_confirm_us)) {
		terminate(s, FTS_CAUSE_CEILING, t_us);
	} else if (hold_confirmed(&s->hold_control, s->cfg.control_confirm_us)) {
		terminate(s, FTS_CAUSE_CONTROL, t_us);
	} else if (!recent(s->t_heartbeat_us, t_us, s->cfg.heartbeat_timeout_us)) {
		terminate(s, FTS_CAUSE_AP_FREEZE, t_us);
	} else if (!recent(s->t_link_us, t_us, s->cfg.link_timeout_us)) {
		terminate(s, FTS_CAUSE_LINK, t_us);
	}
	if ((s->state == FTS_TERMINATED) && (s->cfg.chute_delay_us <= 0)) {
		s->t_chute_us = t_us;
	}
}

bool fts_relay_open(const struct fts *s)
{
	return s->state == FTS_TERMINATED;
}

bool fts_chute_fire(const struct fts *s)
{
	return (s->state == FTS_TERMINATED) && (s->t_chute_us >= 0);
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cmake -S tests/host -B build/host >/dev/null && cmake --build build/host >/dev/null && ctest --test-dir build/host --output-on-failure`
Expected: `100% tests passed, 0 tests failed out of 4`. If a threshold test fails because of an off-by-one in the confirmation window, fix the logic to match the spec (condition held for at least the confirmation time), not the test.

- [ ] **Step 5: Commit**

```bash
git add fts/src/logic/fts.h fts/src/logic/fts.c tests/host
git commit -m "feat: add the FTS state machine and triggers" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: Self-test (PBIT) checks

**Files:**
- Create: `fts/src/logic/pbit.h`, `fts/src/logic/pbit.c`, `tests/host/test_pbit.c`
- Modify: `tests/host/CMakeLists.txt`

**Interfaces:**
- Produces: `PBIT_IMU_RATE` (bit 0), `PBIT_GNSS_RATE` (1), `PBIT_BARO_RATE` (2), `PBIT_GRAVITY` (3), `PBIT_RELAY` (4), `PBIT_CHUTE` (5), `PBIT_FENCE` (6, set by `fts_pbit_done`); `struct pbit_input { uint32_t imu_count; uint32_t gnss_count; uint32_t baro_count; int64_t window_us; float accel_norm; bool relay_readback_open; bool chute_readback_fired; }`; `uint8_t pbit_eval(const struct pbit_input *in)`.

Rates over the window must reach at least 80% of nominal (IMU 100 Hz, GNSS 10 Hz, baro 20 Hz); `|accel_norm - 9.81| <= 1.0`; the relay must read back closed and the parachute not fired.

- [ ] **Step 1: Write the failing test**

Create `tests/host/test_pbit.c`:

```c
#include "check.h"
#include "pbit.h"

static struct pbit_input good(void)
{
	struct pbit_input in = {.imu_count = 100U, .gnss_count = 10U, .baro_count = 20U,
				.window_us = 1000000, .accel_norm = 9.81f,
				.relay_readback_open = false, .chute_readback_fired = false};

	return in;
}

static void test_all_pass(void)
{
	struct pbit_input in = good();

	CHECK(pbit_eval(&in) == 0U);
}

static void test_rates(void)
{
	struct pbit_input in = good();

	in.imu_count = 80U;
	CHECK(pbit_eval(&in) == 0U);
	in.imu_count = 79U;
	CHECK(pbit_eval(&in) == PBIT_IMU_RATE);
	in = good();
	in.gnss_count = 7U;
	CHECK(pbit_eval(&in) == PBIT_GNSS_RATE);
	in = good();
	in.baro_count = 0U;
	CHECK(pbit_eval(&in) == PBIT_BARO_RATE);
}

static void test_gravity_and_outputs(void)
{
	struct pbit_input in = good();

	in.accel_norm = 11.0f;
	CHECK(pbit_eval(&in) == PBIT_GRAVITY);
	in = good();
	in.relay_readback_open = true;
	CHECK(pbit_eval(&in) == PBIT_RELAY);
	in = good();
	in.chute_readback_fired = true;
	CHECK(pbit_eval(&in) == PBIT_CHUTE);
}

int main(void)
{
	test_all_pass();
	test_rates();
	test_gravity_and_outputs();
	return CHECK_DONE();
}
```

Append to `tests/host/CMakeLists.txt`:

```cmake
add_executable(test_pbit test_pbit.c ${REPO}/fts/src/logic/pbit.c)
target_include_directories(test_pbit PRIVATE ${REPO}/fts/src/logic)
target_link_libraries(test_pbit m)
add_test(NAME pbit COMMAND test_pbit)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S tests/host -B build/host 2>&1 | grep -m2 -i "cannot find\|pbit.c"`
Expected: CMake reports that `fts/src/logic/pbit.c` cannot be found.

- [ ] **Step 3: Implement**

Create `fts/src/logic/pbit.h`:

```c
#ifndef PBIT_H_
#define PBIT_H_

#include <stdbool.h>
#include <stdint.h>

#define PBIT_IMU_RATE (1U << 0)
#define PBIT_GNSS_RATE (1U << 1)
#define PBIT_BARO_RATE (1U << 2)
#define PBIT_GRAVITY (1U << 3)
#define PBIT_RELAY (1U << 4)
#define PBIT_CHUTE (1U << 5)
#define PBIT_FENCE (1U << 6)

struct pbit_input {
	uint32_t imu_count;
	uint32_t gnss_count;
	uint32_t baro_count;
	int64_t window_us;
	float accel_norm;
	bool relay_readback_open;
	bool chute_readback_fired;
};

/* Returns the fail mask (PBIT_* bits); PBIT_FENCE is added by fts_pbit_done(). */
uint8_t pbit_eval(const struct pbit_input *in);

#endif /* PBIT_H_ */
```

Create `fts/src/logic/pbit.c`:

```c
#include <math.h>

#include "pbit.h"

#define IMU_HZ 100.0
#define GNSS_HZ 10.0
#define BARO_HZ 20.0
#define MIN_RATE_FRACTION 0.8
#define GRAVITY 9.81f
#define GRAVITY_TOL 1.0f

static bool rate_ok(uint32_t count, int64_t window_us, double hz)
{
	double expected = hz * (double)window_us / 1e6;

	return (double)count >= MIN_RATE_FRACTION * expected - 1e-9;
}

uint8_t pbit_eval(const struct pbit_input *in)
{
	uint8_t fail = 0U;

	if (!rate_ok(in->imu_count, in->window_us, IMU_HZ)) {
		fail |= PBIT_IMU_RATE;
	}
	if (!rate_ok(in->gnss_count, in->window_us, GNSS_HZ)) {
		fail |= PBIT_GNSS_RATE;
	}
	if (!rate_ok(in->baro_count, in->window_us, BARO_HZ)) {
		fail |= PBIT_BARO_RATE;
	}
	if (fabsf(in->accel_norm - GRAVITY) > GRAVITY_TOL) {
		fail |= PBIT_GRAVITY;
	}
	if (in->relay_readback_open) {
		fail |= PBIT_RELAY;
	}
	if (in->chute_readback_fired) {
		fail |= PBIT_CHUTE;
	}
	return fail;
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cmake -S tests/host -B build/host >/dev/null && cmake --build build/host >/dev/null && ctest --test-dir build/host --output-on-failure`
Expected: `100% tests passed, 0 tests failed out of 5`.

- [ ] **Step 5: Commit**

```bash
git add fts/src/logic/pbit.h fts/src/logic/pbit.c tests/host
git commit -m "feat: add FTS self-test checks" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

## After this plan

Write plan 2 from `docs/feasibility.md`: Zephyr app on `native_sim` (two UARTs on PTYs, 100 Hz tick, PBIT window, status at 10 Hz), the Gazebo I/O bridge (sensor feed, gated motors, parachute drag and canopy, MAVLink heartbeat, fault hooks), the drone model and world, the ROS 2 ground station and test runner, scenario runs and `docs/test-log.md`.
