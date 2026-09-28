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
