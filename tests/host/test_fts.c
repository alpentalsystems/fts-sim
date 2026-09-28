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
	fts_tick(&s, 1 * S + 900 * MS); /* last heartbeat at 0.9 s: exactly 1.0 s old */
	CHECK(s.state == FTS_ARMED);
	fts_tick(&s, 1 * S + 900 * MS + 1);
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

static void test_clock_rewind_does_not_trip_timeouts(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c); /* last heartbeat 0.9 s, last command 1 s */
	fts_tick(&s, 500 * MS); /* clock jumped back, no new frames yet */
	CHECK(s.state == FTS_ARMED);
	fts_tick(&s, 1 * S + 500 * MS); /* 1.0 s after the rewind */
	CHECK(s.state == FTS_ARMED);
	fts_tick(&s, 1 * S + 500 * MS + 1);
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_AP_FREEZE);
}

static void test_terminate_arm_from_the_future_is_rejected(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	CHECK(fts_on_command(&s, 2 * S, FTS_CMD_TERMINATE_ARM));
	CHECK(!fts_on_command(&s, 1 * S + 500 * MS, FTS_CMD_TERMINATE));
	CHECK(s.state == FTS_ARMED);
}

static void test_terminate_cannot_be_hidden_by_disarm(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	(void)fts_on_command(&s, 1 * S, FTS_CMD_TERMINATE_ARM);
	CHECK(fts_on_command(&s, 1 * S, FTS_CMD_TERMINATE));
	CHECK(!fts_on_command(&s, 1 * S, FTS_CMD_DISARM));
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_MANUAL);
	CHECK(fts_relay_open(&s) && s.t_relay_us == 1 * S);
}

static void test_parachute_fires_after_clock_rewind(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	(void)fts_on_command(&s, 2 * S, FTS_CMD_TERMINATE_ARM);
	(void)fts_on_command(&s, 2 * S, FTS_CMD_TERMINATE);
	fts_tick(&s, 2 * S);
	fts_tick(&s, 0); /* clock reset before the parachute fired */
	CHECK(!fts_chute_fire(&s));
	fts_tick(&s, 300 * MS);
	CHECK(fts_chute_fire(&s));
}

static void test_gnss_altitude_without_fix_is_ignored(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c); /* last fix at 0.9 s; stay under the 1.0 s GNSS timeout */
	for (int64_t t = 1 * S; t < 1 * S + 800 * MS; t += 10 * MS) {
		fts_on_imu(&s, t, LEVEL, STILL);
		if ((t % (100 * MS)) == 0) {
			fts_on_gnss(&s, t, 375665000, 1269780000, 1000000, 0U);
			fts_on_heartbeat(&s, t);
			fts_on_baro(&s, t, 101325.0f);
			(void)fts_on_command(&s, t, FTS_CMD_PING);
		}
		fts_tick(&s, t);
	}
	CHECK(s.state == FTS_ARMED);
}

static void test_arm_rejects_stale_gnss_or_baro(void)
{
	struct fts_config c;
	struct fts s;

	setup(&s, &c);
	healthy(&s, 0, 1 * S, 0); /* GNSS and baro stop after 0.9 s */
	for (int64_t t = 1 * S; t <= 3 * S; t += 100 * MS) {
		fts_on_heartbeat(&s, t);
		fts_tick(&s, t);
	}
	CHECK(!fts_on_command(&s, 3 * S, FTS_CMD_ARM));
	CHECK(s.state == FTS_SAFE);
}

/* Feeds healthy inputs except GNSS, which reports the given fix. */
static void gnss_fix(struct fts *s, int64_t t0, int64_t t1, uint8_t fix)
{
	for (int64_t t = t0; t < t1; t += 10 * MS) {
		fts_on_imu(s, t, LEVEL, STILL);
		if ((t % (100 * MS)) == 0) {
			fts_on_gnss(s, t, 375665000, 1269780000, 30000, fix);
			fts_on_heartbeat(s, t);
			fts_on_baro(s, t, 101325.0f);
			(void)fts_on_command(s, t, FTS_CMD_PING);
		}
		fts_tick(s, t);
	}
}

static void test_gnss_fix_lost_terminates(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c); /* last fix at 0.9 s */
	gnss_fix(&s, 1 * S, 1 * S + 910 * MS, 0U); /* through 1.9 s */
	CHECK(s.state == FTS_ARMED);
	gnss_fix(&s, 1 * S + 910 * MS, 2 * S, 0U);
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_GNSS_LOST);
}

static void test_gnss_silence_terminates(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	for (int64_t t = 1 * S; t < 2 * S; t += 10 * MS) {
		fts_on_imu(&s, t, LEVEL, STILL);
		fts_on_heartbeat(&s, t);
		fts_tick(&s, t);
	}
	CHECK(s.state == FTS_TERMINATED && s.cause == FTS_CAUSE_GNSS_LOST);
}

static void test_short_fix_dropout_does_not_terminate(void)
{
	struct fts_config c;
	struct fts s;

	armed(&s, &c);
	gnss_fix(&s, 1 * S, 1 * S + 800 * MS, 0U);
	gnss_fix(&s, 1 * S + 800 * MS, 4 * S, 3U);
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
	test_clock_rewind_does_not_trip_timeouts();
	test_terminate_arm_from_the_future_is_rejected();
	test_terminate_cannot_be_hidden_by_disarm();
	test_parachute_fires_after_clock_rewind();
	test_gnss_altitude_without_fix_is_ignored();
	test_arm_rejects_stale_gnss_or_baro();
	test_gnss_fix_lost_terminates();
	test_gnss_silence_terminates();
	test_short_fix_dropout_does_not_terminate();
	return CHECK_DONE();
}
