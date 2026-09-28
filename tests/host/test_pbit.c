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
