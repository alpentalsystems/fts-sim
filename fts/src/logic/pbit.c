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
