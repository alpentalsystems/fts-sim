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
