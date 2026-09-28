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
