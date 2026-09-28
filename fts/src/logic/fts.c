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
