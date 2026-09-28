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
