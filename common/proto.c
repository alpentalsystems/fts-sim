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
