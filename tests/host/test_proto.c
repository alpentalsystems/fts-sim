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
