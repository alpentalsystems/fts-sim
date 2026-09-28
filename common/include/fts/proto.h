#ifndef FTS_PROTO_H_
#define FTS_PROTO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FTS_SYNC 0xA5U
#define FTS_MAX_PAYLOAD 48U
/* sync, type, len, crc (2) */
#define FTS_FRAME_OVERHEAD 5U
#define FTS_FRAME_MAX (FTS_FRAME_OVERHEAD + FTS_MAX_PAYLOAD)

#define FTS_MSG_IMU 1U
#define FTS_MSG_GNSS 2U
#define FTS_MSG_BARO 3U
#define FTS_MSG_AP_HEARTBEAT 4U
#define FTS_MSG_GS_CMD 5U
#define FTS_MSG_OUTPUT 6U
#define FTS_MSG_READBACK 7U
#define FTS_MSG_STATUS 8U

#define FTS_CMD_PING 0U
#define FTS_CMD_ARM 1U
#define FTS_CMD_DISARM 2U
#define FTS_CMD_TERMINATE_ARM 3U
#define FTS_CMD_TERMINATE 4U

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection. */
uint16_t fts_crc16(const uint8_t *data, size_t len);

struct fts_frame {
	uint8_t type;
	uint8_t len;
	uint8_t payload[FTS_MAX_PAYLOAD];
};

/* Returns the frame length, or 0 if the payload or buffer is invalid. */
size_t fts_frame_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out,
			size_t out_size);

#define FTS_PARSER_QUEUE 4U

struct fts_parser {
	uint8_t buf[FTS_FRAME_MAX];
	uint8_t n;
	struct fts_frame queue[FTS_PARSER_QUEUE];
	uint8_t q_head;
	uint8_t q_count;
	uint32_t crc_errors;
	uint32_t len_errors;
	uint32_t queue_overflows;
};

void fts_parser_init(struct fts_parser *p);

/*
 * Feeds one byte; returns true when *out holds a valid frame. A bad
 * candidate frame drops only its sync byte, so frames after a false sync are
 * still found. Call fts_parser_next() until false for further frames.
 */
bool fts_parser_feed(struct fts_parser *p, uint8_t byte, struct fts_frame *out);
bool fts_parser_next(struct fts_parser *p, struct fts_frame *out);

struct fts_imu {
	uint64_t t_us;
	float q[4]; /* w, x, y, z: body to world */
	float gyro[3];
	float accel[3];
};

struct fts_gnss {
	uint64_t t_us;
	int32_t lat_e7;
	int32_t lon_e7;
	int32_t alt_mm;
	uint8_t fix; /* 0 none, 3 3D */
	uint8_t sats;
};

struct fts_baro {
	uint64_t t_us;
	float pressure_pa;
	float temp_c;
};

struct fts_ap_heartbeat {
	uint64_t t_us;
	uint32_t seq;
};

struct fts_gs_cmd {
	uint32_t seq;
	uint8_t cmd;
};

struct fts_output {
	uint8_t relay_open;
	uint8_t chute_fire;
};

struct fts_readback {
	uint64_t t_us;
	uint8_t relay_open;
	uint8_t chute_fired;
};

struct fts_status {
	uint64_t t_us;
	uint8_t state;
	uint8_t cause;
	uint8_t pbit_fail;
	uint64_t t_trigger_us;
	uint64_t t_relay_us;
	uint64_t t_chute_us;
};

size_t fts_encode_imu(const struct fts_imu *m, uint8_t *out, size_t out_size);
size_t fts_encode_gnss(const struct fts_gnss *m, uint8_t *out, size_t out_size);
size_t fts_encode_baro(const struct fts_baro *m, uint8_t *out, size_t out_size);
size_t fts_encode_ap_heartbeat(const struct fts_ap_heartbeat *m, uint8_t *out, size_t out_size);
size_t fts_encode_gs_cmd(const struct fts_gs_cmd *m, uint8_t *out, size_t out_size);
size_t fts_encode_output(const struct fts_output *m, uint8_t *out, size_t out_size);
size_t fts_encode_readback(const struct fts_readback *m, uint8_t *out, size_t out_size);
size_t fts_encode_status(const struct fts_status *m, uint8_t *out, size_t out_size);

/* Return 0 on success, -1 if the frame type or length does not match. */
int fts_decode_imu(const struct fts_frame *f, struct fts_imu *m);
int fts_decode_gnss(const struct fts_frame *f, struct fts_gnss *m);
int fts_decode_baro(const struct fts_frame *f, struct fts_baro *m);
int fts_decode_ap_heartbeat(const struct fts_frame *f, struct fts_ap_heartbeat *m);
int fts_decode_gs_cmd(const struct fts_frame *f, struct fts_gs_cmd *m);
int fts_decode_output(const struct fts_frame *f, struct fts_output *m);
int fts_decode_readback(const struct fts_frame *f, struct fts_readback *m);
int fts_decode_status(const struct fts_frame *f, struct fts_status *m);

#ifdef __cplusplus
}
#endif

#endif /* FTS_PROTO_H_ */
