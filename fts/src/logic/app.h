#ifndef APP_H_
#define APP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fts.h"
#include "fts/proto.h"
#include "pbit.h"

#define APP_PBIT_WINDOW_US 1000000
/* STATUS and periodic OUTPUT every N IMU ticks (10 Hz at 100 Hz IMU). */
#define APP_REPORT_TICKS 10U
#define APP_TX_MAX 128U

/* Bytes to send after one input frame. */
struct app_tx {
	uint8_t link1[APP_TX_MAX];
	size_t n1;
	uint8_t link2[APP_TX_MAX];
	size_t n2;
};

struct app {
	struct fts fts;
	int64_t t_now_us; /* newest sensor time; -1 before the first */
	int64_t pbit_start_us;
	struct pbit_input pbit;
	double accel_sum;
	uint32_t accel_n;
	bool readback_seen;
	struct fts_readback readback;
	uint32_t tick_count;
	bool last_relay;
	bool last_chute;
	enum fts_state last_state;
	uint32_t rx_unknown;
	uint32_t tx_drops;
};

/* Demo site: PX4 default home as origin, +-100 m fence, 40 m ceiling. */
void app_demo_config(struct fts_config *c);
void app_init(struct app *a, const struct fts_config *c);

/* Link 1: sensors, heartbeat and readback in; OUTPUT and STATUS out. */
void app_on_link1(struct app *a, const struct fts_frame *f, struct app_tx *tx);

/* Link 2: ground station commands in; STATUS out. */
void app_on_link2(struct app *a, const struct fts_frame *f, struct app_tx *tx);

#endif /* APP_H_ */
