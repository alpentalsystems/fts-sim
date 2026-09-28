#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "app.h"
#include "fts/proto.h"

/* Link 1: bridge (sensors, heartbeat, outputs). Link 2: ground station. */
static const struct device *const link1 = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct device *const link2 = DEVICE_DT_GET(DT_NODELABEL(uart1));

static const char *const state_names[] = {"PBIT", "SAFE", "ARMED", "TERMINATED", "FAULT"};
static const char *const cause_names[] = {"NONE", "FENCE", "CEILING", "CONTROL",
					  "AP_FREEZE", "LINK", "MANUAL", "GNSS_LOST"};

static struct app app;
static struct app_tx tx;
static struct fts_parser parser1;
static struct fts_parser parser2;

typedef void (*handler_t)(struct app *a, const struct fts_frame *f, struct app_tx *tx);

static void send(const struct device *dev, const uint8_t *buf, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		uart_poll_out(dev, buf[i]);
	}
}

static void flush(void)
{
	send(link1, tx.link1, tx.n1);
	send(link2, tx.link2, tx.n2);
	tx.n1 = 0U;
	tx.n2 = 0U;
}

static void drain(const struct device *dev, struct fts_parser *p, handler_t on_frame)
{
	unsigned char c;
	struct fts_frame f;

	while (uart_poll_in(dev, &c) == 0) {
		if (!fts_parser_feed(p, c, &f)) {
			continue;
		}
		do {
			on_frame(&app, &f, &tx);
			flush();
		} while (fts_parser_next(p, &f));
	}
}

static void report(enum fts_state *last)
{
	const struct fts *s = &app.fts;

	if (s->state == *last) {
		return;
	}
	printk("FTS %s -> %s", state_names[*last], state_names[s->state]);
	if (s->state == FTS_TERMINATED) {
		printk(" cause %s t_trigger_us %lld", cause_names[s->cause], (long long)s->t_trigger_us);
	} else if (s->state == FTS_FAULT) {
		printk(" pbit_fail 0x%02x", s->pbit_fail);
	}
	printk("\n");
	*last = s->state;
}

int main(void)
{
	struct fts_config cfg;
	enum fts_state last = FTS_PBIT;

	if (!device_is_ready(link1) || !device_is_ready(link2)) {
		printk("FTS links not ready\n");
		return -1;
	}
	app_demo_config(&cfg);
	app_init(&app, &cfg);
	fts_parser_init(&parser1);
	fts_parser_init(&parser2);
	printk("FTS started\n");
	for (;;) {
		drain(link1, &parser1, app_on_link1);
		drain(link2, &parser2, app_on_link2);
		report(&last);
		k_sleep(K_MSEC(1));
	}
	return 0;
}
