/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * TI mmWave IWRL6322 radar sensor sample — sensor_stream path.
 *
 * Requires CONFIG_SENSOR_ASYNC_API=y (see prj.conf).
 *
 * The driver's GPIO ISR workqueue handler fires on each rising edge of the
 * PRESENCE_DETECT pulse.  It writes the raw MPD point-cloud payload
 * directly into the RTIO pool buffer (zero-copy, no memcpy) and signals
 * the multishot SQE.  RTIO automatically re-arms the SQE for the next
 * trigger.
 *
 * This application uses the iwrl6322_decoder_api extension to decode each
 * buffer without an extra copy:
 *   dec->get_sensor_info(buf, buf_len, &info) — validates buf, copies scalar
 *   metadata, and sets info.points to a pointer directly into the RTIO buffer.
 *
 * The IWRL6322 driver is RTIO-only.  sensor_sample_fetch / sensor_channel_get
 * are not supported — use sensor_stream / sensor_read instead.
 *
 * Configuration
 * ─────────────
 *   • Operating mode: MPD Verbose (reports every frame)
 *   • Max range index: 0 (4 m)  ← index, not metres; 1=8m, 2=12m
 *   • Update rate: 1 Hz (1000 ms frame period)
 *   • Motion sensitivity: 0 (default)
 *   • Detection sensitivity: 0 (default)
 *   • MPD profile: auto-select (0xAA)
 *   • Region: FCC / CE (default)
 *   • Boot calibration: disabled
 *
 * One detection zone
 * ──────────────────
 *   Zone 0:  X ±150 cm, Y 0–400 cm, Z 0–100 cm  (INCLUDE)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

/* iwrl6322.h provides IWRL6322_MODE_* and IWRL6322_ATTR_* */
#include "iwrl6322.h"

LOG_MODULE_REGISTER(iwrl6322_sample, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------
 * RTIO context and streaming iodev
 *
 * SENSOR_DT_STREAM_IODEV creates an rtio_iodev that sets is_streaming=true.
 * The driver stores the multishot SQE and completes it on every DATA_READY
 * GPIO rising edge; RTIO re-arms the SQE automatically.
 *
 * RTIO_DEFINE_WITH_MEMPOOL allocates a pool of payload blocks. Each slot holds
 * one [iwrl6322_buffer_header + variant-specific payload]. On the 80 KiB
 * cc1352p1_launchxl we can keep four blocks for the small default payload, but
 * a 32 KiB payload only fits as a single in-flight block.
 * -------------------------------------------------------------------------
 */

SENSOR_DT_STREAM_IODEV(iwrl6322_iodev,
		       DT_NODELABEL(iwrl6322),
		       SENSOR_STREAM_TRIGGER_PREP(SENSOR_TRIG_DATA_READY,
						  SENSOR_STREAM_DATA_INCLUDE));

#define IWRL6322_PAYLOAD_SIZE CONFIG_IWRL6322_MAX_PAYLOAD_BYTES 
#define IWRL6322_BUFFER_SIZE (sizeof(struct iwrl6322_buffer_header) + IWRL6322_PAYLOAD_SIZE)
#define RTIO_BLOCK_SIZE NHPOT(IWRL6322_BUFFER_SIZE)
#if IWRL6322_PAYLOAD_SIZE >= 32768
#define RTIO_NUM_BLOCKS 1
#else
#define RTIO_NUM_BLOCKS 4
#endif

static const uint8_t fe_raw_prefix[] = "DATA: ";
static const struct device *const fe_raw_uart =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static const char base64_table[] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

RTIO_DEFINE_WITH_MEMPOOL(iwrl6322_ctx,
			 /* sq_sz   */ 4,
			 /* cq_sz   */ 4,
			 /* num_blks */ RTIO_NUM_BLOCKS,
			 /* blk_size */ RTIO_BLOCK_SIZE,
			 /* align    */ sizeof(void *));

/* -------------------------------------------------------------------------
 * Sensor configuration
 * -------------------------------------------------------------------------
 */

static void uart_write_base64(const uint8_t *src, size_t len)
{
	size_t i = 0;

	while ((i + 3U) <= len) {
		const uint8_t a = src[i++];
		const uint8_t b = src[i++];
		const uint8_t c = src[i++];
		const uint32_t triple = ((uint32_t)a << 16) |
					((uint32_t)b << 8) |
					(uint32_t)c;

		uart_poll_out(fe_raw_uart, base64_table[(triple >> 18) & 0x3FU]);
		uart_poll_out(fe_raw_uart, base64_table[(triple >> 12) & 0x3FU]);
		uart_poll_out(fe_raw_uart, base64_table[(triple >> 6) & 0x3FU]);
		uart_poll_out(fe_raw_uart, base64_table[triple & 0x3FU]);
	}

	if (i < len) {
		const size_t remaining = len - i;
		const uint8_t a = src[i++];
		const uint8_t b = (remaining == 2U) ? src[i++] : 0U;
		const uint32_t triple = ((uint32_t)a << 16) |
					((uint32_t)b << 8);

		uart_poll_out(fe_raw_uart, base64_table[(triple >> 18) & 0x3FU]);
		uart_poll_out(fe_raw_uart, base64_table[(triple >> 12) & 0x3FU]);
		if (remaining == 2U) {
			uart_poll_out(fe_raw_uart, base64_table[(triple >> 6) & 0x3FU]);
		} else {
			uart_poll_out(fe_raw_uart, '=');
		}
		uart_poll_out(fe_raw_uart, '=');
	}
}

static void print_fe_raw_base64(const struct iwrl6322_sensor_info *info)
{
	if (!device_is_ready(fe_raw_uart)) {
		LOG_ERR("console UART not ready");
		return;
	}

	for (size_t i = 0; i < sizeof(fe_raw_prefix) - 1U; i++) {
		uart_poll_out(fe_raw_uart, fe_raw_prefix[i]);
	}
	uart_write_base64((const uint8_t *)info->raw_data, info->raw_data_len);
	uart_poll_out(fe_raw_uart, '\r');
	uart_poll_out(fe_raw_uart, '\n');
}

#define ZONE_SET(dev, attr, zone, value, label)                         \
	do {                                                            \
		struct sensor_value _v = { .val1 = (zone),              \
					   .val2 = (value) };           \
		int _r = sensor_attr_set((dev), SENSOR_CHAN_ALL,        \
			(enum sensor_attribute)(attr), &_v);            \
		if (_r < 0) {                                           \
			LOG_ERR(label ": %d", _r);                      \
			return _r;                                      \
		}                                                       \
	} while (0)

/* -------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------
 */

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(iwrl6322));
	struct sensor_value attr_val;
	int ret;
	uint32_t last_wait_ms = 0;

	if (dev == NULL || !device_is_ready(dev)) {
		LOG_ERR("IWRL6322 not ready");
		return -ENODEV;
	}

	LOG_INF("IWRL6322 \"%s\" ready", dev->name);

	ret = sensor_attr_get(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_CHIP_ID, &attr_val);
	if (ret == 0) {
		uint32_t ver = (uint32_t)attr_val.val1;

		LOG_INF("uDFP firmware: %u.%u.%u.%u",
			(ver >> 24) & 0xFF, (ver >> 16) & 0xFF,
			(ver >>  8) & 0xFF,  ver         & 0xFF);
	}
	else{
		LOG_ERR("No uDFP Firmware Found");
	}

	/* Explicit stop → start sequence. */
	attr_val.val1 = IWRL6322_MODE_STOP;
	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
			      (enum sensor_attribute)IWRL6322_ATTR_OPERATING_MODE,
			      &attr_val);
	if (ret < 0) {
		LOG_ERR("Failed to stop sensor: %d", ret);
		return ret;
	}

	/* ------------------------------------------------------------------
	 * Start the stream.
	 *
	 * sensor_stream() submits a multishot SQE.  The driver stores it and
	 * completes it from the GPIO ISR workqueue on each DATA_READY rising
	 * edge, writing the sensor frame directly into the RTIO mempool buffer.
	 * RTIO re-arms the SQE automatically after each completion.
	 * ------------------------------------------------------------------
	 */
	ret = sensor_stream(&iwrl6322_iodev, &iwrl6322_ctx, NULL, NULL);
	if (ret < 0) {
		LOG_ERR("sensor_stream failed: %d", ret);
		return ret;
	}

	LOG_INF("Stream started — waiting for DATA_READY triggers...");

	attr_val.val1 = IWRL6322_MODE_RAW_ADC;
	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
			      (enum sensor_attribute)IWRL6322_ATTR_OPERATING_MODE,
			      &attr_val);
	if (ret < 0) {
		LOG_ERR("Failed to start sensor: %d", ret);
		return ret;
	}
	LOG_INF("Sensor started in MPD verbose mode");

	/* ------------------------------------------------------------------
	 * Main loop — consume completed CQEs as they arrive.
	 * ------------------------------------------------------------------
	 */
	while (1) {
		struct rtio_cqe *cqe;
		uint8_t *buf;
		uint32_t buf_len;

			cqe = rtio_cqe_consume(&iwrl6322_ctx);
			if (cqe == NULL) {
				uint32_t now = k_uptime_get_32();

				if ((now - last_wait_ms) >= 1000U) {
					last_wait_ms = now;
				}

				k_msleep(10);
				continue;
			}

			if (cqe->result < 0) {
				LOG_ERR("stream error: %d", cqe->result);
				rtio_cqe_release(&iwrl6322_ctx, cqe);
				continue;
		}

		/* Extract the buffer pointer from the CQE. */
		ret = rtio_cqe_get_mempool_buffer(&iwrl6322_ctx, cqe,
						  &buf, &buf_len);
		/* CQE is no longer needed once we have the buffer. */
		rtio_cqe_release(&iwrl6322_ctx, cqe);

			if (ret < 0) {
				LOG_ERR("get_mempool_buffer failed: %d", ret);
				continue;
			}

		/*
		 * Decode — zero-copy: info.points points directly into buf.
		 * Valid until rtio_release_buffer() below.
		 */
		struct iwrl6322_sensor_info info;

			ret = iwrl6322_decoder_get_sensor_info(buf, buf_len, &info);
			if (ret < 0) {
				LOG_ERR("get_sensor_info failed: %d", ret);
			} else if (info.raw_data != NULL) {
				/* RAW_ADC or 1D_FFT mode: emit one parser-friendly base64 row. */
				print_fe_raw_base64(&info);
			} else {
				LOG_INF("Frame %u: %d detection(s)  zone0=%d  zone1=%d",
				(unsigned int)info.frame_count,
				(int)info.num_detections,
				(info.zone_occupancy >> 0) & 1,
				(info.zone_occupancy >> 1) & 1);

			for (uint8_t i = 0; i < info.num_detections; i++) {
				const struct iwrl6322_point_data *pt =
					&info.points[i];

				LOG_INF("  [%u] X=%d cm  Y=%u cm  Z=%d cm"
					"  vel=%d  snr=%d.%d dB",
					(uint32_t)i,
					(int)pt->x_cm,
					(unsigned int)pt->y_cm,
					(int)pt->z_cm,
					(int)pt->velocity,
					pt->snr_01db / 10,
					(int)(pt->snr_01db >= 0
					      ? pt->snr_01db % 10
					      : -(pt->snr_01db % 10)));
			}
		}

		/* Return the buffer to the RTIO mempool. */
		rtio_release_buffer(&iwrl6322_ctx, buf, buf_len);
	}

	return 0;
}
