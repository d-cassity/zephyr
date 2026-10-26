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
 * ---------------------------------------------------------------------- */

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

RTIO_DEFINE_WITH_MEMPOOL(iwrl6322_ctx, //TODO CHANGE
			 /* sq_sz   */ 4,
			 /* cq_sz   */ 4,
			 /* num_blks */ RTIO_NUM_BLOCKS,
			 /* blk_size */ RTIO_BLOCK_SIZE,
			 /* align    */ sizeof(void *));

/* -------------------------------------------------------------------------
 * Sensor configuration
 * ---------------------------------------------------------------------- */

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

// static int configure_sensor(const struct device *dev)
// {
// 	struct sensor_value v;
// 	int ret;

// 	/*
// 	 * Max range index — NOT a distance in metres.
// 	 *   0 = 4 m  (default)
// 	 *   1 = 8 m
// 	 *   2 = 12 m
// 	 * Maps to c_MaxRange in the IWRL6322 firmware MPD configuration.
// 	 */
// 	v.val1 = 0; /* 4 m */
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      SENSOR_ATTR_FULL_SCALE, &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set max range index: %d", ret);
// 		return ret;
// 	}

// 	/*
// 	 * Update rate in millihertz.
// 	 * 1000 mHz = 1 Hz → 1000 ms MPD frame period.
// 	 */
// 	v.val1 = 1000;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set update rate: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 0;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_MOTION_SENSITIVITY,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set motion sensitivity: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 0;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_DETECTION_SENSITIVITY,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set detection sensitivity: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 0; /* IWRL6322_REGION_FCC_CE */
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_REGION,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set region: %d", ret);
// 		return ret;
// 	}

// 	/*
// 	 * MPD measurement profile selection.
// 	 * 0xAA = M_ML_SENS_MPD_MEAS_AUTO — firmware picks the best profile.
// 	 * Use a fixed index (0–3) to select a specific profile.
// 	 */
// 	v.val1 = 0x0; /* option 0*/
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_MPD_PROF_SEL,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set MPD profile: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 1; /* disabled */
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_RUN_CAL_CTRL,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set boot calib: %d", ret);
// 		return ret;
// 	}

// 	/* ------------------------------------------------------------------
// 	 * App config fields
// 	 * AOP/Module antenna (0), holdtime 1000×10us, no FE calib override.
// 	 * ------------------------------------------------------------------ */
// 	v.val1 = 0;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_ANTENNA_TYPE,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set antenna type: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 1000;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_MPD_HOLDTIME_10US,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set MPD holdtime: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 0;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_CALIB_CTRL,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE calib ctrl: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 1;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_DEEP_SLEEP_ENTRY_DIS,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set deep sleep entry dis: %d", ret);
// 		return ret;
// 	}

// 	/* ------------------------------------------------------------------
// 	 * FE chirp profile
// 	 *
// 	 *  c_AdcSampRate=16        → 12.5 MSPS  (400MHz / (16*2))
// 	 *  c_NumVirtualAntennas=4
// 	 *  h_ChirpTxRxChEnaSel=0xA965
// 	 *  h_NumOfAdcSamples=128
// 	 *  c_ChirpAdcStartSkipSamp=54
// 	 *  h_ChirpRampEndTime=246  → 24.6 µs
// 	 *  h_ChirpIdleTime=531     → 53.1 µs
// 	 *  w_ChirpRfFreqStart=0xBE00 → ~60 GHz
// 	 *  xh_ChirpRfFreqSlope=4912  → ~140 MHz/µs up-sweep
// 	 *  xh_TxStartTime=1        → 20 ns
// 	 *  c_ChirpRxHpfSel=1       → 350 kHz HPF
// 	 *  c_CalRxGainSel=42       → 42 dB
// 	 *  c_CalTxBackOffSelOvr=0  → 0 dB back-off
// 	 * ------------------------------------------------------------------ */
// 	v.val1 = 16;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_ADC_SAMP_RATE,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE ADC samp rate: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 4;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_NUM_VIRTUAL_ANT,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE num virtual ant: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 0xA965;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_TXRX_CH_ENA,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE TxRx ch ena: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 128;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_NUM_ADC_SAMPLES,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE num ADC samples: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 54;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_ADC_SKIP_SAMP,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE ADC skip samp: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 246;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_RAMP_END_TIME,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE ramp end time: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 531;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_IDLE_TIME,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE idle time: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 0x0000BE00;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_RF_FREQ_START,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE RF freq start: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 4912;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_RF_FREQ_SLOPE,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE RF freq slope: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 1;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_TX_START_TIME,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE TX start time: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 1;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_HPF_CORNER,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE HPF corner: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 42;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_CAL_RX_GAIN,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE cal RX gain: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 0;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_CAL_TX_BACKOFF,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE cal TX backoff: %d", ret);
// 		return ret;
// 	}

// 	/* ------------------------------------------------------------------
// 	 * FE chirp frame
// 	 *
// 	 *  c_NumOfChirpsAccum=0    → accumulation disabled
// 	 *  h_NumOfChirpsInBurst=4
// 	 *  w_BurstIdleTime=8011    → 801.1 µs between bursts
// 	 *  h_NumOfBurstsInFrame=4
// 	 *  h_NumOfFrames=0         → infinite
// 	 *  w_FramePeriodicity=500  → 500 ms
// 	 * ------------------------------------------------------------------ */
// 	v.val1 = 0;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_CHIRPS_ACCUM,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE chirps accum: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 4;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_CHIRPS_PER_BURST,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE chirps per burst: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 8011;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_BURST_IDLE_TIME,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE burst idle time: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 4;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_BURSTS_PER_FRAME,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE bursts per frame: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 0;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_NUM_FRAMES,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE num frames: %d", ret);
// 		return ret;
// 	}

// 	v.val1 = 200;
// 	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
// 			      (enum sensor_attribute)IWRL6322_ATTR_FE_FRAME_PERIOD_MS,
// 			      &v);
// 	if (ret < 0) {
// 		LOG_ERR("Failed to set FE frame period: %d", ret);
// 		return ret;
// 	}

// 	return 0;
// }


/* -------------------------------------------------------------------------
 * main
 * ---------------------------------------------------------------------- */

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(iwrl6322)); //TODO CHANGE
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

	// ret = configure_sensor(dev);
	// if (ret < 0) {
	// 	return ret;
	// }

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
	 * ------------------------------------------------------------------ */
	ret = sensor_stream(&iwrl6322_iodev, &iwrl6322_ctx, NULL, NULL); //TODO CHANGE
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
	 * ------------------------------------------------------------------ */
	while (1) {
		struct rtio_cqe *cqe;
		uint8_t *buf;
		uint32_t buf_len;

			cqe = rtio_cqe_consume(&iwrl6322_ctx); //TODO CHANGE
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
				rtio_cqe_release(&iwrl6322_ctx, cqe); //TODO CHANGE
				continue;
		}

		/* Extract the buffer pointer from the CQE. */
		ret = rtio_cqe_get_mempool_buffer(&iwrl6322_ctx, cqe, //TODO CHANGE
						  &buf, &buf_len);
		/* CQE is no longer needed once we have the buffer. */
		rtio_cqe_release(&iwrl6322_ctx, cqe); //TODO CHANGE

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
