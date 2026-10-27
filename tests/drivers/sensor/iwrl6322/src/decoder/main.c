/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 *
 * IWRL6322 decoder unit tests.
 *
 * These tests exercise iwrl6322_decoder.c directly — no hardware, no TI
 * library runtime, no SPI or GPIO.  They construct synthetic RTIO buffers
 * in the format produced by the driver and verify every code path in the
 * decoder API:
 *
 *   - get_frame_count()  — MPD and FE modes, bad magic, truncated buffers
 *   - get_size_info()    — supported and unsupported channel specs
 *   - decode()           — zero-copy pointer, frame iterator, batching
 *   - get_sensor_info()  — MPD and FE helper, boundary conditions
 *
 * Buffer layout (same as the live driver):
 *   [struct iwrl6322_buffer_header]
 *   [struct iwrl6322_pc_payload_hdr]   — MPD modes only
 *   [T_ML_SENS_POINT_CLOUD_DATA × N]   — MPD modes only
 *
 * The pc_payload_hdr type is local to iwrl6322_decoder.c; we re-declare
 * a layout-identical helper here so that the test can build well-formed
 * buffers without modifying the driver source.
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#include <stdint.h>

#include "iwrl6322.h"   /* public driver header — enums, structs, API */

/* -------------------------------------------------------------------------
 * Local re-declaration of the MPD payload header.
 *
 * This struct is defined statically inside iwrl6322_decoder.c.  Its layout
 * is stable and documented in the decoder source; we duplicate it here
 * solely to construct valid test buffers.  A mismatch would be caught by a
 * test failure on the frame count or point data values.
 * ------------------------------------------------------------------------
 */
struct pc_payload_hdr {
	uint32_t w_FrameCount;
	uint8_t  c_NumDetPoints;
	uint8_t  c_ZoneOccupancy;
	uint16_t h_Reserved;
};

/* -------------------------------------------------------------------------
 * Helpers to build synthetic RTIO buffers
 * -------------------------------------------------------------------------
 */

/**
 * @brief Fill an iwrl6322_buffer_header in @p buf at offset 0.
 *
 * @param buf           Destination buffer
 * @param operating_mode One of enum iwrl6322_operating_mode
 * @param payload_size  Bytes of payload that follow the header
 * @param timestamp_us  Fake timestamp
 */
static void fill_header(uint8_t *buf,
			uint8_t operating_mode,
			uint16_t payload_size,
			uint32_t timestamp_us)
{
	struct iwrl6322_buffer_header *hdr = (struct iwrl6322_buffer_header *)buf;

	hdr->magic          = IWRL6322_BUFFER_MAGIC;
	hdr->operating_mode = operating_mode;
	hdr->reserved1      = 0;
	hdr->payload_size   = payload_size;
	hdr->reserved2      = 0;
	hdr->timestamp_us   = timestamp_us;
}

/**
 * @brief Build a complete MPD buffer with @p num_pts synthetic targets.
 *
 * @param buf           Destination buffer (must be large enough)
 * @param operating_mode IWRL6322_MODE_MPD_PRESENCE or IWRL6322_MODE_MPD_VERBOSE
 * @param frame_count   Rolling frame counter placed in the pc_payload_hdr
 * @param zone_occ      Zone occupancy bitmap
 * @param num_pts       Number of T_ML_SENS_POINT_CLOUD_DATA entries to append
 * @return Total number of bytes written
 */
static size_t build_mpd_buf(uint8_t *buf,
			    uint8_t operating_mode,
			    uint32_t frame_count,
			    uint8_t zone_occ,
			    uint8_t num_pts)
{
	size_t payload_size = sizeof(struct pc_payload_hdr) +
			      num_pts * sizeof(struct iwrl6322_point_data);

	fill_header(buf, operating_mode, (uint16_t)payload_size, 1000U);

	struct pc_payload_hdr *pc =
		(struct pc_payload_hdr *)(buf + sizeof(struct iwrl6322_buffer_header));

	pc->w_FrameCount    = frame_count;
	pc->c_NumDetPoints  = num_pts;
	pc->c_ZoneOccupancy = zone_occ;
	pc->h_Reserved      = 0;

	/* Fill each point with recognisable synthetic values */
	struct iwrl6322_point_data *pts =
		(struct iwrl6322_point_data *)((uint8_t *)pc + sizeof(struct pc_payload_hdr));

	for (uint8_t i = 0; i < num_pts; i++) {
		pts[i].x_cm      = (int16_t)(10 * (i + 1));
		pts[i].y_cm      = (uint16_t)(20 * (i + 1));
		pts[i].z_cm      = (int16_t)(-5 * (i + 1));
		pts[i].velocity  = (int16_t)(i + 1);
		pts[i].snr_01db  = (int16_t)(100 + i);
		pts[i].noise_01db = (int16_t)(50 + i);
	}

	return sizeof(struct iwrl6322_buffer_header) + payload_size;
}

/**
 * @brief Build a complete FE (RAW_ADC or 1D_FFT) buffer with @p num_samples
 *        synthetic uint16_t samples.
 *
 * @return Total bytes written
 */
static size_t build_fe_buf(uint8_t *buf, uint8_t operating_mode,
			   uint16_t num_samples)
{
	uint16_t payload_size = num_samples * sizeof(uint16_t);

	fill_header(buf, operating_mode, payload_size, 2000U);

	uint16_t *samples =
		(uint16_t *)(buf + sizeof(struct iwrl6322_buffer_header));

	for (uint16_t i = 0; i < num_samples; i++) {
		samples[i] = i + 1;
	}

	return sizeof(struct iwrl6322_buffer_header) + payload_size;
}

/* -------------------------------------------------------------------------
 * Test suite setup
 * -------------------------------------------------------------------------
 */

ZTEST_SUITE(iwrl6322_decoder, NULL, NULL, NULL, NULL, NULL);

/* -------------------------------------------------------------------------
 * get_frame_count tests
 * -------------------------------------------------------------------------
 */

ZTEST(iwrl6322_decoder, test_get_frame_count_mpd_presence)
{
	/* 3 detected points in MPD_PRESENCE mode */
	uint8_t buf[256];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	uint16_t count = 0xFFFF;

	build_mpd_buf(buf, IWRL6322_MODE_MPD_PRESENCE, 42, 0x03, 3);

	/* Obtain decoder via public helper (no live device needed) */
	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_not_null(dec);
	zassert_ok(dec->get_frame_count(buf, cs, &count));
	zassert_equal(count, 3, "expected 3 detections, got %u", count);
}

ZTEST(iwrl6322_decoder, test_get_frame_count_mpd_verbose)
{
	uint8_t buf[512];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	uint16_t count = 0xFFFF;

	build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 1, 0xFF, 10);

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_ok(dec->get_frame_count(buf, cs, &count));
	zassert_equal(count, 10);
}

ZTEST(iwrl6322_decoder, test_get_frame_count_zero_detections)
{
	uint8_t buf[128];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	uint16_t count = 0xFFFF;

	build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 5, 0, 0);

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_ok(dec->get_frame_count(buf, cs, &count));
	zassert_equal(count, 0);
}

ZTEST(iwrl6322_decoder, test_get_frame_count_clamps_at_64)
{
	/*
	 * The decoder must clamp c_NumDetPoints to IWRL6322_MAX_DET_POINTS (64)
	 * even if the firmware returns a larger value.
	 */
	uint8_t buf[1024];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	uint16_t count = 0;

	/* Build with 64 real points; then hand-patch c_NumDetPoints to 100 */
	build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 1, 0, 64);

	struct pc_payload_hdr *pc =
		(struct pc_payload_hdr *)(buf + sizeof(struct iwrl6322_buffer_header));

	pc->c_NumDetPoints = 100; /* exceeds max */

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_ok(dec->get_frame_count(buf, cs, &count));
	zassert_true(count <= 64, "frame_count %u exceeds max 64", count);
}

ZTEST(iwrl6322_decoder, test_get_frame_count_fe_mode_returns_zero)
{
	/* FE modes have no structured frame count */
	/* 12-byte header + 64 samples × 2 bytes = 140 bytes — fits in 256 */
	uint8_t buf[256];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	uint16_t count = 0xFFFF;

	build_fe_buf(buf, IWRL6322_MODE_RAW_ADC, 64);

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_ok(dec->get_frame_count(buf, cs, &count));
	zassert_equal(count, 0);
}

ZTEST(iwrl6322_decoder, test_get_frame_count_bad_magic)
{
	uint8_t buf[256];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	uint16_t count = 0xFFFF;

	build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 1, 0, 2);

	/* Corrupt the magic number */
	struct iwrl6322_buffer_header *hdr = (struct iwrl6322_buffer_header *)buf;
	hdr->magic = 0xDEAD;

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_equal(dec->get_frame_count(buf, cs, &count), -EINVAL,
		      "bad magic must return -EINVAL");
}

ZTEST(iwrl6322_decoder, test_get_frame_count_wrong_chan)
{
	uint8_t buf[256];
	const struct sensor_decoder_api *dec;
	/* Any channel other than IWRL6322_CHAN_SENSOR_INFO must return -EINVAL */
	struct sensor_chan_spec cs = {
		.chan_type = SENSOR_CHAN_ACCEL_X,
		.chan_idx  = 0,
	};
	uint16_t count = 0xFFFF;

	build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 1, 0, 1);

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_equal(dec->get_frame_count(buf, cs, &count), -EINVAL);
}

/* -------------------------------------------------------------------------
 * get_size_info tests
 * -------------------------------------------------------------------------
 */

ZTEST(iwrl6322_decoder, test_get_size_info_sensor_info_chan)
{
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	size_t base = 0, frame = 0xDEAD;

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_ok(dec->get_size_info(cs, &base, &frame));
	zassert_equal(base, sizeof(struct iwrl6322_point_cloud_data));
	/*
	 * frame_size must be 0: readings is a zero-copy pointer into the RTIO
	 * buffer, so the output struct has fixed size regardless of point count.
	 */
	zassert_equal(frame, 0, "frame_size must be 0 for zero-copy decoder");
}

ZTEST(iwrl6322_decoder, test_get_size_info_unsupported_chan)
{
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = SENSOR_CHAN_GYRO_X,
		.chan_idx  = 0,
	};
	size_t base = 0, frame = 0;

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_equal(dec->get_size_info(cs, &base, &frame), -ENOTSUP);
}

/* -------------------------------------------------------------------------
 * decode() tests
 * -------------------------------------------------------------------------
 */

ZTEST(iwrl6322_decoder, test_decode_mpd_single_batch)
{
	/*
	 * Decode all 3 points in a single call (max_count >= num_pts).
	 * Verify zero-copy: readings points into the buffer, not a copy.
	 */
	uint8_t buf[512];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	struct iwrl6322_point_cloud_data out;
	uint32_t fit = 0;
	int decoded;

	build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 7, 0x05, 3);

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	decoded = dec->decode(buf, cs, &fit, 10, &out);
	zassert_equal(decoded, 3, "expected 3 decoded points, got %d", decoded);
	zassert_equal(fit, 3, "fit must advance to 3");
	zassert_equal(out.frame_count, 7);
	zassert_equal(out.zone_occupancy, 0x05);
	zassert_not_null(out.readings);

	/* Verify synthetic point values */
	zassert_equal(out.readings[0].x_cm, 10);
	zassert_equal(out.readings[0].y_cm, 20);
	zassert_equal(out.readings[0].z_cm, -5);
	zassert_equal(out.readings[1].x_cm, 20);
	zassert_equal(out.readings[2].x_cm, 30);

	/* Second call with fit=3 must return 0 (all consumed) */
	decoded = dec->decode(buf, cs, &fit, 10, &out);
	zassert_equal(decoded, 0, "second decode call must return 0");
}

ZTEST(iwrl6322_decoder, test_decode_mpd_batched_iteration)
{
	/*
	 * Verify the frame iterator by decoding 5 points in batches of 2.
	 * Total calls needed: ceil(5/2) = 3.
	 */
	uint8_t buf[512];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	struct iwrl6322_point_cloud_data out;
	uint32_t fit = 0;
	int total = 0;
	int decoded;

	build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 1, 0, 5);

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));

	decoded = dec->decode(buf, cs, &fit, 2, &out);
	zassert_equal(decoded, 2);
	total += decoded;

	decoded = dec->decode(buf, cs, &fit, 2, &out);
	zassert_equal(decoded, 2);
	total += decoded;

	decoded = dec->decode(buf, cs, &fit, 2, &out);
	zassert_equal(decoded, 1, "last batch must be 1 (5 mod 2)");
	total += decoded;

	zassert_equal(total, 5);

	/* Exhausted */
	decoded = dec->decode(buf, cs, &fit, 2, &out);
	zassert_equal(decoded, 0);
}

ZTEST(iwrl6322_decoder, test_decode_fe_mode_returns_zero)
{
	/* FE modes: decode() must return 0 (no structured point data) */
	uint8_t buf[512];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	struct iwrl6322_point_cloud_data out;
	uint32_t fit = 0;

	build_fe_buf(buf, IWRL6322_MODE_1D_FFT, 64);

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_equal(dec->decode(buf, cs, &fit, 10, &out), 0);
}

ZTEST(iwrl6322_decoder, test_decode_max_count_zero)
{
	/* max_count=0 must always return 0 without touching *fit */
	uint8_t buf[512];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	struct iwrl6322_point_cloud_data out;
	uint32_t fit = 0;

	build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 1, 0, 4);

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_equal(dec->decode(buf, cs, &fit, 0, &out), 0);
	zassert_equal(fit, 0, "fit must not advance when max_count=0");
}

ZTEST(iwrl6322_decoder, test_decode_bad_magic)
{
	uint8_t buf[512];
	const struct sensor_decoder_api *dec;
	struct sensor_chan_spec cs = {
		.chan_type = (uint16_t)IWRL6322_CHAN_SENSOR_INFO,
		.chan_idx  = 0,
	};
	struct iwrl6322_point_cloud_data out;
	uint32_t fit = 0;

	build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 1, 0, 2);

	struct iwrl6322_buffer_header *hdr = (struct iwrl6322_buffer_header *)buf;
	hdr->magic = 0xBAD0;

	zassert_ok(iwrl6322_get_decoder(NULL, &dec));
	zassert_equal(dec->decode(buf, cs, &fit, 10, &out), -EINVAL);
}

/* -------------------------------------------------------------------------
 * get_sensor_info tests
 * -------------------------------------------------------------------------
 */

ZTEST(iwrl6322_decoder, test_get_sensor_info_mpd)
{
	uint8_t buf[512];
	struct iwrl6322_sensor_info info;

	size_t total = build_mpd_buf(buf, IWRL6322_MODE_MPD_PRESENCE, 99, 0x07, 4);

	zassert_ok(iwrl6322_decoder_get_sensor_info(buf, total, &info));
	zassert_equal(info.frame_count, 99);
	zassert_equal(info.num_detections, 4);
	zassert_equal(info.zone_occupancy, 0x07);
	zassert_not_null(info.points);
	zassert_is_null(info.raw_data);
	zassert_equal(info.raw_data_len, 0);

	/* Spot-check the zero-copy point pointer */
	zassert_equal(info.points[0].x_cm, 10);
	zassert_equal(info.points[3].x_cm, 40);
}

ZTEST(iwrl6322_decoder, test_get_sensor_info_fe_raw_adc)
{
	uint8_t buf[512];
	struct iwrl6322_sensor_info info;

	size_t total = build_fe_buf(buf, IWRL6322_MODE_RAW_ADC, 32);

	zassert_ok(iwrl6322_decoder_get_sensor_info(buf, total, &info));
	zassert_is_null(info.points);
	zassert_not_null(info.raw_data);
	zassert_equal(info.raw_data_len, 32 * sizeof(uint16_t));
	zassert_equal(info.raw_data[0], 1);
	zassert_equal(info.raw_data[31], 32);
}

ZTEST(iwrl6322_decoder, test_get_sensor_info_fe_1d_fft)
{
	uint8_t buf[512];
	struct iwrl6322_sensor_info info;

	size_t total = build_fe_buf(buf, IWRL6322_MODE_1D_FFT, 16);

	zassert_ok(iwrl6322_decoder_get_sensor_info(buf, total, &info));
	zassert_is_null(info.points);
	zassert_not_null(info.raw_data);
	zassert_equal(info.raw_data_len, 16 * sizeof(uint16_t));
}

ZTEST(iwrl6322_decoder, test_get_sensor_info_truncated_buffer)
{
	/* A buffer shorter than the minimum required must return -EINVAL */
	uint8_t buf[4] = {0}; /* far too short */
	struct iwrl6322_sensor_info info;

	zassert_equal(iwrl6322_decoder_get_sensor_info(buf, sizeof(buf), &info),
		      -EINVAL);
}

ZTEST(iwrl6322_decoder, test_get_sensor_info_bad_magic)
{
	uint8_t buf[512];
	struct iwrl6322_sensor_info info;

	size_t total = build_mpd_buf(buf, IWRL6322_MODE_MPD_VERBOSE, 1, 0, 2);

	struct iwrl6322_buffer_header *hdr = (struct iwrl6322_buffer_header *)buf;
	hdr->magic = 0xFFFF;

	zassert_equal(iwrl6322_decoder_get_sensor_info(buf, total, &info),
		      -EINVAL);
}

ZTEST(iwrl6322_decoder, test_get_sensor_info_unsupported_mode)
{
	/*
	 * IWRL6322_MODE_STOP and IWRL6322_MODE_POWER_OFF are not valid payload
	 * modes; get_sensor_info must return -ENOTSUP.
	 */
	uint8_t buf[128];
	struct iwrl6322_sensor_info info;

	/* Build a header with STOP mode and a non-zero payload_size so the
	 * length check passes, then verify the mode rejection.
	 */
	fill_header(buf, IWRL6322_MODE_STOP, 16, 0);

	/*
	 * Provide a buf_len large enough that the size check in
	 * validate_and_skip_header passes but the mode check fails.
	 */
	zassert_equal(
		iwrl6322_decoder_get_sensor_info(buf, sizeof(buf), &info),
		-ENOTSUP);
}

/* -------------------------------------------------------------------------
 * Buffer magic / IWRL6322_BUFFER_MAGIC constant test
 * -------------------------------------------------------------------------
 */

ZTEST(iwrl6322_decoder, test_buffer_magic_value)
{
	/*
	 * IWRL6322_BUFFER_MAGIC is a public API constant used by applications
	 * to validate buffers before processing.  Verify its value hasn't
	 * inadvertently changed — it is the lower 16 bits of the part number.
	 */
	zassert_equal(IWRL6322_BUFFER_MAGIC, 0x6322U);
}
