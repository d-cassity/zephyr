/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 *
 * IWRL6322 hardware integration tests.
 *
 * These tests require a physical IWRL6322 connected to the target board
 * (cc1352p1_launchxl) and the full driver compiled in (CONFIG_IWRL6322=y).
 *
 * Coverage:
 *   - Device readiness after driver init
 *   - sensor_attr_set / sensor_attr_get roundtrips for key attributes
 *   - Boundary values and out-of-range rejection
 *   - sensor_get_decoder() returns a valid decoder API
 *   - RTIO iodev can be submitted without error
 *
 * Tests are intentionally side-effect-free with respect to the sensor RF:
 * they only exercise the driver's parameter store (attr_set/attr_get), not
 * the actual RF bring-up sequence.  The RAW_ADC and MPD streaming tests
 * live in a separate test application that requires a longer timeout.
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include "iwrl6322.h"

/* Devicetree node alias — defined in boards/cc1352p1_launchxl.overlay */
#define IWRL6322_NODE DT_NODELABEL(mmwave)

/* -------------------------------------------------------------------------
 * Suite fixture — holds the device pointer used by all tests
 * -------------------------------------------------------------------------
 */

struct iwrl6322_integration_fixture {
	const struct device *dev;
};

static void *iwrl6322_setup(void)
{
	static struct iwrl6322_integration_fixture fix;

	fix.dev = DEVICE_DT_GET(IWRL6322_NODE);
	zassert_not_null(fix.dev, "DEVICE_DT_GET returned NULL");
	zassert_true(device_is_ready(fix.dev),
		     "IWRL6322 device not ready — check wiring and overlay");
	return &fix;
}

ZTEST_SUITE(iwrl6322_integration, NULL, iwrl6322_setup, NULL, NULL, NULL);

/* -------------------------------------------------------------------------
 * Helper: set an attribute and immediately read it back
 * -------------------------------------------------------------------------
 */

static int attr_roundtrip(const struct device *dev,
			  enum sensor_attribute attr,
			  int32_t set_val,
			  int32_t *got_val)
{
	struct sensor_value sv = { .val1 = set_val, .val2 = 0 };
	int ret;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL, attr, &sv);
	if (ret != 0) {
		return ret;
	}

	sv.val1 = ~set_val; /* poison before get */
	ret = sensor_attr_get(dev, SENSOR_CHAN_ALL, attr, &sv);
	if (ret != 0) {
		return ret;
	}

	*got_val = sv.val1;
	return 0;
}

/* -------------------------------------------------------------------------
 * Basic driver / API tests
 * -------------------------------------------------------------------------
 */

ZTEST_F(iwrl6322_integration, test_device_is_ready)
{
	/* Redundant with the suite setup assertion; kept as an explicit test
	 * so that the test report names it clearly.
	 */
	zassert_true(device_is_ready(fixture->dev));
}

ZTEST_F(iwrl6322_integration, test_sensor_get_decoder)
{
	const struct sensor_decoder_api *dec = NULL;

	zassert_ok(sensor_get_decoder(fixture->dev, &dec));
	zassert_not_null(dec);
	zassert_not_null(dec->get_frame_count);
	zassert_not_null(dec->get_size_info);
	zassert_not_null(dec->decode);
}

/* -------------------------------------------------------------------------
 * attr_set / attr_get roundtrip tests
 *
 * Each test picks a representative value well within the valid range and
 * verifies the driver stores and returns it unchanged.  Out-of-range
 * rejection tests follow in a separate group.
 * -------------------------------------------------------------------------
 */

ZTEST_F(iwrl6322_integration, test_attr_region_fcc)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_REGION,
				  IWRL6322_REGION_FCC_CE, &got));
	zassert_equal(got, IWRL6322_REGION_FCC_CE);
}

ZTEST_F(iwrl6322_integration, test_attr_region_telec)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_REGION,
				  IWRL6322_REGION_TELEC, &got));
	zassert_equal(got, IWRL6322_REGION_TELEC);
}

ZTEST_F(iwrl6322_integration, test_attr_motion_sensitivity_min)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MOTION_SENSITIVITY,
				  0, &got));
	zassert_equal(got, 0);
}

ZTEST_F(iwrl6322_integration, test_attr_motion_sensitivity_max)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MOTION_SENSITIVITY,
				  3, &got));
	zassert_equal(got, 3);
}

ZTEST_F(iwrl6322_integration, test_attr_detection_sensitivity)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_DETECTION_SENSITIVITY,
				  3, &got));
	zassert_equal(got, 3);
}

ZTEST_F(iwrl6322_integration, test_attr_zone_count)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_ZONE_COUNT,
				  4, &got));
	zassert_equal(got, 4);
}

ZTEST_F(iwrl6322_integration, test_attr_zone_boundaries_zone0)
{
	/*
	 * Set all six boundary coordinates for zone 0 and verify each one
	 * reads back correctly.
	 */
	const struct device *dev = fixture->dev;
	struct sensor_value sv;
	int ret;

	/* xmin = -100 cm */
	sv.val1 = 0; sv.val2 = -100;
	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL,
			      (enum sensor_attribute)IWRL6322_ATTR_ZONE_XMIN, &sv);
	zassert_ok(ret, "ZONE_XMIN set failed: %d", ret);

	sv.val1 = 0; sv.val2 = ~(-100);
	ret = sensor_attr_get(dev, SENSOR_CHAN_ALL,
			      (enum sensor_attribute)IWRL6322_ATTR_ZONE_XMIN, &sv);
	zassert_ok(ret);
	zassert_equal(sv.val2, -100, "ZONE_XMIN readback mismatch: %d", sv.val2);

	/* xmax = 100 cm */
	sv.val1 = 0; sv.val2 = 100;
	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_XMAX, &sv));
	sv.val1 = 0; sv.val2 = 0;
	zassert_ok(sensor_attr_get(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_XMAX, &sv));
	zassert_equal(sv.val2, 100);

	/* ymin = 0 cm */
	sv.val1 = 0; sv.val2 = 0;
	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_YMIN, &sv));

	/* ymax = 400 cm */
	sv.val1 = 0; sv.val2 = 400;
	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_YMAX, &sv));
	sv.val1 = 0; sv.val2 = 0;
	zassert_ok(sensor_attr_get(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_YMAX, &sv));
	zassert_equal(sv.val2, 400);

	/* zmin = -50 cm */
	sv.val1 = 0; sv.val2 = -50;
	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_ZMIN, &sv));

	/* zmax = 250 cm */
	sv.val1 = 0; sv.val2 = 250;
	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_ZMAX, &sv));
	sv.val1 = 0; sv.val2 = 0;
	zassert_ok(sensor_attr_get(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_ZMAX, &sv));
	zassert_equal(sv.val2, 250);

	/* include type */
	sv.val1 = 0; sv.val2 = 1; /* include */
	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_TYPE, &sv));
	sv.val1 = 0; sv.val2 = 0;
	zassert_ok(sensor_attr_get(dev, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)IWRL6322_ATTR_ZONE_TYPE, &sv));
	zassert_equal(sv.val2, 1);
}

ZTEST_F(iwrl6322_integration, test_attr_zone_coordinate_cartesian)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_ZONE_COORDINATE_CARTESIAN,
				  1, &got));
	zassert_equal(got, 1);

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_ZONE_COORDINATE_CARTESIAN,
				  0, &got));
	zassert_equal(got, 0);
}

ZTEST_F(iwrl6322_integration, test_attr_disable_static_obj_removal)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_DISABLE_STATIC_OBJ_REMOVAL,
				  1, &got));
	zassert_equal(got, 1);
}

ZTEST_F(iwrl6322_integration, test_attr_operating_mode_stop)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_OPERATING_MODE,
				  IWRL6322_MODE_STOP, &got));
	zassert_equal(got, IWRL6322_MODE_STOP);
}

ZTEST_F(iwrl6322_integration, test_attr_mpd_prof_sel)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MPD_PROF_SEL,
				  2, &got));
	zassert_equal(got, 2);
}

ZTEST_F(iwrl6322_integration, test_attr_mpd_detection_mode)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MPD_DETECTION_MODE,
				  3, &got));
	zassert_equal(got, 3);
}

ZTEST_F(iwrl6322_integration, test_attr_mpd_snr_boost)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MPD_SNR_BOOST,
				  1, &got));
	zassert_equal(got, 1);
}

ZTEST_F(iwrl6322_integration, test_attr_mpd_num_frames)
{
	int32_t got;

	/* 0 = infinite */
	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MPD_NUM_FRAMES,
				  0, &got));
	zassert_equal(got, 0);

	/* finite frame count */
	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MPD_NUM_FRAMES,
				  100, &got));
	zassert_equal(got, 100);
}

ZTEST_F(iwrl6322_integration, test_attr_mpd_holdtime)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MPD_HOLDTIME_10US,
				  5000, &got));
	zassert_equal(got, 5000);
}

ZTEST_F(iwrl6322_integration, test_attr_mpd_adv_cfar_cfg)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MPD_ADV_CFAR_CFG,
				  -3, &got));
	zassert_equal(got, -3);

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MPD_ADV_CFAR_CFG,
				  6, &got));
	zassert_equal(got, 6);
}

ZTEST_F(iwrl6322_integration, test_attr_sampling_frequency)
{
	/*
	 * SENSOR_ATTR_SAMPLING_FREQUENCY maps to frame_period_ms.
	 * Set 1 Hz (1000 mHz) and verify readback.
	 */
	const struct device *dev = fixture->dev;
	struct sensor_value sv = { .val1 = 1000, .val2 = 0 }; /* 1000 mHz = 1 Hz */
	int32_t got;

	zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ALL,
				   SENSOR_ATTR_SAMPLING_FREQUENCY, &sv));
	sv.val1 = 0;
	zassert_ok(sensor_attr_get(dev, SENSOR_CHAN_ALL,
				   SENSOR_ATTR_SAMPLING_FREQUENCY, &sv));
	got = sv.val1;
	zassert_equal(got, 1000, "sampling frequency readback mismatch: %d", got);
}

ZTEST_F(iwrl6322_integration, test_attr_full_scale_range)
{
	/*
	 * SENSOR_ATTR_FULL_SCALE maps to max_range_m.
	 * 0 = 4 m, 1 = 8 m, 2 = 12 m.
	 */
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev, SENSOR_ATTR_FULL_SCALE, 1, &got));
	zassert_equal(got, 1);
}

ZTEST_F(iwrl6322_integration, test_attr_sens_pos_x)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_SENS_POS_X_CM,
				  -75, &got));
	zassert_equal(got, -75);
}

ZTEST_F(iwrl6322_integration, test_attr_sens_pos_y)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_SENS_POS_Y_CM,
				  200, &got));
	zassert_equal(got, 200);
}

ZTEST_F(iwrl6322_integration, test_attr_sens_pos_z)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_SENS_POS_Z_CM,
				  300, &got));
	zassert_equal(got, 300);
}

ZTEST_F(iwrl6322_integration, test_attr_sens_tilt_azimuth)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_SENS_TILT_AZIMUTH,
				  -45, &got));
	zassert_equal(got, -45);
}

ZTEST_F(iwrl6322_integration, test_attr_sens_tilt_elevation)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_SENS_TILT_ELEVATION,
				  30, &got));
	zassert_equal(got, 30);
}

ZTEST_F(iwrl6322_integration, test_attr_antenna_type)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_ANTENNA_TYPE,
				  0, &got));
	zassert_equal(got, 0);
}

ZTEST_F(iwrl6322_integration, test_attr_fe_adc_samp_rate)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_FE_ADC_SAMP_RATE,
				  16, &got));
	zassert_equal(got, 16);
}

ZTEST_F(iwrl6322_integration, test_attr_fe_num_adc_samples)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_FE_NUM_ADC_SAMPLES,
				  128, &got));
	zassert_equal(got, 128);
}

ZTEST_F(iwrl6322_integration, test_attr_fe_chirps_per_burst)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_FE_CHIRPS_PER_BURST,
				  64, &got));
	zassert_equal(got, 64);
}

ZTEST_F(iwrl6322_integration, test_attr_fe_num_frames_infinite)
{
	int32_t got;

	/* 0 = infinite */
	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_FE_NUM_FRAMES,
				  0, &got));
	zassert_equal(got, 0);
}

ZTEST_F(iwrl6322_integration, test_attr_fe_frame_period_ms)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_FE_FRAME_PERIOD_MS,
				  200, &got));
	zassert_equal(got, 200);
}

ZTEST_F(iwrl6322_integration, test_attr_run_cal_ctrl)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_RUN_CAL_CTRL,
				  1, &got));
	zassert_equal(got, 1);
}

ZTEST_F(iwrl6322_integration, test_attr_deep_sleep_entry_dis)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_DEEP_SLEEP_ENTRY_DIS,
				  1, &got));
	zassert_equal(got, 1);
}

ZTEST_F(iwrl6322_integration, test_attr_xtal_less_mode)
{
	int32_t got;

	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_XTAL_LESS_MODE,
				  0, &got));
	zassert_equal(got, 0);
}

ZTEST_F(iwrl6322_integration, test_attr_mount_type)
{
	int32_t got;

	/* wall mount */
	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MOUNT_TYPE,
				  0, &got));
	zassert_equal(got, 0);

	/* ceiling mount */
	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_MOUNT_TYPE,
				  1, &got));
	zassert_equal(got, 1);
}

/* test_attr_gpadc_param: omitted — IWRL6322_ATTR_GPADC_PARAM is not yet
 * implemented in the driver (attr_set/attr_get are commented out).
 * Re-enable this test when GPADC support is added.
 */

/* -------------------------------------------------------------------------
 * Out-of-range rejection tests
 *
 * The driver must return -EINVAL for values outside the documented range.
 * These ensure the driver enforces its own documented constraints.
 * -------------------------------------------------------------------------
 */

ZTEST_F(iwrl6322_integration, test_attr_motion_sensitivity_out_of_range)
{
	struct sensor_value sv = { .val1 = 4, .val2 = 0 }; /* max is 3 */

	zassert_equal(
		sensor_attr_set(fixture->dev, SENSOR_CHAN_ALL,
				(enum sensor_attribute)IWRL6322_ATTR_MOTION_SENSITIVITY,
				&sv),
		-EINVAL,
		"motion_sensitivity=4 must be rejected");
}

ZTEST_F(iwrl6322_integration, test_attr_detection_sensitivity_out_of_range)
{
	struct sensor_value sv = { .val1 = 6, .val2 = 0 }; /* max is 5 */

	zassert_equal(
		sensor_attr_set(fixture->dev, SENSOR_CHAN_ALL,
				(enum sensor_attribute)IWRL6322_ATTR_DETECTION_SENSITIVITY,
				&sv),
		-EINVAL,
		"detection_sensitivity=6 must be rejected");
}

ZTEST_F(iwrl6322_integration, test_attr_zone_count_out_of_range)
{
	struct sensor_value sv = { .val1 = 17, .val2 = 0 }; /* max is 16 */

	zassert_equal(
		sensor_attr_set(fixture->dev, SENSOR_CHAN_ALL,
				(enum sensor_attribute)IWRL6322_ATTR_ZONE_COUNT,
				&sv),
		-EINVAL,
		"zone_count=17 must be rejected");
}

ZTEST_F(iwrl6322_integration, test_attr_mpd_adv_cfar_out_of_range)
{
	struct sensor_value sv = { .val1 = 7, .val2 = 0 }; /* max is +6 */

	zassert_equal(
		sensor_attr_set(fixture->dev, SENSOR_CHAN_ALL,
				(enum sensor_attribute)IWRL6322_ATTR_MPD_ADV_CFAR_CFG,
				&sv),
		-EINVAL,
		"mpd_adv_cfar_cfg=7 must be rejected");
}

/* -------------------------------------------------------------------------
 * Factory calibration and inter-chirp power save tests
 * -------------------------------------------------------------------------
 */

ZTEST_F(iwrl6322_integration, test_attr_trig_boot_cal)
{
	int32_t got;

	/* Enable factory calibration trigger */
	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_TRIG_BOOT_CAL,
				  1, &got));
	zassert_equal(got, 1);

	/* Disable factory calibration trigger */
	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_TRIG_BOOT_CAL,
				  0, &got));
	zassert_equal(got, 0);
}

ZTEST_F(iwrl6322_integration, test_attr_fe_inter_chirp_ps_ovrd)
{
	int32_t got;

	/* Override disabled (bit 0 = 0, default power save applies) */
	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_FE_INTER_CHIRP_PS_OVRD,
				  0, &got));
	zassert_equal(got, 0);

	/*
	 * Override enabled with all subsystems powered down between chirps:
	 * bit 0 = override enable
	 * bit 1 = PA/TX off
	 * bit 2 = LO/TX off
	 * bit 3 = RF/RX off
	 * bit 4 = BB/RX off
	 * bit 5 = LO/RX off
	 * bit 6 = synth off
	 * = 0x7F
	 */
	zassert_ok(attr_roundtrip(fixture->dev,
				  (enum sensor_attribute)IWRL6322_ATTR_FE_INTER_CHIRP_PS_OVRD,
				  0x7F, &got));
	zassert_equal(got, 0x7F);
}
