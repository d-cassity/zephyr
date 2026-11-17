/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 *
 * IWRL6322 decoder implementation for the TI IWRL6322 radar sensor driver.
 *
 * The RTIO buffer produced by iwrl6322_submit_sync() contains:
 *   [struct iwrl6322_buffer_header] [raw device memory payload]
 *
 * The header makes the buffer self-describing with operating mode and
 * timestamp. The operating_mode field selects the decode path:
 *
 *   IWRL6322_MODE_MPD_PRESENCE (3) or IWRL6322_MODE_MPD_VERBOSE (4):
 *     Payload = struct iwrl6322_pc_payload_hdr + T_ML_SENS_POINT_CLOUD_DATA[]
 *     decode() fills an iwrl6322_point_cloud_data (zero-copy pointer into buffer).
 *
 *   IWRL6322_MODE_RAW_ADC (1) or IWRL6322_MODE_1D_FFT (2):
 *     Payload = raw 16-bit ADC or 1D-FFT samples.
 *     get_frame_count() returns 0 (no structured decode path).
 *     Applications must read the payload directly past the iwrl6322_buffer_header.
 *
 * Both MPD decode paths are zero-copy: the readings pointer in
 * iwrl6322_point_cloud_data points directly into the RTIO buffer payload.
 * The pointed-to memory is valid as long as the application holds the RTIO buffer.
 *
 * decode() frame iterator (*fit):
 *   0 on first call; advanced by the batch size returned; returns 0 when all
 *   points have been consumed.
 */

#define DT_DRV_COMPAT ti_iwrl6322

#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>
#include "iwrl6322.h"
#include <mmwaveulink/include/ml_smartfe.h>

/*
 * Guard that struct iwrl6322_point_data is layout-compatible with
 * T_ML_SENS_POINT_CLOUD_DATA so a direct pointer cast is safe.
 */
BUILD_ASSERT(sizeof(struct iwrl6322_point_data) == sizeof(T_ML_SENS_POINT_CLOUD_DATA),
	     "iwrl6322_point_data / T_ML_SENS_POINT_CLOUD_DATA size mismatch");

/*
 * MPD point cloud payload header, as transferred by ml_devSensorGetData()
 * (see the point cloud table in ml_dev.h, M_ML_POINTCLOUD_HDR_SIZE).
 *
 * Layout: [iwrl6322_pc_payload_hdr (12 bytes)]
 *         [T_ML_SENS_POINT_CLOUD_DATA x (major + minor points)]
 *         [clusters, not decoded]
 *
 * The major and minor point entries are assembled back to back, so the
 * number of points that follow the header is major + minor.
 */
struct iwrl6322_pc_payload_hdr {
	uint32_t w_FrameCount;        /* Rolling frame counter */
	uint16_t h_ZoneState;         /* Zone occupancy, bit i = zone i occupied */
	uint8_t  c_NumPointsMajor;    /* Points in the major point cloud */
	uint8_t  c_NumPointsMinor;    /* Points in the minor point cloud */
	uint8_t  c_NumClustersMajor;  /* Clusters in the major point cloud */
	uint8_t  c_NumClustersMinor;  /* Clusters in the minor point cloud */
	uint8_t  c_Reserved[2];
};

BUILD_ASSERT(sizeof(struct iwrl6322_pc_payload_hdr) == 12U,
	     "iwrl6322_pc_payload_hdr must match M_ML_POINTCLOUD_HDR_SIZE");

/* Maximum number of MPD detection points returned by IWRL6322 firmware */
#define IWRL6322_MAX_DET_POINTS 64U

/* -------------------------------------------------------------------------
 * Buffer Header Validation
 * -------------------------------------------------------------------------
 */

/*
 * Validate buffer header and return pointer to the payload.
 *
 * @param buffer     RTIO buffer with iwrl6322_buffer_header + payload
 * @param buf_len    Total buffer length (or SIZE_MAX to skip length check)
 * @param hdr_out    Output: pointer to validated header (optional, can be NULL)
 * @return Pointer to payload (after header), or NULL if validation failed
 */
static const uint8_t *validate_and_skip_header(const uint8_t *buffer,
						size_t buf_len,
						const struct iwrl6322_buffer_header **hdr_out)
{
	if (buf_len != SIZE_MAX && buf_len < sizeof(struct iwrl6322_buffer_header)) {
		return NULL;
	}

	const struct iwrl6322_buffer_header *hdr =
		(const struct iwrl6322_buffer_header *)buffer;

	if (hdr->magic != IWRL6322_BUFFER_MAGIC) {
		return NULL;
	}

	if (buf_len != SIZE_MAX &&
	    buf_len < sizeof(struct iwrl6322_buffer_header) + hdr->payload_size) {
		return NULL;
	}

	if (hdr_out != NULL) {
		*hdr_out = hdr;
	}

	return buffer + sizeof(struct iwrl6322_buffer_header);
}

/*
 * Number of point entries that can be read from the payload: major + minor
 * from the device header, limited to what the payload actually holds and to
 * IWRL6322_MAX_DET_POINTS. The caller has checked payload_size >= header size.
 */
static uint8_t pc_num_points(const struct iwrl6322_pc_payload_hdr *pc_hdr,
			     uint16_t payload_size)
{
	uint32_t reported = (uint32_t)pc_hdr->c_NumPointsMajor + pc_hdr->c_NumPointsMinor;
	uint32_t room = (payload_size - sizeof(struct iwrl6322_pc_payload_hdr)) /
			sizeof(T_ML_SENS_POINT_CLOUD_DATA);

	return (uint8_t)MIN(MIN(reported, room), (uint32_t)IWRL6322_MAX_DET_POINTS);
}

/* Returns true if the operating_mode in the header is an MPD mode */
static inline bool is_mpd_mode(uint8_t operating_mode)
{
	return operating_mode == (uint8_t)IWRL6322_MODE_MPD_PRESENCE ||
	       operating_mode == (uint8_t)IWRL6322_MODE_MPD_VERBOSE;
}

/* -------------------------------------------------------------------------
 * Standard sensor_decoder_api callbacks
 * -------------------------------------------------------------------------
 */

static int iwrl6322_decoder_get_frame_count(const uint8_t *buffer,
					   struct sensor_chan_spec chan_spec,
					   uint16_t *frame_count)
{
	if (chan_spec.chan_type != (uint16_t)IWRL6322_CHAN_SENSOR_INFO ||
	    chan_spec.chan_idx  != 0) {
		return -EINVAL;
	}

	const struct iwrl6322_buffer_header *hdr;
	const uint8_t *payload = validate_and_skip_header(buffer, SIZE_MAX, &hdr);

	if (payload == NULL) {
		return -EINVAL;
	}

	/* FE modes (RAW_ADC, 1D_FFT) have no structured decode path */
	if (!is_mpd_mode(hdr->operating_mode)) {
		*frame_count = 0;
		return 0;
	}

	if (hdr->payload_size < sizeof(struct iwrl6322_pc_payload_hdr)) {
		return -EINVAL;
	}

	const struct iwrl6322_pc_payload_hdr *pc_hdr =
		(const struct iwrl6322_pc_payload_hdr *)payload;

	*frame_count = pc_num_points(pc_hdr, hdr->payload_size);
	return 0;
}

static int iwrl6322_decoder_get_size_info(struct sensor_chan_spec chan_spec,
					 size_t *base_size,
					 size_t *frame_size)
{
	if (chan_spec.chan_type != (uint16_t)IWRL6322_CHAN_SENSOR_INFO) {
		return -ENOTSUP;
	}

	/*
	 * readings is a pointer into the RTIO buffer — no inline storage.
	 * frame_size is 0: the output struct has fixed size regardless of
	 * how many points are decoded per call.
	 */
	*base_size  = sizeof(struct iwrl6322_point_cloud_data);
	*frame_size = 0;
	return 0;
}

static int iwrl6322_decoder_decode(const uint8_t *buffer,
				  struct sensor_chan_spec chan_spec,
				  uint32_t *fit,
				  uint16_t max_count,
				  void *data_out)
{
	if (chan_spec.chan_type != (uint16_t)IWRL6322_CHAN_SENSOR_INFO ||
	    chan_spec.chan_idx  != 0) {
		return -EINVAL;
	}

	if (max_count == 0) {
		return 0;
	}

	const struct iwrl6322_buffer_header *hdr;
	const uint8_t *payload = validate_and_skip_header(buffer, SIZE_MAX, &hdr);

	if (payload == NULL) {
		return -EINVAL;
	}

	/* FE modes: no structured decode; caller must read payload directly */
	if (!is_mpd_mode(hdr->operating_mode)) {
		return 0;
	}

	if (hdr->payload_size < sizeof(struct iwrl6322_pc_payload_hdr)) {
		return -EINVAL;
	}

	const struct iwrl6322_pc_payload_hdr *pc_hdr =
		(const struct iwrl6322_pc_payload_hdr *)payload;
	struct iwrl6322_point_cloud_data *out = data_out;

	uint8_t total = pc_num_points(pc_hdr, hdr->payload_size);
	uint8_t start = (uint8_t)*fit;

	if (start >= total) {
		return 0;
	}

	uint8_t count = (uint8_t)MIN(max_count, (uint16_t)(total - start));

	out->header.base_timestamp_ns = (uint64_t)hdr->timestamp_us * 1000ULL;
	out->header.reading_count     = count;
	out->frame_count              = pc_hdr->w_FrameCount;
	out->zone_occupancy           = pc_hdr->h_ZoneState;

	/* Zero-copy: point directly into the RTIO buffer payload after the header */
	const T_ML_SENS_POINT_CLOUD_DATA *pts =
		(const T_ML_SENS_POINT_CLOUD_DATA *)(payload +
			sizeof(struct iwrl6322_pc_payload_hdr));

	out->readings = (const struct iwrl6322_point_data *)&pts[start];

	*fit = start + count;
	return count;
}

int iwrl6322_decoder_get_sensor_info(const uint8_t *buf, size_t buf_len,
					   struct iwrl6322_sensor_info *out)
{
	const size_t min_len = sizeof(struct iwrl6322_buffer_header) +
			       sizeof(struct iwrl6322_pc_payload_hdr);

	if (buf_len < min_len) {
		return -EINVAL;
	}

	const struct iwrl6322_buffer_header *hdr;
	const uint8_t *payload = validate_and_skip_header(buf, buf_len, &hdr);

	if (payload == NULL) {
		return -EINVAL;
	}

	/* Clear all output fields; only the relevant subset is populated below. */
	out->frame_count    = 0;
	out->num_detections = 0;
	out->zone_occupancy = 0;
	out->points         = NULL;
	out->raw_data       = NULL;
	out->raw_data_len   = 0;

	/* FE modes: expose raw payload directly; no point cloud to decode. */
	if (hdr->operating_mode == (uint8_t)IWRL6322_MODE_RAW_ADC ||
	    hdr->operating_mode == (uint8_t)IWRL6322_MODE_1D_FFT) {
		out->raw_data     = (const uint16_t *)payload;
		out->raw_data_len = hdr->payload_size;
		return 0;
	}

	/* MPD modes: parse the point cloud payload header. */
	if (hdr->operating_mode != (uint8_t)IWRL6322_MODE_MPD_PRESENCE &&
	    hdr->operating_mode != (uint8_t)IWRL6322_MODE_MPD_VERBOSE) {
		return -ENOTSUP;
	}

	if (hdr->payload_size < sizeof(struct iwrl6322_pc_payload_hdr)) {
		return -EINVAL;
	}

	const struct iwrl6322_pc_payload_hdr *pc_hdr =
		(const struct iwrl6322_pc_payload_hdr *)payload;

	out->frame_count    = pc_hdr->w_FrameCount;
	out->num_detections = pc_num_points(pc_hdr, hdr->payload_size);
	out->zone_occupancy = pc_hdr->h_ZoneState;

	/* Zero-copy: point directly into the RTIO buffer payload */
	const T_ML_SENS_POINT_CLOUD_DATA *pts =
		(const T_ML_SENS_POINT_CLOUD_DATA *)(payload +
			sizeof(struct iwrl6322_pc_payload_hdr));

	out->points = (const struct iwrl6322_point_data *)pts;
	return 0;
}

/* -------------------------------------------------------------------------
 * Decoder registration
 * -------------------------------------------------------------------------
 */

static const struct sensor_decoder_api iwrl6322_decoder = {
	.get_frame_count = iwrl6322_decoder_get_frame_count,
	.get_size_info   = iwrl6322_decoder_get_size_info,
	.decode          = iwrl6322_decoder_decode,
};

int iwrl6322_get_decoder(const struct device *dev,
		       const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &iwrl6322_decoder;
	return 0;
}
