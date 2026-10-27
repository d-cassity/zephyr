/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal header for the TI IWRL6322 radar sensor driver.
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TI_IWRL6322_H_
#define ZEPHYR_DRIVERS_SENSOR_TI_IWRL6322_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Custom sensor channels for the IWRL6322 driver.
 *
 * channel_get() for IWRL6322_CHAN_SENSOR_INFO returns:
 *   val1 — number of detected targets in the last frame (0–64)
 *   val2 — zone-occupancy bitmap (bit i = zone i occupied)
 *
 * Full point cloud is accessible via sensor_get_decoder() / sensor_read().
 * Data is only accessible via the RTIO streaming API for this device.
 */
enum iwrl6322_sensor_channel {
	IWRL6322_CHAN_SENSOR_INFO = SENSOR_CHAN_PRIV_START,
	IWRL6322_CHAN_PRESENCE,
	/* Raw GPADC measurement result (w_MeasResVal, 1 LSB = 1.8V/256) */
	IWRL6322_CHAN_GPADC,
};

/**
 * @brief Operating mode values for IWRL6322_ATTR_OPERATING_MODE.
 *
 * @note Switching between the Front-End (FE) mode family (RAW_ADC, 1D_FFT)
 * and the MPD mode family (MPD_PRESENCE, MPD_VERBOSE) requires a full device
 * power cycle.  Before switching families, set the operating mode to
 * IWRL6322_MODE_POWER_OFF first; the driver will reload the device on the
 * next mode set.  Switching between modes within the same family (e.g.
 * RAW_ADC -> 1D_FFT, or MPD_PRESENCE -> MPD_VERBOSE) does not require a
 * power cycle.
 */
enum iwrl6322_operating_mode {
	/** Stop sensor — no RF emissions, no data */
	IWRL6322_MODE_STOP         = 0,
	/** Front-end Raw ADC mode */
	IWRL6322_MODE_RAW_ADC      = 1,
	/** Front-end 1D-FFT mode */
	IWRL6322_MODE_1D_FFT       = 2,
	/** MPD quiet mode — data only when presence detected */
	IWRL6322_MODE_MPD_PRESENCE = 3,
	/** MPD verbose mode — data every frame */
	IWRL6322_MODE_MPD_VERBOSE  = 4,
	/** Power off — assert nRESET, firmware must be reloaded on next start */
	IWRL6322_MODE_POWER_OFF    = 5,
};

/**
 * @brief Regulatory region values.
 */
enum iwrl6322_region {
	IWRL6322_REGION_FCC_CE = 0,
	IWRL6322_REGION_TELEC  = 1,
};

/**
 * @brief Sensor attribute values for the IWRL6322 driver.
 *
 * Use with sensor_attr_set() / sensor_attr_get() and SENSOR_CHAN_ALL.
 *
 * Standard Zephyr attributes also accepted:
 *   SENSOR_ATTR_FULL_SCALE         — MPD max range index (0=4m, 1=8m, 2=12m)
 *   SENSOR_ATTR_SAMPLING_FREQUENCY — frame rate in millihertz
 */

enum iwrl6322_sensor_attr {
	/** Operating mode — one of enum iwrl6322_operating_mode */
	IWRL6322_ATTR_OPERATING_MODE = SENSOR_ATTR_PRIV_START,

	/** Regulatory region — one of enum iwrl6322_region */
	IWRL6322_ATTR_REGION,

	/** Motion detection sensitivity (0=lowest, 3=highest). val1. */
	IWRL6322_ATTR_MOTION_SENSITIVITY,

	/** Point-cloud detection sensitivity (0=lowest, 5=highest). val1. */
	IWRL6322_ATTR_DETECTION_SENSITIVITY,

	/** Number of active detection zones (1–16). val1=count. */
	IWRL6322_ATTR_ZONE_COUNT,

	/** Zone X lower bound (cm, signed); val1=zone_idx, val2=xmin */
	IWRL6322_ATTR_ZONE_XMIN,
	/** Zone X upper bound (cm, signed); val1=zone_idx, val2=xmax */
	IWRL6322_ATTR_ZONE_XMAX,
	/** Zone Y lower bound (cm, 0..1500); val1=zone_idx, val2=ymin */
	IWRL6322_ATTR_ZONE_YMIN,
	/** Zone Y upper bound (cm, 0..1500); val1=zone_idx, val2=ymax */
	IWRL6322_ATTR_ZONE_YMAX,
	/** Zone Z lower bound (cm, 0..1500); val1=zone_idx, val2=zmin */
	IWRL6322_ATTR_ZONE_ZMIN,
	/** Zone Z upper bound (cm, 0..1500); val1=zone_idx, val2=zmax */
	IWRL6322_ATTR_ZONE_ZMAX,

	/** Zone inclusion/exclusion; val1=zone_idx, val2=1(INCLUDE)/0(EXCLUDE) */
	IWRL6322_ATTR_ZONE_TYPE,

	/** Zone coordinate system; val1=1(Cartesian)/0(Polar) */
	IWRL6322_ATTR_ZONE_COORDINATE_CARTESIAN,

	/** Bypass static-object removal; val1=1(disable algo)/0(enable, default) */
	IWRL6322_ATTR_DISABLE_STATIC_OBJ_REMOVAL,

	/* ---- IWRL6322-specific attributes below ---- */

	/** MPD measurement profile index. val1=0..3 or 0xAA (auto) */
	IWRL6322_ATTR_MPD_PROF_SEL,

	/** TI library API timeout in ms (1..65535). Init-time only. */
	IWRL6322_ATTR_API_TIMEOUT_MS,

	/** Antenna type (b3_DevAntennaSel). val1=0..3 */
	IWRL6322_ATTR_ANTENNA_TYPE,

	/** MPD signal-to-deep-sleep hold time in 10 µs units. val1=1..1000000 */
	IWRL6322_ATTR_MPD_HOLDTIME_10US,

	/** FE calibration control bitmask (h_FeCalibCtrl). val1=0x000..0x1FF */
	IWRL6322_ATTR_FE_CALIB_CTRL,

	/** MPD detection mode. val1=1(1VA-1D)..4(4VA-3D) */
	IWRL6322_ATTR_MPD_DETECTION_MODE,

	/** MPD SNR boost. val1=0(disabled), 1(level1), 2(level2) */
	IWRL6322_ATTR_MPD_SNR_BOOST,

	/** MPD frames per start. val1=0(infinite)..65535 */
	IWRL6322_ATTR_MPD_NUM_FRAMES,

	/** FE ADC sampling rate divider. val1=8..100 */
	IWRL6322_ATTR_FE_ADC_SAMP_RATE,

	/** FE number of virtual antennas. val1=1..4 */
	IWRL6322_ATTR_FE_NUM_VIRTUAL_ANT,

	/** FE TX/RX channel enable bitmask. val1=0x0000..0xFFFF */
	IWRL6322_ATTR_FE_TXRX_CH_ENA,

	/** FE ADC samples per chirp. val1=32/64/128/256 */
	IWRL6322_ATTR_FE_NUM_ADC_SAMPLES,

	/** FE ramp end time in 100ns units. val1=60..2560 */
	IWRL6322_ATTR_FE_RAMP_END_TIME,

	/** FE idle time in 100ns units. val1=31..10000 */
	IWRL6322_ATTR_FE_IDLE_TIME,

	/** FE ADC start skip samples. val1=0..63 */
	IWRL6322_ATTR_FE_ADC_SKIP_SAMP,

	/** FE RF start frequency. val1=hex value */
	IWRL6322_ATTR_FE_RF_FREQ_START,

	/** FE RF frequency slope (signed). val1=-13981..13981 */
	IWRL6322_ATTR_FE_RF_FREQ_SLOPE,

	/** FE TX start time in 20ns units (signed). val1=-300..300 */
	IWRL6322_ATTR_FE_TX_START_TIME,

	/** FE HPF corner frequency. val1=0..3 */
	IWRL6322_ATTR_FE_HPF_CORNER,

	/** FE calibration RX gain (34..48 dB, even). val1 */
	IWRL6322_ATTR_FE_CAL_RX_GAIN,

	/** FE calibration TX backoff (0..15 dB). val1 */
	IWRL6322_ATTR_FE_CAL_TX_BACKOFF,

	/* FE interchirp power saving settings (0 - 127)*/
	IWRL6322_ATTR_FE_INTER_CHIRP_PS_OVRD,

	/** FE chirps to accumulate. val1=0 or 2..64 */
	IWRL6322_ATTR_FE_CHIRPS_ACCUM,

	/** FE chirps per burst. val1=1..1023 */
	IWRL6322_ATTR_FE_CHIRPS_PER_BURST,

	/** FE burst idle time in 100ns units. val1=1500..150000 */
	IWRL6322_ATTR_FE_BURST_IDLE_TIME,

	/** FE bursts per frame. val1=1..127 */
	IWRL6322_ATTR_FE_BURSTS_PER_FRAME,

	/** FE number of frames. val1=0(infinite)..65535 */
	IWRL6322_ATTR_FE_NUM_FRAMES,

	/** FE frame period in ms. val1=10..21600000 */
	IWRL6322_ATTR_FE_FRAME_PERIOD_MS,

	/** MPD signal trigger type. val1=0(pulse)/1(level) */
	IWRL6322_ATTR_MPD_SIG_TRIG_TYPE,

	/** MPD auto boot mode. val1=0(disabled)/1(enabled) */
	IWRL6322_ATTR_MPD_AUTO_BOOT_MODE,

	/** MCU clock output select. val1=0..3 */
	IWRL6322_ATTR_MCU_CLK_OUT_SEL,

	/** Disable deep sleep entry. val1=0(enabled)/1(disabled) */
	IWRL6322_ATTR_DEEP_SLEEP_ENTRY_DIS,

	/** Trigger factory calibration on next app config. val1=0/1 */
	IWRL6322_ATTR_TRIG_BOOT_CAL,

	/** Hardware-In-Loop enable (TI internal). val1=0/1 */
	IWRL6322_ATTR_HIL_ENABLE,

	/** JTAG debug interface enable (TI internal). val1=0/1 */
	IWRL6322_ATTR_JTAG_ENABLE,

	/** XTAL-less clock mode. val1=0/1 */
	IWRL6322_ATTR_XTAL_LESS_MODE,

	/** GPIO power-measurement marker (TI internal). val1=0/1 */
	IWRL6322_ATTR_GPIO_MARKER_ENA,

	/** CW/CZ mode enable (TI internal). val1=0/1 */
	IWRL6322_ATTR_CZ_CW_MODE_ENA,

	/** Hold in LPDS, disable RTC wakeup (TI internal). val1=0/1 */
	IWRL6322_ATTR_HOLD_LPDS,

	/* Mount type. val1=0(wall mount)/1(ceiling mount)*/
	IWRL6322_ATTR_MOUNT_TYPE,

	/** Run calibration control (c_RunCalCtrl). val1=0(disable)/1(enable) */
	IWRL6322_ATTR_RUN_CAL_CTRL,

	/** Periodic runtime calibration interval in frames. val1=0(disabled)..255 */
	IWRL6322_ATTR_RUN_CAL_TRIG_INTERVAL,

	/** Flash sensor config to device flash. val1=0/1 */
	IWRL6322_ATTR_FLASH_SENS_CFG,

	/** Advanced CFAR threshold offset. val1=-6..+6 */
	IWRL6322_ATTR_MPD_ADV_CFAR_CFG,

	/** Sensor X position offset from global origin (cm). val1=-1500..1500 */
	IWRL6322_ATTR_SENS_POS_X_CM,

	/** Sensor Y position offset from global origin (cm). val1=-1500..1500 */
	IWRL6322_ATTR_SENS_POS_Y_CM,

	/** Sensor Z (height) offset from ground plane (cm). val1=-1500..1500 */
	IWRL6322_ATTR_SENS_POS_Z_CM,

	/** Sensor azimuth tilt about Z axis (degrees). val1=-90..90 */
	IWRL6322_ATTR_SENS_TILT_AZIMUTH,

	/** Sensor elevation tilt about X axis (degrees). val1=-90..90 */
	IWRL6322_ATTR_SENS_TILT_ELEVATION,

	/** Sensor in-plane tilt about Y axis (degrees): -90, 0, or +90 only */
	IWRL6322_ATTR_SENS_IN_PLANE_TILT,

	/** GPADC parameter word (w_ParamVal: bits[9:2]=channel ctrl, bits[28:26]=signal type) */
	IWRL6322_ATTR_GPADC_PARAM,

	/** GPADC config word (w_CfgVal) */
	IWRL6322_ATTR_GPADC_CFG,

	/** Enable custom steering vectors (c_CustomSteerVecEnCtrl). val1=0(disable)/1(enable) */
	IWRL6322_ATTR_STEER_VEC_CUSTOM_ENA,

	/** Number of azimuth bins in steering vector table (c_NumAzimBins). val1=1..255 */
	IWRL6322_ATTR_STEER_VEC_NUM_AZIM_BINS,

	/** Number of elevation bins in steering vector table (c_NumElevBins). val1=1..255 */
	IWRL6322_ATTR_STEER_VEC_NUM_ELEV_BINS,

	/** Azimuth step size between bins (c_AzimStepSize). val1 in 0.1-deg units, 0..255 */
	IWRL6322_ATTR_STEER_VEC_AZIM_STEP,

	/** Elevation step size between bins (c_ElevStepSize). val1 in 0.1-deg units, 0..255 */
	IWRL6322_ATTR_STEER_VEC_ELEV_STEP,

	/** Steering vector element size (c_SvElemSize). val1=0(4-bit)/1(8-bit) */
	IWRL6322_ATTR_STEER_VEC_ELEM_SIZE,
};

/**
 * @brief Cartesian boundary and type for one detection zone.
 *
 * All coordinates are in centimetres (1 LSB = 1 cm).
 */
struct iwrl6322_zone_cfg {
	int16_t  xmin_cm;
	int16_t  xmax_cm;
	uint16_t ymin_cm;
	uint16_t ymax_cm;
	int16_t  zmin_cm;
	int16_t  zmax_cm;
	bool     include;
};

/**
 * @brief RTIO buffer header — metadata for self-describing sensor buffers.
 *
 * Layout in RTIO buffer:
 *   [struct iwrl6322_buffer_header] [device payload]
 *
 * Validate header->magic == IWRL6322_BUFFER_MAGIC before interpreting payload.
 */
struct iwrl6322_buffer_header {
	uint16_t magic;          /**< IWRL6322_BUFFER_MAGIC */
	uint8_t  operating_mode; /**< One of enum iwrl6322_operating_mode */
	uint8_t  reserved1;
	uint16_t payload_size;   /**< Payload bytes following this header */
	uint16_t reserved2;
	uint32_t timestamp_us;   /**< k_uptime_get_32() at capture */
};

/**
 * @brief Magic number for RTIO buffer header validation.
 */
#define IWRL6322_BUFFER_MAGIC 0x6322U

/**
 * @brief One detected target as produced by the decoder.
 */
struct iwrl6322_point_data {
	int16_t  x_cm;        /**< X position in global space, 1 LSB = 1 cm, signed */
	uint16_t y_cm;        /**< Y position (range), 1 LSB = 1 cm */
	int16_t  z_cm;        /**< Z position in global space, 1 LSB = 1 cm, signed */
	int16_t  velocity;    /**< Radial velocity */
	int16_t  snr_01db;    /**< SNR, 1 LSB = 0.1 dB, signed */
	int16_t  noise_01db;  /**< Noise, 1 LSB = 0.1 dB, signed */
};

/**
 * @brief Decoded point cloud output (zero-copy).
 *
 * readings points directly into the RTIO buffer and is valid as long
 * as the buffer is held by the application.
 */
struct iwrl6322_point_cloud_data {
	struct sensor_data_header           header;
	uint32_t                            frame_count;
	uint8_t                             zone_occupancy;
	const struct iwrl6322_point_data   *readings;
};

/**
 * @brief Decoded sensor info result from iwrl6322_decoder_get_sensor_info().
 *
 * Exactly one of points or raw_data is non-NULL depending on operating mode.
 */
struct iwrl6322_sensor_info {
	uint32_t frame_count;
	uint8_t  num_detections;
	uint8_t  zone_occupancy;
	const struct iwrl6322_point_data *points;    /**< MPD modes; NULL in FE modes */
	const uint16_t *raw_data;                    /**< FE modes; NULL in MPD modes */
	uint32_t        raw_data_len;
};

int iwrl6322_decoder_get_sensor_info(const uint8_t *buf, size_t buf_len,
				     struct iwrl6322_sensor_info *out);

/**
 * @brief Driver configuration (populated from devicetree at build time).
 */
struct iwrl6322_config {
	/** SPI bus + chip-select from the DT spi-device binding */
	struct spi_dt_spec spi;
	/** SPI_BUSY input GPIO — device drives this low when ready */
	struct gpio_dt_spec spi_busy_gpio;
	/** nRESET output GPIO (active-low) */
	struct gpio_dt_spec nreset_gpio;
	/** WAKE_UP output GPIO — keep device active / allow deep sleep (optional) */
	struct gpio_dt_spec wake_up_gpio;
	/** PRESENCE_DETECT input GPIO (optional, used for triggers) */
	struct gpio_dt_spec pres_det_gpio;
	/** SOP0 output GPIO — boot mode select (optional) */
	struct gpio_dt_spec sop0_gpio;
	/** SOP1 output GPIO — boot mode select (optional) */
	struct gpio_dt_spec sop1_gpio;
};

/**
 * @brief Driver runtime data.
 */
struct iwrl6322_data {
	/* ---- TI library state ------------------------------------------- */
	bool     firmware_loaded;    /* true once RBL download + switch succeeded */
	bool     cal_data_valid;     /* true once factory cal data retrieved successfully */

	struct k_mutex dev_mutex;
	struct k_sem dev_sem;

	/** Runtime-configurable sensor parameters (attr_set / Kconfig defaults) */
	uint8_t update_rate;    /**< Update rate: 1, 2, 4, or 5 Hz */
	bool sensor_running;    /**< True after a successful start command */
	uint16_t api_timeout_ms;/**< TI library API response timeout (1–65535 ms) */
	
	/* ---- Sensor mode configuration ----------------------------------- */
	bool    run_boot_calib;              /**< Run on-chip calibration at start */
	bool    store_cfg_to_flash;          /**< Write config to flash on next ml_sensorConfigApi */
	bool    sensor_location;             /**< false=indoor, true=outdoor */
	bool    high_power_mode;             /**< High-power TX mode */
	uint8_t region;                      /**< Regulatory region (enum iwrl6322_region) */
	uint8_t dev_sleep_hold_ms;           /**< Sleep hold time, 1 LSB = 0.5ms (0–255) */
	uint8_t override_ctrl;               /**< AdvanceConf override bitmask (0–15) */
	uint8_t tx_power_backoff;            /**< TX power backoff level (0–13) */
	uint8_t rf_start_freq;               /**< RF start frequency offset (0–55) */
	
	/* ---- Detection zone configuration -------------------------------- */
	struct iwrl6322_zone_cfg zones[16]; /**< Zone boundary and type (index = zone id) */
	uint8_t num_zones;                /**< Number of active zones (1–16) */
	bool    coordinate_cartesian;    /**< true=Cartesian (XYZ), false=Polar (R/Theta/Z) */
	
	/* ---- Device / app config ----------------------------------------- */
	uint8_t  dev_antenna_sel;	   /* b3_DevAntennaSel: 0 = AOP, 1 = WCSP 3D, 2 = WCSP 2D, 3 = WCSP Z*/
	uint8_t  smart_fe_op_mode;     /* b1_SmartFeOpMode: M_ML_DEV_OP_MODE_MPD_ENABLE or FE_ENABLE */
	uint8_t  data_tx_mode;         /* b2_DataTxMode: M_ML_DEV_OUTPUT_MODE_* */
	uint8_t  mpd_data_out_mode;    /* b1_MpdDataOutMode: M_ML_DEV_MPD_SIGNAL_OUT_MODE_* */
	bool     mpd_sig_trig_type;    /* b1_MpdSigTrigType: false=pulse, true=level */
	bool     mpd_auto_boot_mode;   /* b1_MpdAutoBootMode: auto-start SM from flash */
	uint8_t  mcu_clk_out_sel;      /* b2_McuClkOutSel: 0=off, 1=XTAL40M, 2=RC10M, 3=CPU/4 */
	bool     deep_sleep_entry_dis; /* b1_DeepSleepEntryDis: disable inter-frame deep sleep */
	bool     trig_boot_cal;        /* b1_TrigBootCal: trigger factory cal on next devAppConfig */
	bool     hil_enable;           /* b1_HilEnable */
	bool     jtag_enable;          /* b1_JtagEnable */
	bool     xtal_less_mode;       /* b1_XtalLessModeEna */
	bool     gpio_marker_ena;      /* b1_GpioMarkerEna: power-measurement marker */
	bool     cz_cw_mode_ena;       /* b1_CzCwModeEna */
	bool     rf_ldo_bypass;        /* b1_RfLdoBypass */
	bool     hold_lpds;            /* b1_HoldLpds: disable RTC wakeup from LPDS */
	bool	mount_type;				/* b1_MountType: 0 = wall-mount, 1 = ceiling-mount*/
	
	uint32_t mpd_holdtime_10us;  /* MPD signal-to-deep-sleep hold time, units of 10 µs */
	uint16_t fe_calib_ctrl;      /* h_FeCalibCtrl bitmask */
	
	/* ---- Start / stop config ----------------------------------------- */
	bool 	run_cal_ctrl;			/* 0 = One shot runtime calib enabled, 1 = disabled */
	uint8_t	mpd_meas_prof_sel;		/* profile to use at sensor start (0-3 or M_ML_SENS_MPD_MEAS_AUTO) */
	uint8_t run_cal_trig_interval;	/* c_RunCalTrigInterval: 0=disabled, 1-255 frames */
	bool    flash_sens_cfg_data;    /* c_FlashSensCfgData: flash config for auto-boot */
	
	/* ---- MPD profile ------------------------------------------------- */
	uint8_t  mpd_detection_mode; 		/* 1=1VA-1D, 2=2VA-2D, 3=3VA-3D, 4=4VA-3D */
	uint8_t max_range_m;    			/**< Hardware maximum detection range. 0 = 4m, 1 = 8m, 2 = 12m */
	uint8_t motion_sensitivity;         /**< Motion sensitivity (0–3) */
	uint8_t detection_sensitivity;      /**< Detection sensitivity (0–5) */
	uint8_t  mpd_snr_boost;      		/* 0=disabled, 1=level1, 2=level2 */
	bool    disable_static_obj_removal; /**< Bypass static-object removal */
	uint16_t mpd_num_frames;     		/* 0=infinite */
	uint32_t frame_period_ms;    		/* MPD frame period in ms; also via SENSOR_ATTR_SAMPLING_FREQUENCY */
	int8_t   mpd_adv_cfar_cfg;      	/* xc_AdvCfarCfg: CFAR offset -6..+6 */
	uint8_t	mpd_min_points;				/* Minimum number of detected points required to form a cluster (1 - 255)*/
	int16_t	mpd_max_distance_cluster;	/* Maximum distance between detected points for the points to be grouped into the same cluster (1 - 32767, 1 LSB = 1 cm)*/
	uint8_t	mpd_steer_vec_en_ctrl;		/* c_SteerVecEnCtrl per profile: 0=enable, 1=disable */

	/* ---- Custom steering vector configuration (ml_smartfeSteerVecCfg) --- */
	bool    steer_vec_custom_ena;     /* c_CustomSteerVecEnCtrl: enable custom sv table */
	uint8_t steer_vec_num_azim_bins;  /* c_NumAzimBins: 1..255 */
	uint8_t steer_vec_num_elev_bins;  /* c_NumElevBins: 1..255 */
	uint8_t steer_vec_azim_step;      /* c_AzimStepSize: 0.1 deg/LSB, 0..255 */
	uint8_t steer_vec_elev_step;      /* c_ElevStepSize: 0.1 deg/LSB, 0..255 */
	uint8_t steer_vec_elem_size;      /* c_SvElemSize: 0=4-bit, 1=8-bit */

	/** Sensor mount configuration (maps to M_ML_SENSOR_MOUNT_CFG) */
	int16_t  mount_offset_x_cm;  /**< Lateral offset from origin, cm (±1500) */
	uint16_t mount_offset_y_cm;  /**< Forward offset from origin, cm (0..1500) */
	uint16_t mount_offset_z_cm;  /**< Height offset from origin, cm (0..1500) */
	int8_t   tilt_azimuth_deg;   /**< Azimuth tilt about Z-axis, degrees (±90) */
	int8_t   tilt_elevation_deg; /**< Elevation tilt about X-axis, degrees (±90) */

	/** GPIO callback for PRESENCE_DETECT edge interrupt */
	struct gpio_callback pres_det_cb;

	/** Handler for SENSOR_TRIG_DATA_READY (rising edge of the PRESENCE_DETECT pulse) */
	sensor_trigger_handler_t data_ready_handler;
	const struct sensor_trigger *data_ready_trigger;

	/** Work item that invokes trigger handlers from the system workqueue */
	struct k_work trig_work;

	/**
	 * Active streaming SQE (set when iodev_submit receives is_streaming=true).
	 * The trig_work_handler allocates a buffer, fetches a frame directly into
	 * it, and completes this SQE.  RTIO re-submits the multishot SQE
	 * automatically after each completion.  NULL when no stream is active.
	 */
	struct rtio_iodev_sqe *streaming_sqe;

	/* FE chirp profile */
	uint8_t  fe_adc_samp_rate;     /* c_AdcSampRate: effective = 400 MHz / (val * 2) */
	uint8_t  fe_num_virtual_ant;   /* 1..4 */
	uint16_t fe_txrx_ch_ena;       /* h_ChirpTxRxChEnaSel bitmask */
	uint16_t fe_num_adc_samples;   /* 32/64/128/256 */
	uint16_t fe_ramp_end_time;     /* units of 100 ns */
	uint16_t fe_idle_time;         /* units of 100 ns */
	uint8_t  fe_adc_skip_samp;     /* 0..63 */
	uint32_t fe_rf_freq_start;     /* 1 LSB ≈ 1.172 MHz */
	int16_t  fe_rf_freq_slope;     /* signed, 1 LSB ≈ 28.61 kHz/µs */
	int16_t  fe_tx_start_time;     /* signed, units of 20 ns */
	uint8_t  fe_hpf_corner;        /* 0=175 kHz, 1=350 kHz, 2=700 kHz, 3=1400 kHz */
	uint8_t  fe_cal_rx_gain;       /* 34..48 dB, even values */
	uint8_t  fe_cal_tx_backoff;    /* 0..15 dB */
	uint8_t  fe_inter_chirp_ps_ovrd;

	/* FE chirp frame */
	uint8_t  fe_chirps_accum;      /* 0=disabled, 2..64 */
	uint16_t fe_chirps_per_burst;  /* 1..1023 */
	uint32_t fe_burst_idle_time;   /* units of 100 ns */
	uint8_t  fe_bursts_per_frame;  /* 1..127 */
	uint16_t fe_num_frames;        /* 0=infinite */
	uint32_t fe_frame_period_ms;   /* frame period in ms */

	/* Sensor position and tilt (zone configuration)                        */
	int16_t  sens_pos_x_cm;          /* xh_SensPosXRange: ±1500 cm */
	int16_t  sens_pos_y_cm;          /* xh_SensPosYTheta: ±1500 cm */
	int16_t  sens_pos_z_cm;          /* xh_SensPosZ: ±1500 cm */
	int8_t   sens_tilt_azimuth;      /* xc_SensTiltAzimuth: ±90 deg */
	int8_t   sens_tilt_elevation;    /* xc_SensTiltElevation: ±90 deg */
	int8_t   sens_in_plane_tilt;     /* xc_InPlaneTilt: 0, ±90 deg */

	/* Reset and power-up timing (init-time only)                           */
	uint8_t  nreset_setup_time_10us; /* c_NResetSetupTime10us: 0=default 20 µs */
	uint8_t  rbl_powerup_delay_ms;   /* c_RblPowerUpDelayMs: 0=default 15 ms */

	uint8_t fe_cal_data[128];
};

/**
 * Implemented in iwrl6322_decoder.c; registered in iwrl6322_driver_api.
 */
int iwrl6322_get_decoder(const struct device *dev,
			 const struct sensor_decoder_api **decoder);

/**
 * @brief Assert (level=1) or deassert (level=0) the WAKE_UP GPIO.
 *
 * @param dev Device instance
 * @param level 1=assert (keep device active), 0=deassert (allow sleep)
 */
void iwrl6322_wakeup(const struct device *dev, int level);
void iwrl6322_nreset(const struct device *dev, int level);

#ifdef __cplusplus
}
#endif

#endif /* IWRL6322_OPS_H_ */
