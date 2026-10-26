/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 *
 * TI IWRL6322 60-64 GHz mmWave radar sensor driver.
 *
 * Wraps the TI mmWaveULink 2.X HAL (ml_mmWaveLinkInit / ml_rblImageDownload /
 * ml_devAppConfig / ml_devSensorStartStop / ml_devSensorGetData, etc.) and
 * exposes data via the Zephyr RTIO streaming API.
 *
 * Sensor data is only accessible via RTIO streaming (sensor_stream / sensor_read):
 *
 *   Buffer layout: [struct iwrl6322_buffer_header] [device payload]
 *
 *   MPD modes (IWRL6322_MODE_MPD_PRESENCE / MPD_VERBOSE):
 *     Payload = struct iwrl6322_pc_payload_hdr + T_ML_SENS_POINT_CLOUD_DATA[]
 *
 *   FE modes (IWRL6322_MODE_RAW_ADC / 1D_FFT):
 *     Payload = raw 16-bit ADC or 1D-FFT samples
 *
 * NOTE: The TI library uses a single global state struct (z_RlHostData).
 * Only one hardware instance is therefore supported at a time.
 */

#define DT_DRV_COMPAT ti_iwrl6322

#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/work.h>
#include <string.h>
#include <stdarg.h>

#include "iwrl6322.h"

/* TI mmWaveULink 2.X headers*/
#include <ml_datatypes.h>
#include <mmic-binaries/udfp.h>
#include <mmwaveulink/include/ml_dev.h>
#include <mmwaveulink/include/ml_fe.h>
#include <mmwaveulink/include/ml_smartfe.h>
#include <radarlink/include/rl_interface.h>
#include <radarlink/include/rl_host.h>

LOG_MODULE_DECLARE(ti_iwrl6322, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Module-level device reference for platform/OS callbacks.
 * The TI library does not carry user context through most callbacks, so we
 * store one pointer here. This limits the driver to a single instance.
 */
static const struct device *g_dev;

/*
 * Convenience pointer to dev->data — avoids casting g_dev->data in every
 * callback. Initialized in iwrl6322_device_init() before any library call.
 */
static struct iwrl6322_data *g_priv;

/* Dedicated work queue — keeps sensor SPI work off the system work queue and
 * provides a predictable priority and stack large enough for ml_sensorInfoGetApi(). */
K_THREAD_STACK_DEFINE(iwrl6322_work_q_stack, CONFIG_IWRL6322_WORK_QUEUE_STACK_SIZE);
static struct k_work_q iwrl6322_work_q;

/* ============================================================================
 * Internal helpers
 * ========================================================================== */

/*
 * Swap adjacent byte pairs in-place for 16-bit MSB-first SPI wire order.
 */
static void swap_byte_pairs_inplace(uint8_t *buf, uint32_t len)
{
	for (uint32_t i = 0; i + 1 < len; i += 2) {
		uint8_t tmp = buf[i];

		buf[i]     = buf[i + 1];
		buf[i + 1] = tmp;
	}
}

/* ============================================================================
 * OS Callbacks
 * ========================================================================== */

static T_ML_STATUS iwrl6322_mutex_create(T_ML_OS_MUTEX_HANDLE *p_hdl, UINT8 *p_name)
{
	ARG_UNUSED(p_name);
	k_mutex_init(&g_priv->dev_mutex);
	*p_hdl = (T_ML_OS_MUTEX_HANDLE)&g_priv->dev_mutex;
	return M_RL_RET_CODE_OK;
}

static T_ML_STATUS iwrl6322_mutex_lock(T_ML_OS_MUTEX_HANDLE *p_hdl, UINT32 w_timeoutUs)
{
	struct k_mutex *mutex = (struct k_mutex *)(*p_hdl);
	k_timeout_t to = (w_timeoutUs == (UINT32)0xFFFFFFFFU)
			 ? K_FOREVER : K_USEC(w_timeoutUs);

	return k_mutex_lock(mutex, to) == 0 ? M_RL_RET_CODE_OK : M_RL_RET_CODE_OSIF_STATUS_FAIL;
}

static T_ML_STATUS iwrl6322_mutex_unlock(T_ML_OS_MUTEX_HANDLE *p_hdl)
{
	struct k_mutex *mutex = (struct k_mutex *)(*p_hdl);

	return k_mutex_unlock(mutex) == 0 ? M_RL_RET_CODE_OK : M_RL_RET_CODE_OSIF_STATUS_FAIL;
}

static T_ML_STATUS iwrl6322_mutex_delete(T_ML_OS_MUTEX_HANDLE *hdl)
{
	ARG_UNUSED(hdl);
	return M_RL_RET_CODE_OK;
}

/* Semaphore callbacks — reserved for IWRL6322 per TI datasheet; stubs provided. */
static T_ML_STATUS iwrl6322_sem_create(T_ML_OS_SEMAPHORE_HANDLE *p_hdl, UINT8 *p_name)
{
	ARG_UNUSED(p_name);
	k_sem_init(&g_priv->dev_sem, 0, 1);
	*p_hdl = (T_ML_OS_SEMAPHORE_HANDLE)&g_priv->dev_sem;
	return M_RL_RET_CODE_OK;
}

static T_ML_STATUS iwrl6322_sem_post(T_ML_OS_SEMAPHORE_HANDLE *p_hdl)
{
	k_sem_give((struct k_sem *)(*p_hdl));
	return M_RL_RET_CODE_OK;
}

static T_ML_STATUS iwrl6322_sem_wait(T_ML_OS_SEMAPHORE_HANDLE *p_hdl, UINT32 w_timeoutUs)
{
	k_timeout_t to = (w_timeoutUs == (UINT32)0xFFFFFFFFU)
			 ? K_FOREVER : K_USEC(w_timeoutUs);

	return k_sem_take((struct k_sem *)(*p_hdl), to) == 0
	       ? M_RL_RET_CODE_OK : M_RL_RET_CODE_OSIF_STATUS_FAIL;
}

/* ============================================================================
 * Platform / Pin Callbacks
 * ========================================================================== */

static T_ML_STATUS iwrl6322_blocking_delay(UINT32 w_delayUs)
{
	k_busy_wait(w_delayUs);
	return M_RL_RET_CODE_OK;
}

static T_ML_STATUS iwrl6322_task_delay(UINT32 w_delayUs)
{
	k_sleep(K_USEC(w_delayUs));
	return M_RL_RET_CODE_OK;
}

static T_ML_STATUS iwrl6322_compute_crc(const UINT8 *p_data, UINT32 w_len, UINT32 *p_crc)
{
	*p_crc = crc32_ieee(p_data, w_len);
	return M_RL_RET_CODE_OK;
}

static void *iwrl6322_memcpy(void *dst, const void *src, UINT32 n)
{
	return memcpy(dst, src, n);
}

static void *iwrl6322_memset(void *dst, const SINT32 val, UINT32 n)
{
	return memset(dst, (int)val, n);
}

static T_ML_STATUS iwrl6322_printf(const UINT8 fmt[], ...)
{
#ifdef CONFIG_IWRL6322_DEBUG_LOG
	char buf[128];
	va_list args;

	va_start(args, fmt);
	vsnprintk(buf, sizeof(buf), (const char *)fmt, args);
	va_end(args);
	LOG_DBG("%s", buf);
#else
	ARG_UNUSED(fmt);
#endif
	return M_RL_RET_CODE_OK;
}

/*
 * Drive nRESET or WAKE_UP GPIO.
 * Called from the TI library — must NOT call iwrl6322_wakeup() (that would
 * re-enter ml_devCtrl and cause unbounded recursion for WAKEUP_REQ).
 */
static T_ML_STATUS iwrl6322_device_pin_ctrl(UINT8 c_devIdx, UINT8 c_pinName, UINT8 c_pinState)
{
	const struct iwrl6322_config *cfg = g_dev->config;

	ARG_UNUSED(c_devIdx);

	switch (c_pinName) {
	case M_ML_DEVICE_PIN_TYPE_NRESET:
		if (cfg->nreset_gpio.port != NULL) {
			gpio_pin_set_dt(&cfg->nreset_gpio, (int)c_pinState);
		}
		break;
	case M_ML_DEVICE_PIN_TYPE_WAKEUP_REQ:
		/* Drive GPIO directly — do NOT call iwrl6322_wakeup() here */
		if (cfg->wake_up_gpio.port != NULL) {
			gpio_pin_set_dt(&cfg->wake_up_gpio, (int)c_pinState);
		}
		break;
	case M_ML_DEVICE_PIN_TYPE_SOP_SELECT:
		/* SOP select not wired; ignore */
		break;
	default:
		return M_RL_RET_CODE_INVALID_CONFIG;
	}
	return M_RL_RET_CODE_OK;
}

/*
 * Wait-pin-status callback — polls a GPIO until it reaches the desired state
 * or the timeout elapses.
 *
 * Pin type macros:
 *   M_ML_DEVICE_PIN_TYPE_DEV_BUSY (3) — SPI_BUSY GPIO
 *   M_ML_DEVICE_PIN_TYPE_MPD      (4) — PRESENCE_DETECT GPIO
 */
static T_ML_STATUS iwrl6322_wait_pin_status(UINT8 c_devIdx, UINT8 c_pinName,
					   UINT8 c_pinState, UINT32 w_timeoutUs)
{
	const struct iwrl6322_config *cfg = g_dev->config;
	const struct gpio_dt_spec *gpio = NULL;

	ARG_UNUSED(c_devIdx);

	switch (c_pinName) {
	case M_ML_DEVICE_PIN_TYPE_DEV_BUSY:
		gpio = &cfg->spi_busy_gpio;
		break;
	case M_ML_DEVICE_PIN_TYPE_MPD:
		gpio = (cfg->pres_det_gpio.port != NULL) ? &cfg->pres_det_gpio : NULL;
		break;
	default:
		return M_RL_RET_CODE_INVALID_CONFIG;
	}

	if (gpio == NULL) {
		return M_RL_RET_CODE_OK;
	}

	uint32_t elapsed = 0U;
	const uint32_t step_us = 10U;

	while (elapsed < w_timeoutUs) {
		int val = gpio_pin_get_dt(gpio);

		if (val == (int)c_pinState) {
			return M_RL_RET_CODE_OK;
		}
		k_busy_wait(step_us);
		elapsed += step_us;
	}

	return M_RL_RET_CODE_SPIBUSY_TIMEOUT;
}

static void iwrl6322_mask_pres_irq(UINT8 c_devIdx)
{
	ARG_UNUSED(c_devIdx);
	/* Edge-triggered interrupt; no masking needed */
}

static void iwrl6322_unmask_pres_irq(UINT8 c_devIdx)
{
	ARG_UNUSED(c_devIdx);
}

/* ============================================================================
 * Communication Interface (SPI) Callbacks
 * ========================================================================== */

static T_ML_COMM_IF_HANDLE iwrl6322_comm_open(UINT8 c_devIdx, UINT32 w_flags)
{
	ARG_UNUSED(c_devIdx);
	ARG_UNUSED(w_flags);
	/*
	 * Return the Zephyr device pointer as an opaque handle.
	 * comm_read/write/close cast it back to retrieve cfg->spi.
	 */
	return (T_ML_COMM_IF_HANDLE)g_dev;
}

static T_ML_STATUS iwrl6322_comm_close(T_ML_COMM_IF_HANDLE *p_hdl)
{
	ARG_UNUSED(p_hdl);
	return M_RL_RET_CODE_OK;
}

/*
 * SPI read — clocks in device response while sending the pattern bytes.
 * UINT32 return: number of bytes transferred, or 0 on error.
 */
static UINT32 iwrl6322_comm_read(T_ML_COMM_IF_HANDLE *p_hdl,
				UINT8 *c_rxBuf, const UINT8 *c_patternBuf,
				UINT32 w_size)
{
	const struct device *dev = (const struct device *)(*p_hdl);
	const struct iwrl6322_config *cfg = dev->config;
	const struct spi_buf tx = {.buf = (void *)c_patternBuf, .len = w_size};
	const struct spi_buf rx = {.buf = c_rxBuf,              .len = w_size};
	const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};
	int ret;

	ret = spi_transceive(cfg->spi.bus, &cfg->spi.config, &tx_set, &rx_set);
	if (ret == 0) {
		swap_byte_pairs_inplace(c_rxBuf, w_size);
	}
	return ret == 0 ? w_size : 0U;
}

/*
 * SPI write — transmits command bytes, optionally receiving the pattern echo.
 * UINT32 return: number of bytes transferred, or 0 on error.
 */
static UINT32 iwrl6322_comm_write(T_ML_COMM_IF_HANDLE *p_hdl,
				 UINT8 *c_txBuf, UINT8 *c_patternBuf,
				 UINT32 w_size)
{
	const struct device *dev = (const struct device *)(*p_hdl);
	const struct iwrl6322_config *cfg = dev->config;
	/* Stack buffer for byte-swapped TX data — chunks must be even-sized for
	 * the 16-bit byte-pair swap to stay aligned across chunk boundaries. */
	uint8_t tx_swap[M_RL_MAX_API_PAYLOAD_SIZE + M_RL_PKT_HDR_SIZE + 16U];
	const uint32_t chunk_max = sizeof(tx_swap);
	uint32_t offset = 0U;

	while (offset < w_size) {
		uint32_t chunk = w_size - offset;

		if (chunk > chunk_max) {
			chunk = chunk_max;
		}

		memcpy(tx_swap, c_txBuf + offset, chunk);
		swap_byte_pairs_inplace(tx_swap, chunk);

		const struct spi_buf tx_b = {.buf = tx_swap, .len = chunk};
		const struct spi_buf_set tx_set = {.buffers = &tx_b, .count = 1};
		int ret;

		if (c_patternBuf != NULL) {
			const struct spi_buf rx = {.buf = c_patternBuf + offset, .len = chunk};
			const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

			ret = spi_transceive(cfg->spi.bus, &cfg->spi.config, &tx_set, &rx_set);
		} else {
			ret = spi_write(cfg->spi.bus, &cfg->spi.config, &tx_set);
		}

		if (ret != 0) {
			return 0U;
		}

		offset += chunk;
	}

	return w_size;
}

/* ============================================================================
 * TI Library Initialisation
 * ========================================================================== */

/*
 * Populate a T_ML_LINK_INIT_CFG with all required Zephyr callbacks.
 */
static void iwrl6322_build_init_cfg(T_ML_LINK_INIT_CFG *cfg, uint16_t timeout_ms)
{
	memset(cfg, 0, sizeof(*cfg));

	/* Protocol: SPI, device-busy GPIO synchronisation, CRC and pattern disabled */
	cfg->z_ProtocolCtrl.bits.b2_CommInterface = M_ML_COMM_INTERFACE_SPI;
	cfg->z_ProtocolCtrl.bits.b1_CrcEnabled    = M_ML_CRC_CHECK_DISABLE;
	cfg->z_ProtocolCtrl.bits.b1_PatternCtrl   = M_ML_PATTERN_CTRL_DISABLE;
	cfg->z_ProtocolCtrl.bits.b1_ErrorRecovery = 0;
	cfg->z_ProtocolCtrl.bits.b1_DeviceBusy    = 0; /* 0 = M_ML_DEVICE_BUSY_CHECK_ENABLE */
	cfg->z_ProtocolCtrl.bits.b2_UartBaud      = 0;
	cfg->z_ProtocolCtrl.bits.b2_SpiMode       = 0;

	cfg->h_DeviceRspTimeoutMs = timeout_ms;

	/* OS — mutex */
	cfg->z_OsCallbackCfg.p_MutexCreate  = iwrl6322_mutex_create;
	cfg->z_OsCallbackCfg.p_MutexLock    = iwrl6322_mutex_lock;
	cfg->z_OsCallbackCfg.p_MutexUnlock  = iwrl6322_mutex_unlock;
	cfg->z_OsCallbackCfg.p_MutexDelete  = iwrl6322_mutex_delete;

	/* OS — semaphore (reserved for IWRL6322 but registered for safety) */
	cfg->z_OsCallbackCfg.p_SemaphoreCreate = iwrl6322_sem_create;
	cfg->z_OsCallbackCfg.p_SemaphorePost   = iwrl6322_sem_post;
	cfg->z_OsCallbackCfg.p_SemaphorePend   = iwrl6322_sem_wait;

	/* OS — utility */
	cfg->z_OsCallbackCfg.p_BlockingDelayUs = iwrl6322_blocking_delay;
	cfg->z_OsCallbackCfg.p_TaskDelayUs     = iwrl6322_task_delay;
	cfg->z_OsCallbackCfg.p_ComputeCrc      = iwrl6322_compute_crc;
	cfg->z_OsCallbackCfg.p_MemCpy          = iwrl6322_memcpy;
	cfg->z_OsCallbackCfg.p_MemSet          = iwrl6322_memset;
	cfg->z_OsCallbackCfg.p_Printf          = iwrl6322_printf;

	/* Device control */
	cfg->z_DeviceCallbackCfg.p_DevicePinControl    = iwrl6322_device_pin_ctrl;
	cfg->z_DeviceCallbackCfg.p_WaitPinStatus       = iwrl6322_wait_pin_status;
	cfg->z_DeviceCallbackCfg.p_MaskPresDetIrq      = iwrl6322_mask_pres_irq;
	cfg->z_DeviceCallbackCfg.p_UnmaskPresDetIrq    = iwrl6322_unmask_pres_irq;
	cfg->z_DeviceCallbackCfg.c_NResetSetupTime10us = g_priv->nreset_setup_time_10us;
	cfg->z_DeviceCallbackCfg.c_RblPowerUpDelayMs   = g_priv->rbl_powerup_delay_ms;

	/* Communication interface */
	cfg->z_CommCallbackCfg.p_CommIfOpen  = iwrl6322_comm_open;
	cfg->z_CommCallbackCfg.p_CommIfClose = iwrl6322_comm_close;
	cfg->z_CommCallbackCfg.p_CommIfRead  = iwrl6322_comm_read;
	cfg->z_CommCallbackCfg.p_CommIfWrite = iwrl6322_comm_write;
	cfg->z_CommCallbackCfg.w_CommTimePerByteUs = 10;

	/* Timeouts (ONLY RELEVNAT WHEN DEVICE BUSY DISABLED)*/
	cfg->z_timeouts.h_rblSetupTimeUs   = 0;
	cfg->z_timeouts.h_cmdHdrTimeUs 	   = 0;
	cfg->z_timeouts.h_cmdLngMsgTimeUs  = 0;
	cfg->z_timeouts.h_respLngMsgTimeUs = 0;
}

/* ============================================================================
 * Firmware Loading
 * ========================================================================== */

/*
 * Load firmware via the ROM Bootloader (RBL) and switch to application mode.
 *
 * On entry the device must be in RBL state (recently released from nRESET).
 * The firmware blob must be provided by the application as the external symbol
 * pair (iwrl6322_fw_data / iwrl6322_fw_size), typically via a generated C file
 * or a linker INCBIN.  If no firmware is registered (size == 0), the driver
 * falls back to booting from on-chip flash.
 *
 * Returns 0 on success, negative errno on failure.
 */
// const uint8_t iwrl6322_fw_data[] __attribute__((weak));
const size_t iwrl6322_fw_size = METAIMAGE_SIZE;

static int iwrl6322_load_firmware(const struct device *dev)
{
	ARG_UNUSED(dev);
	T_ML_STATUS ret;
	T_ML_RBL_APPLICATION_ENTRY_CFG entry_cfg = {0};
	T_ML_RBL_APPLICATION_ENTRY_RSP entry_rsp = {0};

	if (1) {
		T_ML_RBL_IMAGE_DOWNLOAD_CFG img_cfg = {
			.w_ImageSrcAddress = (UINT32)(&metaimage_data[0]),
			.w_ImageSize       = METAIMAGE_SIZE
		};

		LOG_INF("IWRL6322: downloading firmware (%zu bytes)", iwrl6322_fw_size);
		ret = ml_rblImageDownload(0U, &img_cfg);
		if (ret != M_RL_RET_CODE_OK) {
			LOG_ERR("IWRL6322: rblImageDownload failed: %d", ret);
			return -EIO;
		}
		entry_cfg.h_BootMode = M_ML_RBL_APP_ENTRY_MODE_SERIAL;
	} else {
		LOG_INF("IWRL6322: booting from flash");
		entry_cfg.h_BootMode = M_ML_RBL_APP_ENTRY_MODE_FLASH;
	}

	ret = ml_rblSwitchToApplication(0U, &entry_cfg, &entry_rsp);
	if (ret != M_RL_RET_CODE_OK) {
		LOG_ERR("IWRL6322: rblSwitchToApplication failed: %d", ret);
		return -EIO;
	}

	if (entry_rsp.w_ImageLoadSts[0] != 0x5A5AA977U) {
		LOG_ERR("IWRL6322: firmware boot failed, ImageLoadSts[0]=0x%08X",
			entry_rsp.w_ImageLoadSts[0]);
		return -EIO;
	}

	LOG_INF("IWRL6322: firmware loaded (RBL v0x%04X)", entry_rsp.h_RblVersion);
	g_priv->firmware_loaded = true;
	return 0;
}

/* ============================================================================
 * Sensor Configuration
 * ========================================================================== */

/* Reconstruct the driver-level operating mode from SDK bitfields. */
static enum iwrl6322_operating_mode iwrl6322_get_mode(const struct iwrl6322_data *data)
{
	if (!data->firmware_loaded || !data->sensor_running) {
		return IWRL6322_MODE_STOP;
	}
	if (data->smart_fe_op_mode == M_ML_DEV_OP_MODE_FE_ENABLE) {
		return (data->data_tx_mode == M_ML_DEV_OUTPUT_MODE_RAW_ADC)
		       ? IWRL6322_MODE_RAW_ADC : IWRL6322_MODE_1D_FFT;
	}
	return (data->mpd_data_out_mode == M_ML_DEV_MPD_SIGNAL_OUT_MODE_VERBOSE)
	       ? IWRL6322_MODE_MPD_VERBOSE : IWRL6322_MODE_MPD_PRESENCE;
}

/* Translate driver operating mode enum to the three SDK AppCfgCtrl bitfields. */
static void iwrl6322_set_mode_bitfields(struct iwrl6322_data *data, int mode)
{
	switch (mode) {
	case IWRL6322_MODE_RAW_ADC:
		data->smart_fe_op_mode  = M_ML_DEV_OP_MODE_FE_ENABLE;
		data->data_tx_mode      = M_ML_DEV_OUTPUT_MODE_RAW_ADC;
		data->mpd_data_out_mode = M_ML_DEV_MPD_SIGNAL_OUT_MODE_VERBOSE;
		break;
	case IWRL6322_MODE_1D_FFT:
		data->smart_fe_op_mode  = M_ML_DEV_OP_MODE_FE_ENABLE;
		data->data_tx_mode      = M_ML_DEV_OUTPUT_MODE_1DFFT;
		data->mpd_data_out_mode = M_ML_DEV_MPD_SIGNAL_OUT_MODE_VERBOSE;
		break;
	case IWRL6322_MODE_MPD_PRESENCE:
		data->smart_fe_op_mode  = M_ML_DEV_OP_MODE_MPD_ENABLE;
		data->data_tx_mode      = M_ML_DEV_OUTPUT_MODE_MPD_DATA;
		data->mpd_data_out_mode = M_ML_DEV_MPD_SIGNAL_OUT_MODE_QUIET;
		break;
	default: /* IWRL6322_MODE_MPD_VERBOSE */
		data->smart_fe_op_mode  = M_ML_DEV_OP_MODE_MPD_ENABLE;
		data->data_tx_mode      = M_ML_DEV_OUTPUT_MODE_MPD_DATA;
		data->mpd_data_out_mode = M_ML_DEV_MPD_SIGNAL_OUT_MODE_VERBOSE;
		break;
	}
}

/*
 * Configure the device using the SDK bitfields stored in g_priv.
 * Call iwrl6322_set_mode_bitfields() before this to set the desired mode.
 */
static int iwrl6322_configure(const struct device *dev)
{
	T_ML_STATUS ret;
	ARG_UNUSED(dev);

	/* --- Device application config ------------------------------------ */
	T_ML_DEVICE_APP_CONFIG app_cfg = {0};

	app_cfg.z_AppCfgCtrl.bits.b3_DevAntennaSel    = g_priv->dev_antenna_sel;
	app_cfg.z_AppCfgCtrl.bits.b1_SmartFeOpMode    = g_priv->smart_fe_op_mode;
	app_cfg.z_AppCfgCtrl.bits.b2_DataTxMode       = g_priv->data_tx_mode;
	app_cfg.z_AppCfgCtrl.bits.b1_MpdDataOutMode   = g_priv->mpd_data_out_mode;
	app_cfg.z_AppCfgCtrl.bits.b1_MpdSigTrigType   =
		g_priv->mpd_sig_trig_type
		? M_ML_DEV_MPD_SIGNAL_OUT_TRIGTYPE_LEVEL
		: M_ML_DEV_MPD_SIGNAL_OUT_TRIGTYPE_PULSE;
	app_cfg.z_AppCfgCtrl.bits.b1_MpdAutoBootMode  	= g_priv->mpd_auto_boot_mode  ? 1U : 0U;
	app_cfg.z_AppCfgCtrl.bits.b2_McuClkOutSel     	= g_priv->mcu_clk_out_sel; //ti internal
	app_cfg.z_AppCfgCtrl.bits.b1_DeepSleepEntryDis 	= g_priv->deep_sleep_entry_dis ? 1U : 0U; //ti internal
	app_cfg.z_AppCfgCtrl.bits.b2_GlobalRegion     	= g_priv->region;
	app_cfg.z_AppCfgCtrl.bits.b1_TrigBootCal      	= g_priv->trig_boot_cal       ? 1U : 0U;
	app_cfg.z_AppCfgCtrl.bits.b1_HilEnable        	= g_priv->hil_enable           ? 1U : 0U; //ti internal
	app_cfg.z_AppCfgCtrl.bits.b1_JtagEnable       	= g_priv->jtag_enable          ? 1U : 0U; //ti internal
	app_cfg.z_AppCfgCtrl.bits.b1_XtalLessModeEna  	= g_priv->xtal_less_mode       ? 1U : 0U; //ti internal
	app_cfg.z_AppCfgCtrl.bits.b1_GpioMarkerEna    	= g_priv->gpio_marker_ena      ? 1U : 0U; //ti internal
	app_cfg.z_AppCfgCtrl.bits.b1_CzCwModeEna      	= g_priv->cz_cw_mode_ena       ? 1U : 0U; //ti internal
	app_cfg.z_AppCfgCtrl.bits.b1_RfLdoBypass      	= g_priv->rf_ldo_bypass        ? 1U : 0U; //ti internal
	app_cfg.z_AppCfgCtrl.bits.b1_HoldLpds         	= g_priv->hold_lpds            ? 1U : 0U; //ti internal
	app_cfg.z_AppCfgCtrl.bits.b1_MountType			= g_priv->mount_type           ? 1U : 0U;

	app_cfg.w_MpdSigOutToDsHoldTime = g_priv->mpd_holdtime_10us;
	app_cfg.h_FeCalibCtrl = g_priv->fe_calib_ctrl;

	bool is_fe_mode = (g_priv->smart_fe_op_mode == M_ML_DEV_OP_MODE_FE_ENABLE);

	ret = ml_devAppConfig(0U, &app_cfg);
	if (ret != M_RL_RET_CODE_OK) {
		LOG_ERR("IWRL6322: devAppConfig failed: %d", ret);
		return -EIO;
	}

	if(g_priv->trig_boot_cal){
		iwrl6322_wakeup(dev, 0);
		k_msleep(100);
		iwrl6322_wakeup(dev, 1);

		ret = ml_devFactCalDataGet(0U, (T_ML_FE_CAL_DATA *)g_priv->fe_cal_data);
		if (ret == M_RL_RET_CODE_OK) {
			// g_priv->cal_data_valid = true;
		} else {
			LOG_WRN("IWRL6322: devFactCalDataGet failed: %d", ret);
		}
	}

	/* --- Mode-specific configuration ---------------------------------- */
	if (is_fe_mode) {
		/* FE chirp profile */
		T_ML_FE_CHIRP_PROF_CONFIG prof = {0};

		prof.c_AdcSampRate           = g_priv->fe_adc_samp_rate;
		prof.c_NumVirtualAntennas    = g_priv->fe_num_virtual_ant;
		prof.h_ChirpTxRxChEnaSel     = g_priv->fe_txrx_ch_ena;
		prof.h_NumOfAdcSamples       = g_priv->fe_num_adc_samples;
		prof.h_ChirpRampEndTime      = g_priv->fe_ramp_end_time;
		prof.h_ChirpIdleTime         = g_priv->fe_idle_time;
		prof.c_ChirpAdcStartSkipSamp = g_priv->fe_adc_skip_samp;
		prof.w_ChirpRfFreqStart      = g_priv->fe_rf_freq_start;
		prof.xh_ChirpRfFreqSlope     = g_priv->fe_rf_freq_slope;
		prof.xh_TxStartTime          = g_priv->fe_tx_start_time;
		prof.c_ChirpRxHpfSel         = g_priv->fe_hpf_corner;
		prof.c_CalRxGainSel          = g_priv->fe_cal_rx_gain;
		prof.c_CalTxBackOffSel	     = g_priv->fe_cal_tx_backoff;
		prof.c_InterChirpPsOvrd      = g_priv->fe_inter_chirp_ps_ovrd; //todo enable full configuration of this setting

		ret = ml_feChirpProfileCfg(0U, &prof);
		if (ret != M_RL_RET_CODE_OK) {
			LOG_ERR("IWRL6322: feChirpProfileCfg failed: %d", ret);
			return -EIO;
		}

		/* FE chirp frame */
		T_ML_FE_CHIRP_FRAME_CONFIG frame = {0};

		frame.c_NumOfChirpsAccum   = g_priv->fe_chirps_accum;
		frame.h_NumOfChirpsInBurst = g_priv->fe_chirps_per_burst;
		frame.w_BurstIdleTime      = g_priv->fe_burst_idle_time;
		frame.h_NumOfBurstsInFrame = g_priv->fe_bursts_per_frame;
		frame.h_NumOfFrames        = g_priv->fe_num_frames;
		frame.w_FramePeriodicity   = g_priv->fe_frame_period_ms;

		ret = ml_feChirpFrameCfg(0U, &frame);
		if (ret != M_RL_RET_CODE_OK) {
			LOG_ERR("IWRL6322: feChirpFrameCfg failed: %d", ret);
			return -EIO;
		}
	} else {
		/* MPD profile 0 */
		T_ML_SENS_MPD_PROF_CFG prof = {0};

		prof.c_MpdCfgProfInd   = 0U;
		prof.c_DetectionMode   = g_priv->mpd_detection_mode;
		prof.c_MaxRange        = g_priv->max_range_m;

		uint8_t motion_nibble = (uint8_t)(g_priv->motion_sensitivity & 0x0FU);
		uint8_t detect_nibble = (uint8_t)((g_priv->detection_sensitivity & 0x0FU) << 4U);

		prof.c_MotionAndDetecSensvty = motion_nibble | detect_nibble;

		prof.h_MpdCtrl = (uint16_t)g_priv->mpd_snr_boost;
		if (g_priv->disable_static_obj_removal) {
			prof.h_MpdCtrl |= M_ML_MPD_STATIC_OBJ_REMOVAL_DISABLE;
		}

		prof.h_NumOfFrames      = g_priv->mpd_num_frames;
		prof.w_FramePeriodicity = g_priv->frame_period_ms;
		prof.xc_AdvCfarCfg      = g_priv->mpd_adv_cfar_cfg;
		prof.c_MinPoints 		= g_priv->mpd_min_points;
		prof.xh_MaxDistance		= g_priv->mpd_max_distance_cluster;
		prof.c_SteerVecEnCtrl	= g_priv->mpd_steer_vec_en_ctrl;

		ret = ml_smartfeMpdProfCfg(0U, &prof);
		if (ret != M_RL_RET_CODE_OK) {
			LOG_ERR("IWRL6322: smartfeMpdProfCfg failed: %d", ret);
			return -EIO;
		}

		/* MPD zone config */
		T_ML_SENS_MPD_ZONE_CFG zone_cfg = {0};

		zone_cfg.c_ZoneCoordSelCtrl  = g_priv->coordinate_cartesian
					       ? M_ML_MPD_ZONE_TYPE_CORD_CARTESIAN
					       : M_ML_MPD_ZONE_TYPE_CORD_CYLINDRICAL;
		zone_cfg.c_NumSelectedZones  = g_priv->num_zones;
		zone_cfg.xh_SensPosXRange    = g_priv->sens_pos_x_cm;
		zone_cfg.xh_SensPosYTheta    = g_priv->sens_pos_y_cm;
		zone_cfg.xh_SensPosZ         = g_priv->sens_pos_z_cm;
		zone_cfg.xc_SensTiltAzimuth  = g_priv->sens_tilt_azimuth;
		zone_cfg.xc_SensTiltElevation = g_priv->sens_tilt_elevation;
		zone_cfg.xc_InPlaneTilt      = g_priv->sens_in_plane_tilt;

		/* Build h_ZoneType bitmask from per-zone include/exclude flags */
		zone_cfg.h_ZoneType = 0;
		for (uint8_t zi = 0; zi < g_priv->num_zones; zi++) {
			if (g_priv->zones[zi].include) {
				zone_cfg.h_ZoneType |= (UINT16)(1U << zi);
			}
		}

		/* Copy zone boundaries from iwrl6322_data */
		for (uint8_t zi = 0; zi < g_priv->num_zones; zi++) {
			zone_cfg.z_ZoneCfg[zi].xh_XRMin    = g_priv->zones[zi].xmin_cm;
			zone_cfg.z_ZoneCfg[zi].xh_XRMax    = g_priv->zones[zi].xmax_cm;
			zone_cfg.z_ZoneCfg[zi].xh_YThetaMin = g_priv->zones[zi].ymin_cm;
			zone_cfg.z_ZoneCfg[zi].xh_YThetaMax = g_priv->zones[zi].ymax_cm;
			zone_cfg.z_ZoneCfg[zi].xh_ZMin      = g_priv->zones[zi].zmin_cm;
			zone_cfg.z_ZoneCfg[zi].xh_ZMax      = g_priv->zones[zi].zmax_cm;
		}

		ret = ml_smartfeMpdZoneCfg(0U, &zone_cfg);
		if (ret != M_RL_RET_CODE_OK) {
			LOG_ERR("IWRL6322: smartfeMpdZoneCfg failed: %d", ret);
			return -EIO;
		}

		/* Custom steering vector config — optional, only sent when enabled */
		if (g_priv->steer_vec_custom_ena) {
			T_ML_SENS_STEER_VEC_CFG sv = {0};

			sv.c_CustomSteerVecEnCtrl = M_ML_CUSTOM_STEER_VEC_ENABLE;
			sv.c_NumAzimBins          = g_priv->steer_vec_num_azim_bins;
			sv.c_NumElevBins          = g_priv->steer_vec_num_elev_bins;
			sv.c_AzimStepSize         = g_priv->steer_vec_azim_step;
			sv.c_ElevStepSize         = g_priv->steer_vec_elev_step;
			sv.c_SvElemSize           = g_priv->steer_vec_elem_size;
			sv.xc_AzimStartVal        = 0; /* reserved, must be 0 */
			sv.xc_ElevStartVal        = 0; /* reserved, must be 0 */

			ret = ml_smartfeSteerVecCfg(0U, &sv);
			if (ret != M_RL_RET_CODE_OK) {
				LOG_ERR("IWRL6322: smartfeSteerVecCfg failed: %d", ret);
				return -EIO;
			}
		}
	}

	return 0;
}

/* ============================================================================
 * Sensor Start / Stop
 * ========================================================================== */

static int iwrl6322_start_stop(const struct device *dev, uint8_t start)
{
	T_ML_SENS_START_STOP_CFG cfg = {0};
	ARG_UNUSED(dev);

	cfg.c_SensorStartStopCtrl = start ? M_ML_SENSOR_START : M_ML_SENSOR_STOP;
	cfg.c_RunCalCtrl          = g_priv->run_cal_ctrl;
	cfg.c_MpdMeasProfSel      = g_priv->mpd_meas_prof_sel;
	cfg.c_RunCalTrigInterval  = g_priv->run_cal_trig_interval;
	cfg.c_FlashSensCfgData    = g_priv->flash_sens_cfg_data
				    ? M_ML_DEV_FLASH_PROFILE_ENABLE
				    : M_ML_DEV_FLASH_PROFILE_DISABLE;

	iwrl6322_wakeup(dev, 1);

	k_msleep(5);

	T_ML_STATUS ret = ml_devSensorStartStop(0U, &cfg);

	if (ret != M_RL_RET_CODE_OK) {
		LOG_ERR("IWRL6322: sensorStartStop(%d) failed: %d", start, ret);
		return -EIO;
	}

	/* After start, deassert WAKE_UP so the device may sleep between frames.
	 * It will be reasserted in iwrl6322_fetch_data before each SPI transfer.
	 * Skipped when power saving is disabled — device stays continuously active. */
	// if (IS_ENABLED(CONFIG_IWRL6322_POWER_SAVING) && start) {
		iwrl6322_wakeup(dev, 0);
	// }

	return 0;
}

/* ============================================================================
 * Data Fetch
 * ========================================================================== */

static int iwrl6322_fetch_data(const struct device *dev, void *buf, size_t buf_len)
{
	if (buf == NULL) {
		return -EINVAL;
	}

	/* Assert WAKE_UP before SPI transactions; device sleeps between frames.
	 * Skipped when power saving is disabled — device is already active. */
	if (IS_ENABLED(CONFIG_IWRL6322_POWER_SAVING)) {
		iwrl6322_wakeup(dev, 1);
	}

	/* Query device for the size of available data */
	T_ML_SENS_DATASIZE_GET_CFG sz_cfg = {0};
	T_ML_STATUS ret;
	ret = ml_devSensorGetDataSize(0U, &sz_cfg);
	if (ret != M_RL_RET_CODE_OK) {
		LOG_ERR("IWRL6322: sensorGetDataSize failed: %d", ret);
		if (IS_ENABLED(CONFIG_IWRL6322_POWER_SAVING)) {
			iwrl6322_wakeup(dev, 0);
		}
		return -EIO;
	}

	/* w_DataSizeInWords is in units of 4 bytes */
	uint32_t data_bytes = sz_cfg.w_DataSizeInWords * 4U;

	if (data_bytes == 0U) {
		LOG_WRN("IWRL6322: device reports 0 data bytes available");
		if (IS_ENABLED(CONFIG_IWRL6322_POWER_SAVING)) {
			iwrl6322_wakeup(dev, 0);
		}
		return 0;
	}

	if (data_bytes > buf_len) {
		LOG_ERR("IWRL6322: data size %u exceeds buffer %zu; clamping",
			data_bytes, buf_len);
		data_bytes = (uint32_t)buf_len;
	}

	/* Fetch data directly into the caller-provided buffer */
	T_ML_SENS_DATA_GET_CFG data_cfg = {
		.w_DataSizeInWords = data_bytes / 4U,
		.w_MemDestAddress  = (UINT32)(uintptr_t)buf,
	};

	ret = ml_devSensorGetData(0U, &data_cfg);
	if (IS_ENABLED(CONFIG_IWRL6322_POWER_SAVING)) {
		iwrl6322_wakeup(dev, 0);
	}
	if (ret != M_RL_RET_CODE_OK) {
		LOG_ERR("IWRL6322: sensorGetData failed: %d", ret);
		return -EIO;
	}

	return (int)data_bytes;
}

/* ============================================================================
 * Attribute Set / Get
 * ========================================================================== */

/*
 * Helper: stop sensor, reconfigure with current iwrl6322_data fields, restart.
 * Used when a config attr is changed while the sensor is running.
 */
static int iwrl6322_reconfigure_running(const struct device *dev)
{
	int r;

	r = iwrl6322_start_stop(dev, 0);
	if (r != 0) {
		return r;
	}
	g_priv->sensor_running = false;

	r = iwrl6322_configure(dev);
	if (r != 0) {
		return r;
	}

	r = iwrl6322_start_stop(dev, 1);
	if (r != 0) {
		return r;
	}
	g_priv->sensor_running = true;
	return 0;
}

static int iwrl6322_attr_set_helper(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	ARG_UNUSED(chan);

	switch ((int)attr) {

	case IWRL6322_ATTR_OPERATING_MODE: {
		int requested_mode = val->val1;

		if (requested_mode == IWRL6322_MODE_POWER_OFF) {
			if (g_priv->sensor_running) {
				iwrl6322_start_stop(dev, 0);
				g_priv->sensor_running = false;
			}
			const T_ML_DEV_POWER_UP_CFG pwr_down = {
				.c_PowerControl = 0U,
				.c_SopBootCfg   = M_ML_DEVICE_SOP_BOOT_SPI_MODE,
			};
			T_ML_STATUS ml_ret = ml_devPwrCtrl(0U, &pwr_down);
			if (ml_ret != M_RL_RET_CODE_OK) {
				LOG_ERR("IWRL6322: devPwrCtrl(down) failed: %d", ml_ret);
			}
			ml_ret = ml_mmWaveLinkDeinit(1U);
			if (ml_ret != M_RL_RET_CODE_OK) {
				LOG_WRN("IWRL6322: mmWaveLinkDeinit failed: %d", ml_ret);
			}
			// g_priv->lib_initialized = false;
			g_priv->firmware_loaded = false;
			return 0;
		}

		if (requested_mode == IWRL6322_MODE_STOP) {
			if (g_priv->sensor_running) {
				int r = iwrl6322_start_stop(dev, 0);

				if (r != 0) {
					return r;
				}
				g_priv->sensor_running = false;
			}
			return 0;
		}

		if (requested_mode < IWRL6322_MODE_RAW_ADC ||
		    requested_mode > IWRL6322_MODE_MPD_VERBOSE) {
			return -EINVAL;
		}

		if (!g_priv->firmware_loaded) {
			// if (!g_priv->lib_initialized) {
			// 	T_ML_LINK_INIT_CFG link_cfg;

			// 	iwrl6322_build_init_cfg(&link_cfg, g_priv->api_timeout_ms);
			// 	T_ML_STATUS ml_ret = ml_mmWaveLinkInit(1U, &link_cfg);

			// 	if (ml_ret != M_RL_RET_CODE_OK) {
			// 		LOG_ERR("IWRL6322: mmWaveLinkInit (re-init) failed: %d",
			// 			ml_ret);
			// 		return -EIO;
			// 	}
			// 	g_priv->lib_initialized = true;
			// }

			const T_ML_DEV_POWER_UP_CFG pwr_down = {
				.c_PowerControl = 0U,
				.c_SopBootCfg   = M_ML_DEVICE_SOP_BOOT_SPI_MODE,
			};
			T_ML_STATUS ml_ret = ml_devPwrCtrl(0U, &pwr_down);

			if (ml_ret != M_RL_RET_CODE_OK) {
				LOG_ERR("IWRL6322: devPwrCtrl(down) failed: %d", ml_ret);
				return -EIO;
			}

			const T_ML_DEV_POWER_UP_CFG pwr_up = {
				.c_PowerControl = 1U,
				.c_SopBootCfg   = M_ML_DEVICE_SOP_BOOT_SPI_MODE,
			};

			ml_ret = ml_devPwrCtrl(0U, &pwr_up);
			if (ml_ret != M_RL_RET_CODE_OK) {
				LOG_ERR("IWRL6322: devPwrCtrl(up) failed: %d", ml_ret);
				return -EIO;
			}

			ml_ret = ml_protocolConfig(0U);
			if (ml_ret != M_RL_RET_CODE_OK) {
				LOG_ERR("IWRL6322: protocolConfig failed: %d", ml_ret);
				return -EIO;
			}

			const T_ML_DEV_CTRL_CFG dev_ctrl = {
				.c_DeviceCtrl = M_ML_DEVICE_CTRL_ACTIVE,
			};

			ml_ret = ml_devCtrl(0U, &dev_ctrl);
			if (ml_ret != M_RL_RET_CODE_OK) {
				LOG_ERR("IWRL6322: devCtrl(active) failed: %d", ml_ret);
				return -EIO;
			}

			int r = iwrl6322_load_firmware(dev);

			if (r != 0) {
				return r;
			}

			k_msleep(500);

			T_ML_DEVICE_APP_STATUS dev_status = {0};
			T_ML_STATUS sret = ml_devAppStatus(0U, &dev_status);

			if (sret == M_RL_RET_CODE_OK) {
				LOG_INF("IWRL6322 devAppStatus:"
					" swState=0x%02X resetCause=0x%02X"
					" bootClkSop=0x%02X pwrState=0x%02X"
					" flash=0x%02X devVar=0x%02X",
					dev_status.c_SwState,
					dev_status.c_ResetCause,
					dev_status.c_BootClkSrcSopMode,
					dev_status.c_AppPwrState,
					dev_status.c_FlashStatus,
					dev_status.c_DevVarTypes);
				LOG_INF("IWRL6322 devAppStatus:"
					" factCal=0x%04X runCal=0x%04X"
					" dpu=0x%04X fec=0x%04X",
					dev_status.h_FefactCalStatus,
					dev_status.h_FeRunCalStatus,
					dev_status.h_DpuStatus,
					dev_status.h_FecStatus);
			} else {
				LOG_WRN("IWRL6322: devAppStatus failed: %d", sret);
			}
		}

		if (g_priv->sensor_running) {
			int r = iwrl6322_start_stop(dev, 0);

			if (r != 0) {
				return r;
			}
			g_priv->sensor_running = false;
		}

		iwrl6322_wakeup(dev, 1);

		iwrl6322_set_mode_bitfields(g_priv, requested_mode);

		int r = iwrl6322_configure(dev);

		if (r != 0) {
			return r;
		}

		r = iwrl6322_start_stop(dev, 1);
		if (r != 0) {
			return r;
		}

		g_priv->sensor_running = true;

		LOG_INF("IWRL6322: mode set to %d", requested_mode);
		return 0;
	}

	case SENSOR_ATTR_FULL_SCALE:
		if (val->val1 < 0 || val->val1 > 2) {
			return -EINVAL;
		}
		g_priv->max_range_m = (uint8_t)val->val1;
		break;

	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (val->val1 <= 0) {
			return -EINVAL;
		}
		g_priv->frame_period_ms = 1000000U / (uint32_t)val->val1;
		break;

	case IWRL6322_ATTR_REGION:
		if (val->val1 != IWRL6322_REGION_FCC_CE && val->val1 != IWRL6322_REGION_TELEC) {
			return -EINVAL;
		}
		g_priv->region = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_MOTION_SENSITIVITY:
		if (val->val1 < 0 || val->val1 > 3) {
			return -EINVAL;
		}
		g_priv->motion_sensitivity = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_DETECTION_SENSITIVITY:
		if (val->val1 < 0 || val->val1 > 5) {
			return -EINVAL;
		}
		g_priv->detection_sensitivity = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_ZONE_COUNT:
		if (val->val1 < 1 || val->val1 > 16) {
			return -EINVAL;
		}
		g_priv->num_zones = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_ZONE_COORDINATE_CARTESIAN:
		g_priv->coordinate_cartesian = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_ZONE_XMIN:
	case IWRL6322_ATTR_ZONE_XMAX:
	case IWRL6322_ATTR_ZONE_YMIN:
	case IWRL6322_ATTR_ZONE_YMAX:
	case IWRL6322_ATTR_ZONE_ZMIN:
	case IWRL6322_ATTR_ZONE_ZMAX: {
		int zi = val->val1;

		if (zi < 0 || zi >= 16) {
			return -EINVAL;
		}
		switch ((int)attr) {
		case IWRL6322_ATTR_ZONE_XMIN: g_priv->zones[zi].xmin_cm = (int16_t)val->val2;  break;
		case IWRL6322_ATTR_ZONE_XMAX: g_priv->zones[zi].xmax_cm = (int16_t)val->val2;  break;
		case IWRL6322_ATTR_ZONE_YMIN: g_priv->zones[zi].ymin_cm = (uint16_t)val->val2; break;
		case IWRL6322_ATTR_ZONE_YMAX: g_priv->zones[zi].ymax_cm = (uint16_t)val->val2; break;
		case IWRL6322_ATTR_ZONE_ZMIN: g_priv->zones[zi].zmin_cm = (int16_t)val->val2; break;
		case IWRL6322_ATTR_ZONE_ZMAX: g_priv->zones[zi].zmax_cm = (int16_t)val->val2; break;
		default: break;
		}
		break;
	}

	case IWRL6322_ATTR_ZONE_TYPE: {
		int zi = val->val1;

		if (zi < 0 || zi >= 16) {
			return -EINVAL;
		}
		g_priv->zones[zi].include = (val->val2 != 0);
		break;
	}

	case IWRL6322_ATTR_DISABLE_STATIC_OBJ_REMOVAL:
		g_priv->disable_static_obj_removal = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_RUN_CAL_CTRL:
		g_priv->run_cal_ctrl = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_MPD_PROF_SEL:
		if (val->val1 != (int)M_ML_SENS_MPD_MEAS_AUTO &&
		    (val->val1 < 0 || val->val1 > 3)) {
			return -EINVAL;
		}
		g_priv->mpd_meas_prof_sel = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_API_TIMEOUT_MS:
		if (val->val1 < 1 || val->val1 > 65535) {
			return -EINVAL;
		}
		g_priv->api_timeout_ms = (uint16_t)val->val1;
		return 0;

	case IWRL6322_ATTR_ANTENNA_TYPE:
		if (val->val1 < 0 || val->val1 > 3) {
			return -EINVAL;
		}
		g_priv->dev_antenna_sel = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_MPD_HOLDTIME_10US:
		if (val->val1 < 1) {
			return -EINVAL;
		}
		g_priv->mpd_holdtime_10us = (uint32_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_CALIB_CTRL:
		if (val->val1 < 0 || val->val1 > 0x1FF) {
			return -EINVAL;
		}
		g_priv->fe_calib_ctrl = (uint16_t)val->val1;
		break;

	case IWRL6322_ATTR_MPD_DETECTION_MODE:
		if (val->val1 < 1 || val->val1 > 4) {
			return -EINVAL;
		}
		g_priv->mpd_detection_mode = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_MPD_SNR_BOOST:
		if (val->val1 < 0 || val->val1 > 2) {
			return -EINVAL;
		}
		g_priv->mpd_snr_boost = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_MPD_NUM_FRAMES:
		if (val->val1 < 0 || val->val1 > 65535) {
			return -EINVAL;
		}
		g_priv->mpd_num_frames = (uint16_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_ADC_SAMP_RATE:
		if (val->val1 < 8 || val->val1 > 100) {
			return -EINVAL;
		}
		g_priv->fe_adc_samp_rate = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_NUM_VIRTUAL_ANT:
		if (val->val1 < 1 || val->val1 > 4) {
			return -EINVAL;
		}
		g_priv->fe_num_virtual_ant = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_TXRX_CH_ENA:
		if (val->val1 < 0 || val->val1 > 0xFFFF) {
			return -EINVAL;
		}
		g_priv->fe_txrx_ch_ena = (uint16_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_NUM_ADC_SAMPLES:
		if (val->val1 != 32 && val->val1 != 64 &&
		    val->val1 != 128 && val->val1 != 256) {
			return -EINVAL;
		}
		g_priv->fe_num_adc_samples = (uint16_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_RAMP_END_TIME:
		if (val->val1 < 60 || val->val1 > 2560) {
			return -EINVAL;
		}
		g_priv->fe_ramp_end_time = (uint16_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_IDLE_TIME:
		if (val->val1 < 31 || val->val1 > 10000) {
			return -EINVAL;
		}
		g_priv->fe_idle_time = (uint16_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_ADC_SKIP_SAMP:
		if (val->val1 < 0 || val->val1 > 63) {
			return -EINVAL;
		}
		g_priv->fe_adc_skip_samp = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_RF_FREQ_START:
		if (val->val1 < 0xBE00 || val->val1 > 0xD550) {
			return -EINVAL;
		}
		g_priv->fe_rf_freq_start = (uint32_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_RF_FREQ_SLOPE:
		if (val->val1 < -13981 || val->val1 > 13981) {
			return -EINVAL;
		}
		g_priv->fe_rf_freq_slope = (int16_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_TX_START_TIME:
		if (val->val1 < -300 || val->val1 > 300) {
			return -EINVAL;
		}
		g_priv->fe_tx_start_time = (int16_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_HPF_CORNER:
		if (val->val1 < 0 || val->val1 > 3) {
			return -EINVAL;
		}
		g_priv->fe_hpf_corner = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_CAL_RX_GAIN:
		if (val->val1 < 34 || val->val1 > 48 || (val->val1 & 1)) {
			return -EINVAL;
		}
		g_priv->fe_cal_rx_gain = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_CAL_TX_BACKOFF:
		if (val->val1 < 0 || val->val1 > 15) {
			return -EINVAL;
		}
		g_priv->fe_cal_tx_backoff = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_INTER_CHIRP_PS_OVRD:
		if (val->val1 < 0 || val->val1 > 127) {
			return -EINVAL;
		}
		g_priv->fe_inter_chirp_ps_ovrd = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_CHIRPS_ACCUM:
		if (val->val1 == 1 || val->val1 < 0 || val->val1 > 64) {
			return -EINVAL;
		}
		g_priv->fe_chirps_accum = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_CHIRPS_PER_BURST:
		if (val->val1 < 1 || val->val1 > 1023) {
			return -EINVAL;
		}
		g_priv->fe_chirps_per_burst = (uint16_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_BURST_IDLE_TIME:
		if (val->val1 < 1500 || val->val1 > 150000) {
			return -EINVAL;
		}
		g_priv->fe_burst_idle_time = (uint32_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_BURSTS_PER_FRAME:
		if (val->val1 < 1 || val->val1 > 127) {
			return -EINVAL;
		}
		g_priv->fe_bursts_per_frame = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_NUM_FRAMES:
		if (val->val1 < 0 || val->val1 > 65535) {
			return -EINVAL;
		}
		g_priv->fe_num_frames = (uint16_t)val->val1;
		break;

	case IWRL6322_ATTR_FE_FRAME_PERIOD_MS:
		if (val->val1 < 10 || val->val1 > 84375000) {
			return -EINVAL;
		}
		g_priv->fe_frame_period_ms = (uint32_t)val->val1;
		break;

	case IWRL6322_ATTR_MPD_SIG_TRIG_TYPE:
		if (val->val1 < 0 || val->val1 > 1) {
			return -EINVAL;
		}
		g_priv->mpd_sig_trig_type = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_MPD_AUTO_BOOT_MODE:
		if (val->val1 < 0 || val->val1 > 1) {
			return -EINVAL;
		}
		g_priv->mpd_auto_boot_mode = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_MCU_CLK_OUT_SEL:
		if (val->val1 < 0 || val->val1 > 3) {
			return -EINVAL;
		}
		g_priv->mcu_clk_out_sel = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_DEEP_SLEEP_ENTRY_DIS:
		if (val->val1 < 0 || val->val1 > 1) {
			return -EINVAL;
		}
		g_priv->deep_sleep_entry_dis = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_TRIG_BOOT_CAL:
		if (val->val1 < 0 || val->val1 > 1) {
			return -EINVAL;
		}
		g_priv->trig_boot_cal = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_HIL_ENABLE:
		g_priv->hil_enable = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_JTAG_ENABLE:
		g_priv->jtag_enable = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_XTAL_LESS_MODE:
		if (val->val1 < 0 || val->val1 > 1) {
			return -EINVAL;
		}
		g_priv->xtal_less_mode = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_GPIO_MARKER_ENA:
		g_priv->gpio_marker_ena = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_CZ_CW_MODE_ENA:
		g_priv->cz_cw_mode_ena = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_HOLD_LPDS:
		g_priv->hold_lpds = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_MOUNT_TYPE:
		g_priv->mount_type = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_RUN_CAL_TRIG_INTERVAL:
		if (val->val1 < 0 || val->val1 > 255) {
			return -EINVAL;
		}
		g_priv->run_cal_trig_interval = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_FLASH_SENS_CFG:
		if (val->val1 < 0 || val->val1 > 1) {
			return -EINVAL;
		}
		g_priv->flash_sens_cfg_data = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_MPD_ADV_CFAR_CFG:
		if (val->val1 < -6 || val->val1 > 6) {
			return -EINVAL;
		}
		g_priv->mpd_adv_cfar_cfg = (int8_t)val->val1;
		break;

	case IWRL6322_ATTR_SENS_POS_X_CM:
		if (val->val1 < -1500 || val->val1 > 1500) {
			return -EINVAL;
		}
		g_priv->sens_pos_x_cm = (int16_t)val->val1;
		break;

	case IWRL6322_ATTR_SENS_POS_Y_CM:
		if (val->val1 < -1500 || val->val1 > 1500) {
			return -EINVAL;
		}
		g_priv->sens_pos_y_cm = (int16_t)val->val1;
		break;

	case IWRL6322_ATTR_SENS_POS_Z_CM:
		if (val->val1 < -1500 || val->val1 > 1500) {
			return -EINVAL;
		}
		g_priv->sens_pos_z_cm = (int16_t)val->val1;
		break;

	case IWRL6322_ATTR_SENS_TILT_AZIMUTH:
		if (val->val1 < -90 || val->val1 > 90) {
			return -EINVAL;
		}
		g_priv->sens_tilt_azimuth = (int8_t)val->val1;
		break;

	case IWRL6322_ATTR_SENS_TILT_ELEVATION:
		if (val->val1 < -90 || val->val1 > 90) {
			return -EINVAL;
		}
		g_priv->sens_tilt_elevation = (int8_t)val->val1;
		break;

	case IWRL6322_ATTR_SENS_IN_PLANE_TILT:
		if (val->val1 != 0 && val->val1 != 90 && val->val1 != -90) {
			return -EINVAL;
		}
		g_priv->sens_in_plane_tilt = (int8_t)val->val1;
		break;

	// case IWRL6322_ATTR_GPADC_PARAM:
	// 	g_priv->gpadc_param_val = (uint32_t)val->val1;
	// 	return 0;

	// case IWRL6322_ATTR_GPADC_CFG:
	// 	g_priv->gpadc_cfg_val = (uint32_t)val->val1;
	// 	return 0;

	case IWRL6322_ATTR_STEER_VEC_CUSTOM_ENA:
		g_priv->steer_vec_custom_ena = (val->val1 != 0);
		break;

	case IWRL6322_ATTR_STEER_VEC_NUM_AZIM_BINS:
		if (val->val1 < 1 || val->val1 > 255) {
			return -EINVAL;
		}
		g_priv->steer_vec_num_azim_bins = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_STEER_VEC_NUM_ELEV_BINS:
		if (val->val1 < 1 || val->val1 > 255) {
			return -EINVAL;
		}
		g_priv->steer_vec_num_elev_bins = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_STEER_VEC_AZIM_STEP:
		if (val->val1 < 0 || val->val1 > 255) {
			return -EINVAL;
		}
		g_priv->steer_vec_azim_step = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_STEER_VEC_ELEV_STEP:
		if (val->val1 < 0 || val->val1 > 255) {
			return -EINVAL;
		}
		g_priv->steer_vec_elev_step = (uint8_t)val->val1;
		break;

	case IWRL6322_ATTR_STEER_VEC_ELEM_SIZE:
		if (val->val1 != 0 && val->val1 != 1) {
			return -EINVAL;
		}
		g_priv->steer_vec_elem_size = (uint8_t)val->val1;
		break;

	default:
		return -ENOTSUP;
	}

	if (g_priv->sensor_running) {
		return iwrl6322_reconfigure_running(dev);
	}
	return 0;
}

static int iwrl6322_attr_get(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, struct sensor_value *val)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);

	switch ((int)attr) {
	case IWRL6322_ATTR_OPERATING_MODE:
		val->val1 = (int32_t)iwrl6322_get_mode(g_priv);
		val->val2 = 0;
		return 0;

	case SENSOR_ATTR_FULL_SCALE:
		val->val1 = (int32_t)g_priv->max_range_m;
		val->val2 = 0;
		return 0;

	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		val->val1 = (g_priv->frame_period_ms > 0)
			    ? (int32_t)(1000000U / g_priv->frame_period_ms)
			    : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_REGION:
		val->val1 = (int32_t)g_priv->region;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MOTION_SENSITIVITY:
		val->val1 = (int32_t)g_priv->motion_sensitivity;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_DETECTION_SENSITIVITY:
		val->val1 = (int32_t)g_priv->detection_sensitivity;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_ZONE_COUNT:
		val->val1 = (int32_t)g_priv->num_zones;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_ZONE_COORDINATE_CARTESIAN:
		val->val1 = g_priv->coordinate_cartesian ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_ZONE_XMIN:
	case IWRL6322_ATTR_ZONE_XMAX:
	case IWRL6322_ATTR_ZONE_YMIN:
	case IWRL6322_ATTR_ZONE_YMAX:
	case IWRL6322_ATTR_ZONE_ZMIN:
	case IWRL6322_ATTR_ZONE_ZMAX: {
		int zi = val->val1;

		if (zi < 0 || zi >= 16) {
			return -EINVAL;
		}
		switch ((int)attr) {
		case IWRL6322_ATTR_ZONE_XMIN: val->val2 = (int32_t)g_priv->zones[zi].xmin_cm; break;
		case IWRL6322_ATTR_ZONE_XMAX: val->val2 = (int32_t)g_priv->zones[zi].xmax_cm; break;
		case IWRL6322_ATTR_ZONE_YMIN: val->val2 = (int32_t)g_priv->zones[zi].ymin_cm; break;
		case IWRL6322_ATTR_ZONE_YMAX: val->val2 = (int32_t)g_priv->zones[zi].ymax_cm; break;
		case IWRL6322_ATTR_ZONE_ZMIN: val->val2 = (int32_t)g_priv->zones[zi].zmin_cm; break;
		case IWRL6322_ATTR_ZONE_ZMAX: val->val2 = (int32_t)g_priv->zones[zi].zmax_cm; break;
		default: break;
		}
		return 0;
	}

	case IWRL6322_ATTR_ZONE_TYPE: {
		int zi = val->val1;

		if (zi < 0 || zi >= 16) {
			return -EINVAL;
		}
		val->val2 = g_priv->zones[zi].include ? 1 : 0;
		return 0;
	}

	case IWRL6322_ATTR_DISABLE_STATIC_OBJ_REMOVAL:
		val->val1 = g_priv->disable_static_obj_removal ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_RUN_CAL_CTRL:
		val->val1 = g_priv->run_cal_ctrl;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MPD_PROF_SEL:
		val->val1 = (int32_t)g_priv->mpd_meas_prof_sel;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_ANTENNA_TYPE:
		val->val1 = (int32_t)g_priv->dev_antenna_sel;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MPD_HOLDTIME_10US:
		val->val1 = (int32_t)g_priv->mpd_holdtime_10us;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_CALIB_CTRL:
		val->val1 = (int32_t)g_priv->fe_calib_ctrl;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MPD_DETECTION_MODE:
		val->val1 = (int32_t)g_priv->mpd_detection_mode;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MPD_SNR_BOOST:
		val->val1 = (int32_t)g_priv->mpd_snr_boost;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MPD_NUM_FRAMES:
		val->val1 = (int32_t)g_priv->mpd_num_frames;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_ADC_SAMP_RATE:
		val->val1 = (int32_t)g_priv->fe_adc_samp_rate;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_NUM_VIRTUAL_ANT:
		val->val1 = (int32_t)g_priv->fe_num_virtual_ant;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_TXRX_CH_ENA:
		val->val1 = (int32_t)g_priv->fe_txrx_ch_ena;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_NUM_ADC_SAMPLES:
		val->val1 = (int32_t)g_priv->fe_num_adc_samples;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_RAMP_END_TIME:
		val->val1 = (int32_t)g_priv->fe_ramp_end_time;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_IDLE_TIME:
		val->val1 = (int32_t)g_priv->fe_idle_time;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_ADC_SKIP_SAMP:
		val->val1 = (int32_t)g_priv->fe_adc_skip_samp;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_RF_FREQ_START:
		val->val1 = (int32_t)g_priv->fe_rf_freq_start;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_RF_FREQ_SLOPE:
		val->val1 = (int32_t)g_priv->fe_rf_freq_slope;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_TX_START_TIME:
		val->val1 = (int32_t)g_priv->fe_tx_start_time;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_HPF_CORNER:
		val->val1 = (int32_t)g_priv->fe_hpf_corner;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_CAL_RX_GAIN:
		val->val1 = (int32_t)g_priv->fe_cal_rx_gain;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_CAL_TX_BACKOFF:
		val->val1 = (int32_t)g_priv->fe_cal_tx_backoff;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_INTER_CHIRP_PS_OVRD:
		val->val1 = (int32_t)g_priv->fe_inter_chirp_ps_ovrd;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_CHIRPS_ACCUM:
		val->val1 = (int32_t)g_priv->fe_chirps_accum;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_CHIRPS_PER_BURST:
		val->val1 = (int32_t)g_priv->fe_chirps_per_burst;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_BURST_IDLE_TIME:
		val->val1 = (int32_t)g_priv->fe_burst_idle_time;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_BURSTS_PER_FRAME:
		val->val1 = (int32_t)g_priv->fe_bursts_per_frame;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_NUM_FRAMES:
		val->val1 = (int32_t)g_priv->fe_num_frames;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FE_FRAME_PERIOD_MS:
		val->val1 = (int32_t)g_priv->fe_frame_period_ms;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MPD_SIG_TRIG_TYPE:
		val->val1 = g_priv->mpd_sig_trig_type ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MPD_AUTO_BOOT_MODE:
		val->val1 = g_priv->mpd_auto_boot_mode ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MCU_CLK_OUT_SEL:
		val->val1 = (int32_t)g_priv->mcu_clk_out_sel;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_DEEP_SLEEP_ENTRY_DIS:
		val->val1 = g_priv->deep_sleep_entry_dis ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_TRIG_BOOT_CAL:
		val->val1 = g_priv->trig_boot_cal ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_HIL_ENABLE:
		val->val1 = g_priv->hil_enable ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_JTAG_ENABLE:
		val->val1 = g_priv->jtag_enable ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_XTAL_LESS_MODE:
		val->val1 = g_priv->xtal_less_mode ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_GPIO_MARKER_ENA:
		val->val1 = g_priv->gpio_marker_ena ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_CZ_CW_MODE_ENA:
		val->val1 = g_priv->cz_cw_mode_ena ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_HOLD_LPDS:
		val->val1 = g_priv->hold_lpds ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MOUNT_TYPE:
		val->val1 = g_priv->mount_type ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_RUN_CAL_TRIG_INTERVAL:
		val->val1 = (int32_t)g_priv->run_cal_trig_interval;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_FLASH_SENS_CFG:
		val->val1 = g_priv->flash_sens_cfg_data ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_MPD_ADV_CFAR_CFG:
		val->val1 = (int32_t)g_priv->mpd_adv_cfar_cfg;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_SENS_POS_X_CM:
		val->val1 = (int32_t)g_priv->sens_pos_x_cm;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_SENS_POS_Y_CM:
		val->val1 = (int32_t)g_priv->sens_pos_y_cm;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_SENS_POS_Z_CM:
		val->val1 = (int32_t)g_priv->sens_pos_z_cm;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_SENS_TILT_AZIMUTH:
		val->val1 = (int32_t)g_priv->sens_tilt_azimuth;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_SENS_TILT_ELEVATION:
		val->val1 = (int32_t)g_priv->sens_tilt_elevation;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_SENS_IN_PLANE_TILT:
		val->val1 = (int32_t)g_priv->sens_in_plane_tilt;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_API_TIMEOUT_MS:
		val->val1 = (int32_t)g_priv->api_timeout_ms;
		val->val2 = 0;
		return 0;

	// case IWRL6322_ATTR_GPADC_PARAM:
	// 	val->val1 = (int32_t)g_priv->gpadc_param_val;
	// 	val->val2 = 0;
	// 	return 0;

	// case IWRL6322_ATTR_GPADC_CFG:
	// 	val->val1 = (int32_t)g_priv->gpadc_cfg_val;
	// 	val->val2 = 0;
	// 	return 0;

	case IWRL6322_ATTR_STEER_VEC_CUSTOM_ENA:
		val->val1 = g_priv->steer_vec_custom_ena ? 1 : 0;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_STEER_VEC_NUM_AZIM_BINS:
		val->val1 = (int32_t)g_priv->steer_vec_num_azim_bins;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_STEER_VEC_NUM_ELEV_BINS:
		val->val1 = (int32_t)g_priv->steer_vec_num_elev_bins;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_STEER_VEC_AZIM_STEP:
		val->val1 = (int32_t)g_priv->steer_vec_azim_step;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_STEER_VEC_ELEV_STEP:
		val->val1 = (int32_t)g_priv->steer_vec_elev_step;
		val->val2 = 0;
		return 0;

	case IWRL6322_ATTR_STEER_VEC_ELEM_SIZE:
		val->val1 = (int32_t)g_priv->steer_vec_elem_size;
		val->val2 = 0;
		return 0;

	default:
		return -ENOTSUP;
	}
}

/*
 * Assert (level=1) or deassert (level=0) WAKE_UP, then also update the TI
 * library's device-control state via ml_devCtrl.
 *
 * Must NOT be called from within a TI library callback (iwrl6322_device_pin_ctrl
 * drives the GPIO directly to avoid re-entering ml_devCtrl).
 */

void iwrl6322_wakeup(const struct device *dev, int level)
{
	const struct iwrl6322_config *cfg = dev->config;

	if (cfg->wake_up_gpio.port != NULL) {
		gpio_pin_set_dt(&cfg->wake_up_gpio, level);
	}

	const T_ML_DEV_CTRL_CFG dev_ctrl = {
		.c_DeviceCtrl = (level != 0) ? M_ML_DEVICE_CTRL_ACTIVE
					     : M_ML_DEVICE_CTRL_LOW,
	};
	T_ML_STATUS ml_ret = ml_devCtrl(0U, &dev_ctrl);

	if (ml_ret != M_RL_RET_CODE_OK) {
		LOG_ERR("ml_devCtrl(wakeup level=%d) failed: %d", level, ml_ret);
	}
}

void iwrl6322_nreset(const struct device *dev, int level)
{
	const struct iwrl6322_config *cfg = dev->config;

	if (cfg->nreset_gpio.port != NULL) {
		gpio_pin_set_dt(&cfg->nreset_gpio, level);
	}
}

/* ============================================================================
 * PRESENCE_DETECT GPIO Interrupt → Zephyr Trigger
 * ========================================================================== */

static void iwrl6322_trig_work_handler(struct k_work *work)
{
	struct iwrl6322_data *data = CONTAINER_OF(work, struct iwrl6322_data, trig_work);

	/*
	 * Streaming path — fetch directly into the RTIO buffer with header.
	 * Buffer layout: [struct iwrl6322_buffer_header]
	 */
	if (data->streaming_sqe != NULL) {
		struct rtio_iodev_sqe *iodev_sqe = data->streaming_sqe;
		const uint32_t payload_size = CONFIG_IWRL6322_MAX_PAYLOAD_BYTES;
		const uint32_t min_buf_len = sizeof(struct iwrl6322_buffer_header) + payload_size;
		uint8_t *buf;
		uint32_t buf_len;
		int ret;
		int rc;

		rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len,
				     &buf, &buf_len);
		if (rc != 0) {
			if (rc == -ENOMEM) {
				/*
				 * In multishot mode we may have a single large block only.
				 * If the application has not released the previous frame yet,
				 * drop this pulse and keep the stream armed instead of
				 * permanently terminating on transient backpressure.
				 */
				LOG_WRN("stream: dropping frame, RTIO buffer still busy");
				return;
			}

			LOG_ERR("stream: RTIO buf alloc failed: %d", rc);
			data->streaming_sqe = NULL;
			rtio_iodev_sqe_err(iodev_sqe, rc);
			return;
		}

		/* Write buffer header with metadata. */
		struct iwrl6322_buffer_header *hdr = (struct iwrl6322_buffer_header *)buf;

		hdr->magic = IWRL6322_BUFFER_MAGIC;
		hdr->operating_mode = iwrl6322_get_mode(data);
		hdr->reserved1 = 0;
		hdr->payload_size = payload_size;
		hdr->timestamp_us = k_uptime_get_32();
		hdr->reserved2 = 0;

		/* Fetch payload directly after the header. */
		ret = iwrl6322_fetch_data(g_dev,
					  buf + sizeof(struct iwrl6322_buffer_header),
					  buf_len - sizeof(struct iwrl6322_buffer_header));
		if (ret < 0) {
			LOG_ERR("stream: fetch_data failed: %d", ret);
			data->streaming_sqe = NULL;
			rtio_iodev_sqe_err(iodev_sqe, ret);
			return;
		}
		hdr->payload_size = (uint32_t)ret;

		/*
		 * Complete the SQE.  For a multishot SQE (sensor_stream),
		 * RTIO re-calls iodev_submit automatically — streaming_sqe
		 * will be refreshed there.
		 */
		rtio_iodev_sqe_ok(iodev_sqe, 0);
		return;
	}

	if (data->data_ready_handler != NULL) {
		data->data_ready_handler(g_dev, data->data_ready_trigger);
	}
}

static void iwrl6322_pres_det_gpio_cb(const struct device *port,
				    struct gpio_callback *cb, uint32_t pins)
{
	struct iwrl6322_data *data =
		CONTAINER_OF(cb, struct iwrl6322_data, pres_det_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_work_submit_to_queue(&iwrl6322_work_q, &data->trig_work);
}

/* ============================================================================
 * Zephyr Sensor Driver API
 * ========================================================================== */

static int iwrl6322_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	return -ENOTSUP;
}

static int iwrl6322_reconfigure_gpio_int(const struct device *dev)
{
	const struct iwrl6322_data *data = dev->data;
	const struct iwrl6322_config *cfg = dev->config;
	bool need_int = (data->data_ready_handler != NULL);

	need_int = need_int || (data->streaming_sqe != NULL);

	if (!need_int) {
		return gpio_pin_interrupt_configure_dt(&cfg->pres_det_gpio,
						       GPIO_INT_DISABLE);
	}
	return gpio_pin_interrupt_configure_dt(&cfg->pres_det_gpio,
					       GPIO_INT_EDGE_RISING);
}

static int iwrl6322_trigger_set(const struct device *dev,
			      const struct sensor_trigger *trig,
			      sensor_trigger_handler_t handler)
{
	struct iwrl6322_data *data = dev->data;
	const struct iwrl6322_config *cfg = dev->config;

	if (cfg->pres_det_gpio.port == NULL) {
		return -ENOTSUP;
	}

	if ((int)trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	data->data_ready_trigger = trig;
	data->data_ready_handler = handler;

	return iwrl6322_reconfigure_gpio_int(dev);
}

static int iwrl6322_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{

	int ret = iwrl6322_attr_set_helper(dev, chan, attr, val);

	if (ret == 0 &&
	    attr == (enum sensor_attribute)IWRL6322_ATTR_OPERATING_MODE &&
	    val->val1 == (int32_t)IWRL6322_MODE_STOP) {
		struct iwrl6322_data *data = dev->data;

		if (data->streaming_sqe != NULL) {
			struct rtio_iodev_sqe *sqe = data->streaming_sqe;

			data->streaming_sqe = NULL;
			rtio_iodev_sqe_err(sqe, -ECANCELED);
		}
	}

	return ret;
}

/*
 * RTIO async submit handler.
 *
 * Allocates a buffer from the RTIO memory pool, writes a metadata header,
 * then fetches data directly into the buffer after the header.
 * The header makes the buffer self-describing so decoders can correctly interpret
 * the data regardless of operating mode.
 *
 * Buffer layout: [struct iwrl6322_buffer_header]
 */
static void iwrl6322_submit_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	const uint32_t payload_size = CONFIG_IWRL6322_MAX_PAYLOAD_BYTES;
	const uint32_t min_buf_len = sizeof(struct iwrl6322_buffer_header) + payload_size;
	uint8_t *buf;
	uint32_t buf_len;
	int rc;
	int ret;

	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = cfg->sensor;
	struct iwrl6322_data *data = dev->data;

	rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (rc != 0) {
		LOG_ERR("RTIO rx buffer alloc failed: %d", rc);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	/* Write buffer header with metadata for self-describing buffer. */
	struct iwrl6322_buffer_header *hdr = (struct iwrl6322_buffer_header *)buf;

	hdr->magic = IWRL6322_BUFFER_MAGIC;
	hdr->operating_mode = iwrl6322_get_mode(data);
	hdr->reserved1 = 0;
	hdr->payload_size = payload_size;
	hdr->timestamp_us = k_uptime_get_32();
	hdr->reserved2 = 0;

	ret = iwrl6322_fetch_data(dev,
				  buf + sizeof(struct iwrl6322_buffer_header),
				  buf_len - sizeof(struct iwrl6322_buffer_header));
	if (ret < 0) {
		LOG_ERR("submit_sync: fetch_data failed: %d", ret);
		rtio_iodev_sqe_err(iodev_sqe, ret);
		return;
	}
	hdr->payload_size = (uint32_t)ret;

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

static void iwrl6322_iodev_submit(const struct device *dev,
				struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg =
		(const struct sensor_read_config *)iodev_sqe->sqe.iodev->data;

	if (cfg->is_streaming) {
		/*
		 * Streaming path (sensor_stream / SENSOR_DT_STREAM_IODEV).
		 *
		 * Only Multi-Zone and Verbose modes produce a point-cloud
		 * frame on each DATA_READY pulse, so ml_sensorInfoGetApi()
		 * can be called in the work handler.  Autonomous mode only
		 * signals presence via the GPIO — it does not support frame
		 * data fetching.
		 */
		struct iwrl6322_data *data = dev->data;

		/*
		 * Reject modes that will never produce data on PRESENCE_DET.
		 * IWRL6322: mode 0 = STOP, mode 5 = POWER_OFF
		 */

		data->streaming_sqe = iodev_sqe;
		iwrl6322_reconfigure_gpio_int(dev);
		return;
	}

	/* One-shot path: dispatch fetch to the RTIO work queue. */
	struct rtio_work_req *req = rtio_work_req_alloc();

	if (req == NULL) {
		LOG_ERR("RTIO work alloc failed; increase CONFIG_RTIO_WORKQ_POOL_ITEMS");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, iwrl6322_submit_sync);
}

static DEVICE_API(sensor, iwrl6322_driver_api) = {
	.channel_get  = iwrl6322_channel_get,
	.trigger_set  = iwrl6322_trigger_set,
	.attr_set     = iwrl6322_attr_set,
	.attr_get     = iwrl6322_attr_get,
	.get_decoder  = iwrl6322_get_decoder,
	.submit       = iwrl6322_iodev_submit,
};

/* ============================================================================
 * Init / Deinit
 * ========================================================================== */

static int iwrl6322_struct_init(const struct device *dev)
{
	g_priv = dev->data;
	g_priv->firmware_loaded  = false;
	
	/* Device / app config */
	g_priv->dev_antenna_sel   = CONFIG_IWRL6322_ANTENNA_TYPE;
	g_priv->mpd_holdtime_10us = CONFIG_IWRL6322_MPD_HOLDTIME_10US;
	g_priv->fe_calib_ctrl     = CONFIG_IWRL6322_FE_CALIB_CTRL; /* all calibrations enabled */
	
	/* MPD profile */
	g_priv->mpd_meas_prof_sel       	= CONFIG_IWRL6322_ATTR_MPD_MEAS_PROF_SEL;
	g_priv->mpd_detection_mode 			= CONFIG_IWRL6322_MPD_DETECTION_MODE;
	g_priv->mpd_snr_boost      			= CONFIG_IWRL6322_MPD_SNR_BOOST;
	g_priv->mpd_num_frames    		 	= CONFIG_IWRL6322_MPD_NUM_FRAMES;
	g_priv->frame_period_ms    			= CONFIG_IWRL6322_MPD_FRAME_PERIOD_MS;
	g_priv->mpd_adv_cfar_cfg     		= CONFIG_IWRL6322_MPD_ADV_CFAR_CFG;
	g_priv->mpd_min_points				= CONFIG_IWRL6322_MPD_MIN_POINTS;
	g_priv->mpd_max_distance_cluster	= CONFIG_IWRL6322_MPD_MAX_DISTANCE_CLUSTER;
	g_priv->mpd_steer_vec_en_ctrl		= CONFIG_IWRL6322_MPD_STEER_VEC_EN_CTRL;

	/* Custom steering vector config — disabled by default */
	g_priv->steer_vec_custom_ena     = false;
	g_priv->steer_vec_num_azim_bins  = 1U;
	g_priv->steer_vec_num_elev_bins  = 1U;
	g_priv->steer_vec_azim_step      = 0U;
	g_priv->steer_vec_elev_step      = 0U;
	g_priv->steer_vec_elem_size      = 0U; /* 4-bit */

	/* FE chirp profile */
	g_priv->fe_adc_samp_rate   		= CONFIG_IWRL6322_FE_ADC_SAMP_RATE;
	g_priv->fe_num_virtual_ant 		= CONFIG_IWRL6322_FE_NUM_VIRTUAL_ANT;
	g_priv->fe_txrx_ch_ena     		= CONFIG_IWRL6322_FE_TXRX_CH_ENA;
	g_priv->fe_num_adc_samples 		= CONFIG_IWRL6322_FE_NUM_ADC_SAMPLES;
	g_priv->fe_ramp_end_time   		= CONFIG_IWRL6322_FE_RAMP_END_TIME;
	g_priv->fe_idle_time       		= CONFIG_IWRL6322_FE_IDLE_TIME;
	g_priv->fe_adc_skip_samp   		= CONFIG_IWRL6322_FE_ADC_SKIP_SAMP;
	g_priv->fe_rf_freq_start   		= CONFIG_IWRL6322_FE_RF_FREQ_START;
	g_priv->fe_rf_freq_slope   		= CONFIG_IWRL6322_FE_RF_FREQ_SLOPE;
	g_priv->fe_tx_start_time   		= CONFIG_IWRL6322_FE_TX_START_TIME;
	g_priv->fe_hpf_corner      		= CONFIG_IWRL6322_FE_HPF_CORNER;
	g_priv->fe_cal_rx_gain     		= CONFIG_IWRL6322_FE_CAL_RX_GAIN;
	g_priv->fe_cal_tx_backoff  		= CONFIG_IWRL6322_FE_CAL_TX_BACKOFF;
	g_priv->fe_inter_chirp_ps_ovrd  = CONFIG_IWRL6322_FE_INTER_CHIRP_PS_OVRD;

	/* FE chirp frame */
	g_priv->fe_chirps_accum   	= CONFIG_IWRL6322_FE_CHIRPS_ACCUM;
	g_priv->fe_chirps_per_burst = CONFIG_IWRL6322_FE_CHIRPS_PER_BURST;
	g_priv->fe_burst_idle_time  = CONFIG_IWRL6322_FE_BURST_IDLE_TIME;
	g_priv->fe_bursts_per_frame = CONFIG_IWRL6322_FE_BURSTS_PER_FRAME;
	g_priv->fe_num_frames       = CONFIG_IWRL6322_FE_NUM_FRAMES;
	g_priv->fe_frame_period_ms  = CONFIG_IWRL6322_FE_FRAME_PERIOD_MS;

	/* Mode bitfields — default to MPD presence (quiet) */
	g_priv->smart_fe_op_mode  = M_ML_DEV_OP_MODE_MPD_ENABLE;
	g_priv->data_tx_mode      = M_ML_DEV_OUTPUT_MODE_MPD_DATA;
	g_priv->mpd_data_out_mode = M_ML_DEV_MPD_SIGNAL_OUT_MODE_QUIET;

	/* App config ctrl — U_APP_CFG_CTRL miscellaneous bitfields */
	g_priv->mpd_sig_trig_type    	= IS_ENABLED(CONFIG_IWRL6322_MPD_SIG_TRIG_TYPE);
	g_priv->mpd_auto_boot_mode   	= IS_ENABLED(CONFIG_IWRL6322_MPD_AUTO_BOOT_MODE);
	g_priv->mcu_clk_out_sel      	= CONFIG_IWRL6322_MCU_CLK_OUT_SEL;
	g_priv->deep_sleep_entry_dis	= IS_ENABLED(CONFIG_IWRL6322_DEEP_SLEEP_ENTRY_DIS);
	g_priv->trig_boot_cal        	= IS_ENABLED(CONFIG_IWRL6322_TRIG_BOOT_CAL);
	g_priv->hil_enable           	= IS_ENABLED(CONFIG_IWRL6322_HIL_ENABLE);
	g_priv->jtag_enable          	= IS_ENABLED(CONFIG_IWRL6322_JTAG_ENABLE);
	g_priv->xtal_less_mode       	= IS_ENABLED(CONFIG_IWRL6322_XTAL_LESS_MODE);
	g_priv->gpio_marker_ena      	= IS_ENABLED(CONFIG_IWRL6322_GPIO_MARKER_ENA);
	g_priv->cz_cw_mode_ena       	= IS_ENABLED(CONFIG_IWRL6322_CZ_CW_MODE_ENA);
	g_priv->rf_ldo_bypass        	= IS_ENABLED(CONFIG_IWRL6322_RF_LDO_BYPASS);
	g_priv->hold_lpds            	= IS_ENABLED(CONFIG_IWRL6322_HOLD_LPDS);
	g_priv->mount_type				= IS_ENABLED(CONFIG_IWRL6322_MOUNT_TYPE);

	/* Sensor start/stop calibration and flash options */
	g_priv->run_cal_trig_interval = CONFIG_IWRL6322_RUN_CAL_TRIG_INTERVAL;
	g_priv->flash_sens_cfg_data   = IS_ENABLED(CONFIG_IWRL6322_FLASH_SENS_CFG);

	/* Sensor position and tilt — per-installation, default 0; set via attr */
	g_priv->sens_pos_x_cm       = 0;
	g_priv->sens_pos_y_cm       = 0;
	g_priv->sens_pos_z_cm       = 0;
	g_priv->sens_tilt_azimuth   = 0;
	g_priv->sens_tilt_elevation = 0;
	g_priv->sens_in_plane_tilt  = 0;

	/* Reset and power-up timing (init-time) */
	g_priv->nreset_setup_time_10us = CONFIG_IWRL6322_NRESET_SETUP_TIME_10US;
	g_priv->rbl_powerup_delay_ms   = CONFIG_IWRL6322_RBL_POWERUP_DELAY_MS;

	/* API timeout */
	g_priv->api_timeout_ms = CONFIG_IWRL6322_API_TIMEOUT_MS;

	/* Initialise runtime config fields from Kconfig defaults.
	 * attr_set() may override these before the first sensor start. */
	g_priv->max_range_m           		= CONFIG_IWRL6322_MPD_MAX_RANGE;
	g_priv->motion_sensitivity    		= CONFIG_IWRL6322_MPD_MOTION_SENSITIVITY;
	g_priv->detection_sensitivity 		= CONFIG_IWRL6322_MPD_DETECTION_SENSITIVITY;
	g_priv->region                		= CONFIG_IWRL6322_REGION;
	g_priv->run_cal_ctrl          		= CONFIG_IWRL6322_RUN_CAL_CTRL;
	g_priv->disable_static_obj_removal 	= !IS_ENABLED(CONFIG_IWRL6322_MPD_STATIC_REMOVAL);
	g_priv->num_zones             		= CONFIG_IWRL6322_MPD_NUM_ZONES;
	g_priv->coordinate_cartesian  		= IS_ENABLED(CONFIG_IWRL6322_ZONE_COORDINATE_CARTESIAN);

	/* Default zone 0: full coverage up to max range */
	g_priv->zones[0].xmin_cm = -500;
	g_priv->zones[0].xmax_cm =  500;
	g_priv->zones[0].ymin_cm = 0;
	g_priv->zones[0].ymax_cm = 1200;
	g_priv->zones[0].zmin_cm = 0;
	g_priv->zones[0].zmax_cm = 300;
	g_priv->zones[0].include = true;

	/* Initialise the TI mmWaveULink library with IWRL6322 callbacks */
	T_ML_LINK_INIT_CFG link_cfg;

	iwrl6322_build_init_cfg(&link_cfg, CONFIG_IWRL6322_API_TIMEOUT_MS);

	/* Device map: bit 0 = device index 0 */
	T_ML_STATUS ret = ml_mmWaveLinkInit(1U, &link_cfg);

	if (ret != M_RL_RET_CODE_OK) {
		LOG_ERR("IWRL6322: mmWaveLinkInit failed: %d", ret);
		return -EIO;
	}

	return 0;
}

static int __maybe_unused iwrl6322_deinit(const struct device *dev)
{
	ARG_UNUSED(dev);
	ml_mmWaveLinkDeinit(1U);
	return 0;
}

/* ============================================================================
 * Initialisation
 * ========================================================================== */

static int iwrl6322_init(const struct device *dev)
{
	struct iwrl6322_data *data = dev->data;
	const struct iwrl6322_config *cfg = dev->config;
	int ret;

	/* Register global reference for callbacks before calling the library */
	g_dev = dev;

	/* ------------------------------------------------------------------ */
	/* GPIO setup                                                           */
	/* ------------------------------------------------------------------ */

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* nRESET: output, initially low — device held in reset (optional) */
	if (cfg->nreset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->nreset_gpio)) {
			LOG_ERR("nRESET GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->nreset_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("nRESET configure failed: %d", ret);
			return ret;
		}
	}

	/* SPI_BUSY: input */
	if (!gpio_is_ready_dt(&cfg->spi_busy_gpio)) {
		LOG_ERR("SPI_BUSY GPIO not ready");
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&cfg->spi_busy_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("SPI_BUSY configure failed: %d", ret);
		return ret;
	}

	/* PRESENCE_DETECT: optional input with edge interrupt */
	if (cfg->pres_det_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->pres_det_gpio)) {
			LOG_ERR("PRES_DET GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->pres_det_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("PRES_DET configure failed: %d", ret);
			return ret;
		}

		gpio_init_callback(&data->pres_det_cb,
				   iwrl6322_pres_det_gpio_cb,
				   BIT(cfg->pres_det_gpio.pin));
		ret = gpio_add_callback(cfg->pres_det_gpio.port,
					&data->pres_det_cb);
		if (ret < 0) {
			LOG_ERR("PRES_DET add_callback failed: %d", ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&cfg->pres_det_gpio,
						      GPIO_INT_EDGE_RISING);
		if (ret < 0) {
			LOG_ERR("PRES_DET interrupt configure failed: %d", ret);
			return ret;
		}
	}

	/* SOP0: optional output, initially low — boot mode select */
	if (cfg->sop0_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->sop0_gpio)) {
			LOG_ERR("SOP0 GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->sop0_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("SOP0 configure failed: %d", ret);
			return ret;
		}
	}

	/* SOP1: optional output, initially low — boot mode select */
	if (cfg->sop1_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->sop1_gpio)) {
			LOG_ERR("SOP1 GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->sop1_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("SOP1 configure failed: %d", ret);
			return ret;
		}
	}

		/* SOP0: optional output, initially low — boot mode select */
	if (cfg->sop0_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->sop0_gpio)) {
			LOG_ERR("SOP0 GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->sop0_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("SOP0 configure failed: %d", ret);
			return ret;
		}
	}

	/* SOP1: optional output, initially low — boot mode select */
	if (cfg->sop1_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->sop1_gpio)) {
			LOG_ERR("SOP1 GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->sop1_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("SOP1 configure failed: %d", ret);
			return ret;
		}
	}

	/* Pull WAKE_UP low again to see if the line will actually go low */
	if (cfg->wake_up_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->wake_up_gpio)) {
			LOG_ERR("WAKE_UP GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->wake_up_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("WAKE_UP configure failed: %d", ret);
			return ret;
		}
	}

	k_work_init(&data->trig_work, iwrl6322_trig_work_handler);

	static const struct k_work_queue_config wq_cfg = {
		.name = "iwrl6322_wq",
	};
	k_work_queue_start(&iwrl6322_work_q, iwrl6322_work_q_stack,
			   K_THREAD_STACK_SIZEOF(iwrl6322_work_q_stack),
			   CONFIG_IWRL6322_WORK_QUEUE_PRIORITY,
			   &wq_cfg);

	ret = iwrl6322_struct_init(dev);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("TI IWRL6322 sensor ready (range=%dm, rate=%dHz, mode=%d, zones=%d)",
		data->max_range_m, data->update_rate, iwrl6322_get_mode(data),
		data->num_zones);
	return 0;
}

/* ============================================================================
 * Device Tree Instantiation
 * ========================================================================== */

#define IWRL6322_DEFINE(n)                                                        \
	static struct iwrl6322_data iwrl6322_data_##n;                              \
                                                                                \
	static const struct iwrl6322_config iwrl6322_config_##n = {                 \
		.spi = SPI_DT_SPEC_INST_GET(                                    \
			n,                                                      \
			SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |                  \
			SPI_TRANSFER_MSB),                                      \
		.spi_busy_gpio = GPIO_DT_SPEC_INST_GET(n, spi_busy_gpios),     \
		.nreset_gpio   = GPIO_DT_SPEC_INST_GET_OR(n, nreset_gpios, {0}), \
		.wake_up_gpio  = GPIO_DT_SPEC_INST_GET_OR(                     \
			n, wake_up_gpios, {0}),                                 \
		.pres_det_gpio = GPIO_DT_SPEC_INST_GET_OR(                     \
			n, presence_detect_gpios, {0}),                         \
		.sop0_gpio     = GPIO_DT_SPEC_INST_GET_OR(n, sop0_gpios, {0}), \
		.sop1_gpio     = GPIO_DT_SPEC_INST_GET_OR(n, sop1_gpios, {0}), \
	};                                                                      \
                                                                                \
	SENSOR_DEVICE_DT_INST_DEFINE(                                           \
		n,                                                              \
		iwrl6322_init, NULL,                                              \
		&iwrl6322_data_##n, &iwrl6322_config_##n,                           \
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                       \
		&iwrl6322_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IWRL6322_DEFINE)
