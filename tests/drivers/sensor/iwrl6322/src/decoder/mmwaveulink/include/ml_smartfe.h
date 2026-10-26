/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal stub for <mmwaveulink/include/ml_smartfe.h> used by the IWRL6322
 * decoder unit tests.
 *
 * The decoder (iwrl6322_decoder.c) only needs the layout of
 * T_ML_SENS_POINT_CLOUD_DATA and the SINT16/UINT16 typedefs.  All other
 * TI library declarations are omitted; none of the library's runtime
 * functions are called by the decoder.
 */

#ifndef MMWAVEULINK_INCLUDE_ML_SMARTFE_STUB_H_
#define MMWAVEULINK_INCLUDE_ML_SMARTFE_STUB_H_

#include <stdint.h>

/* TI primitive types used inside T_ML_SENS_POINT_CLOUD_DATA */
typedef int16_t  SINT16;
typedef uint16_t UINT16;

/**
 * @brief One detected target as returned by the IWRL6322 firmware.
 *
 * Layout must exactly match struct iwrl6322_point_data in iwrl6322.h —
 * a BUILD_ASSERT in iwrl6322_decoder.c enforces this at compile time.
 *
 *   xh_XR       → x_cm      (int16_t,  1 LSB = 1 cm, signed)
 *   xh_YTheta   → y_cm      (uint16_t, 1 LSB = 1 cm)
 *   xh_Z        → z_cm      (int16_t,  1 LSB = 1 cm, signed)
 *   xh_Velocity → velocity  (int16_t)
 *   h_Snr       → snr_01db  (int16_t,  1 LSB = 0.1 dB)
 *   h_Noise     → noise_01db(int16_t,  1 LSB = 0.1 dB)
 */
typedef struct {
	SINT16 xh_XR;       /**< X / R coordinate, 1 LSB = 1 cm */
	SINT16 xh_YTheta;   /**< Y / theta coordinate, 1 LSB = 1 cm */
	SINT16 xh_Z;        /**< Z / height, 1 LSB = 1 cm */
	SINT16 xh_Velocity; /**< Radial velocity, 1 LSB = 1 cm/s */
	UINT16 h_Snr;       /**< SNR, 1 LSB = 0.1 dB */
	UINT16 h_Noise;     /**< Noise, 1 LSB = 0.1 dB */
} T_ML_SENS_POINT_CLOUD_DATA;

#endif /* MMWAVEULINK_INCLUDE_ML_SMARTFE_STUB_H_ */
