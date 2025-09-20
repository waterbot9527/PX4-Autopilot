#pragma once

#include <lib/drivers/st_lsm6dsv16x_common/lsm6dsv16x_reg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// enum foo
//   {
//     LSM6DSV16X_FIFO_EMPTY                    = 0x0,
//     LSM6DSV16X_GY_NC_TAG                     = 0x1,
//     LSM6DSV16X_XL_NC_TAG                     = 0x2,
//     LSM6DSV16X_TEMPERATURE_TAG               = 0x3,
//     LSM6DSV16X_TIMESTAMP_TAG                 = 0x4,
//     LSM6DSV16X_CFG_CHANGE_TAG                = 0x5,
//     LSM6DSV16X_XL_NC_T_2_TAG                 = 0x6,
//     LSM6DSV16X_XL_NC_T_1_TAG                 = 0x7,
//     LSM6DSV16X_XL_2XC_TAG                    = 0x8,
//     LSM6DSV16X_XL_3XC_TAG                    = 0x9,
//     LSM6DSV16X_GY_NC_T_2_TAG                 = 0xA,
//     LSM6DSV16X_GY_NC_T_1_TAG                 = 0xB,
//     LSM6DSV16X_GY_2XC_TAG                    = 0xC,
//     LSM6DSV16X_GY_3XC_TAG                    = 0xD,
//     LSM6DSV16X_SENSORHUB_SLAVE0_TAG          = 0xE,
//     LSM6DSV16X_SENSORHUB_SLAVE1_TAG          = 0xF,
//     LSM6DSV16X_SENSORHUB_SLAVE2_TAG          = 0x10,
//     LSM6DSV16X_SENSORHUB_SLAVE3_TAG          = 0x11,
//     LSM6DSV16X_STEP_COUNTER_TAG              = 0x12,
//     LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG = 0x13,
//     LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG       = 0x16,
//     LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG       = 0x17,
//     LSM6DSV16X_SENSORHUB_NACK_TAG            = 0x19,
//     LSM6DSV16X_MLC_RESULT_TAG                = 0x1A,
//     LSM6DSV16X_MLC_FILTER                    = 0x1B,
//     LSM6DSV16X_MLC_FEATURE                   = 0x1C,
//     LSM6DSV16X_XL_DUAL_CORE                  = 0x1D,
//     LSM6DSV16X_GY_ENHANCED_EIS               = 0x1E,
//   };

void lsm6dsv16x_fifo_out_raw_parse(lsm6dsv16x_fifo_out_raw_t *val, uint8_t *buff);

#ifdef __cplusplus
}
#endif
