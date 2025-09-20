#include "lsm6dsv16_utils.h"
#include <string.h>

void lsm6dsv16x_fifo_out_raw_parse(lsm6dsv16x_fifo_out_raw_t *val, uint8_t *buff)
{
  lsm6dsv16x_fifo_data_out_tag_t fifo_data_out_tag;

  memcpy((uint8_t *)&fifo_data_out_tag, &buff[0], 1);

  switch (fifo_data_out_tag.tag_sensor)
  {
    case LSM6DSV16X_FIFO_EMPTY:
      val->tag = LSM6DSV16X_FIFO_EMPTY;
      break;

    case LSM6DSV16X_GY_NC_TAG:
      val->tag = LSM6DSV16X_GY_NC_TAG;
      break;

    case LSM6DSV16X_XL_NC_TAG:
      val->tag = LSM6DSV16X_XL_NC_TAG;
      break;

    case LSM6DSV16X_TIMESTAMP_TAG:
      val->tag = LSM6DSV16X_TIMESTAMP_TAG;
      break;

    case LSM6DSV16X_TEMPERATURE_TAG:
      val->tag = LSM6DSV16X_TEMPERATURE_TAG;
      break;

    case LSM6DSV16X_CFG_CHANGE_TAG:
      val->tag = LSM6DSV16X_CFG_CHANGE_TAG;
      break;

    case LSM6DSV16X_XL_NC_T_2_TAG:
      val->tag = LSM6DSV16X_XL_NC_T_2_TAG;
      break;

    case LSM6DSV16X_XL_NC_T_1_TAG:
      val->tag = LSM6DSV16X_XL_NC_T_1_TAG;
      break;

    case LSM6DSV16X_XL_2XC_TAG:
      val->tag = LSM6DSV16X_XL_2XC_TAG;
      break;

    case LSM6DSV16X_XL_3XC_TAG:
      val->tag = LSM6DSV16X_XL_3XC_TAG;
      break;

    case LSM6DSV16X_GY_NC_T_2_TAG:
      val->tag = LSM6DSV16X_GY_NC_T_2_TAG;
      break;

    case LSM6DSV16X_GY_NC_T_1_TAG:
      val->tag = LSM6DSV16X_GY_NC_T_1_TAG;
      break;

    case LSM6DSV16X_GY_2XC_TAG:
      val->tag = LSM6DSV16X_GY_2XC_TAG;
      break;

    case LSM6DSV16X_GY_3XC_TAG:
      val->tag = LSM6DSV16X_GY_3XC_TAG;
      break;

    case LSM6DSV16X_SENSORHUB_SLAVE0_TAG:
      val->tag = LSM6DSV16X_SENSORHUB_SLAVE0_TAG;
      break;

    case LSM6DSV16X_SENSORHUB_SLAVE1_TAG:
      val->tag = LSM6DSV16X_SENSORHUB_SLAVE1_TAG;
      break;

    case LSM6DSV16X_SENSORHUB_SLAVE2_TAG:
      val->tag = LSM6DSV16X_SENSORHUB_SLAVE2_TAG;
      break;

    case LSM6DSV16X_SENSORHUB_SLAVE3_TAG:
      val->tag = LSM6DSV16X_SENSORHUB_SLAVE3_TAG;
      break;

    case LSM6DSV16X_STEP_COUNTER_TAG:
      val->tag = LSM6DSV16X_STEP_COUNTER_TAG;
      break;

    case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
      val->tag = LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG;
      break;

    case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
      val->tag = LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG;
      break;

    case LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG:
      val->tag = LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG;
      break;

    case LSM6DSV16X_SENSORHUB_NACK_TAG:
      val->tag = LSM6DSV16X_SENSORHUB_NACK_TAG;
      break;

    case LSM6DSV16X_MLC_RESULT_TAG:
      val->tag = LSM6DSV16X_MLC_RESULT_TAG;
      break;

    case LSM6DSV16X_MLC_FILTER:
      val->tag = LSM6DSV16X_MLC_FILTER;
      break;

    case LSM6DSV16X_MLC_FEATURE:
      val->tag = LSM6DSV16X_MLC_FEATURE;
      break;

    case LSM6DSV16X_XL_DUAL_CORE:
      val->tag = LSM6DSV16X_XL_DUAL_CORE;
      break;

    case LSM6DSV16X_GY_ENHANCED_EIS:
      val->tag = LSM6DSV16X_GY_ENHANCED_EIS;
      break;

    default:
      val->tag = LSM6DSV16X_FIFO_EMPTY;
      break;
  }

  val->cnt = fifo_data_out_tag.tag_cnt;

  val->data[0] = buff[1];
  val->data[1] = buff[2];
  val->data[2] = buff[3];
  val->data[3] = buff[4];
  val->data[4] = buff[5];
  val->data[5] = buff[6];

  return;
}
