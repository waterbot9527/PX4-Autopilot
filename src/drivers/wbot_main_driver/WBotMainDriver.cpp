/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMages (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file WBotMainDriver.cpp
 */

#include "WBotMainDriver.h"
#include <px4_platform_common/time.h>
#include <lib/drivers/st_lsm6dsv16x_common/lsm6dsv16x_reg.h>
#include <lib/drivers/st_lis2mdl_common/lis2mdl_reg.h>
#include <string.h>
#include "spi_mcu_def.h"
#include "lsm6dsv16_utils.h"

using namespace time_literals;

WBotMainDriver::WBotMainDriver(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation),
	_px4_mag(get_device_id(), config.rotation)
{
	wbot_crc32_init_table();

	_px4_gyro.set_scale(math::radians(1.0f)); // parse 的时候已经做过scale 处理了
	_px4_gyro.set_range(math::radians(1000.f)); // 和单片机里的参数一致

	// Accelerometer configuration 16 G range
	_px4_accel.set_scale(CONSTANTS_ONE_G); // parse 的时候已经做过scale 处理了
	_px4_accel.set_range(2.0f * CONSTANTS_ONE_G); // 和单片机里的参数一致

}

WBotMainDriver::~WBotMainDriver()
{
}

int WBotMainDriver::init()
{
	PX4_INFO("Water Robot Main Driver Initialized!");

	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);

		return ret;
	}
	DEVICE_DEBUG("SPI::init ok (%i)", ret);
	DEVICE_DEBUG("spi dev id= %i addr= %i", get_device_id(), get_device_address());
	_wbot_moto_sub = orb_subscribe_multi(ORB_ID(wbot_moto), get_device_address());

	return Reset() ? 0 : -1;
}

bool WBotMainDriver::Reset()
{

	ScheduleClear();
	ScheduleNow();

	uint32_t interva_delay_us = 1000*1000;
	ScheduleOnInterval(interva_delay_us, interva_delay_us);

	return true;
}

void WBotMainDriver::RunImpl()
{
	//PX4_INFO("Water Robot Main Driver running!");
	if (should_exit()) {
		PX4_INFO("Water Robot Main Driver quit!");
		exit_and_cleanup();
		return;
	}

	bool updated;
	orb_check(_wbot_moto_sub, &updated);  // 检查订阅的 topic 是否有新数据

	if (updated) {
		struct wbot_moto_s data;
		orb_copy(ORB_ID(wbot_moto), _wbot_moto_sub, &data);
		PX4_INFO("dev(%i) Got new data: %i %i",
			get_device_address(), data.speed[0], data.speed[1]);

		// TODO: set motor/led command
	}

	//const hrt_abstime now = hrt_absolute_time();

	static uint8_t rs_cache[MAX_SPI_BUF_LEN];
	// TODO: add cmd
	if (PX4_OK != transfer(rs_cache, rs_cache, sizeof(rs_cache)) )
	{
		PX4_WARN("wbot main spi can't read , spi id=%i", get_device_address());
		return;
	}

	// get data ok
	parse_spi_data(rs_cache);


}

bool WBotMainDriver::parse_spi_ms5837_data(uint8_t *data, uint32_t len)
{
	if ( len != 8 )
	{
		return false;
	}

	uint32_t pressure_raw = data[0] | (data[1]<<8) | (data[2]<<16) | (data[3]<<24);
	uint32_t temperature_raw = data[4] | (data[5]<<8) | (data[6]<<16) | (data[7]<<24);

	float temperature_celsius 	= temperature_raw / 100.0;
	float pressure_mbar   		= pressure_raw / 10.0;

	(void)temperature_celsius;
	(void)pressure_mbar;
	//todo： publish data
	return true;
}

bool WBotMainDriver::parse_spi_imu_data(uint8_t *data, uint32_t len)
{
	uint32_t cnt = len / 7;
	if ( len % 7 != 0 )
	{
		return false ;
	}

	int16_t *datax, *datay, *dataz;
	(void)datax;(void)datay;(void)dataz;
	for (uint32_t n=0; n<cnt; n++)
	{
		lsm6dsv16x_fifo_out_raw_t f_data;
		lsm6dsv16x_fifo_out_raw_parse(&f_data, &data[7*n]);
		datax = (int16_t *)&f_data.data[0];
		datay = (int16_t *)&f_data.data[2];
		dataz = (int16_t *)&f_data.data[4];
		switch (f_data.tag) {
		case 2: //LSM6DSV16X_XL_NC_TAG:
		{
			lsm6dsv16x_from_fs2_to_mg(*datax);
			lsm6dsv16x_from_fs2_to_mg(*datay);
			lsm6dsv16x_from_fs2_to_mg(*dataz);
			break;
		}
		case 4: //LSM6DSV16X_TIMESTAMP_TAG:
		{
			int32_t *ts = (int32_t *)&f_data.data[0];
			float_t aa = lsm6dsv16x_from_lsb_to_nsec(*ts)/1000;
			(void)aa;
			break;
		}
		case 1: //LSM6DSV16X_GY_NC_TAG:
		{
			// datasheet : Table 3. Mechanical characteristics
                	lsm6dsv16x_from_fs1000_to_mdps(*datax);
                	lsm6dsv16x_from_fs1000_to_mdps(*datay);
                	lsm6dsv16x_from_fs1000_to_mdps(*dataz);
          		break;
		}
		case 0xE: //LSM6DSV16X_SENSORHUB_SLAVE0_TAG:
		{
			// 无干扰下， 正常数据应在 400 mG 数量级（几百毫高斯）

			lis2mdl_from_lsb_to_mgauss(*datax);
			lis2mdl_from_lsb_to_mgauss(*datay);
			lis2mdl_from_lsb_to_mgauss(*dataz);
			break;
		}
		case 0: //LSM6DSV16X_FIFO_EMPTY:
		{
			break;
		}
		default:
			break;
		}

	}

	return true;
}

// data: 接收到的 SPI 包
// 返回值: 0 成功，负数表示错误
int WBotMainDriver::parse_spi_data(uint8_t *data) {
    if (!data) return -1; // 缓冲区太小

    // 检查包头
    if (data[0] != 0x5a || data[1] != 0x5a) return -2;

    uint32_t total_len = data[2]; // copy_data 填写的总长度
    if (total_len < 10 ) return -3; // 长度非法

    // 检查包尾
    if (data[total_len-6] != 0xa5 || data[total_len-5] != 0xa5) return -4;

    // CRC 校验
    uint32_t crc_recv = data[total_len-4] | (data[total_len-3]<<8) |
                        (data[total_len-2]<<16) | (data[total_len-1]<<24);
    uint32_t crc_calc = wbot_crc32(data, total_len-4);
    if (crc_recv != crc_calc) return -5;

    // 解析有效数据
    uint32_t index = 3; // 跳过包头 + total_len 字段
    while (index + 3 <= total_len - 6) { // 至少 3 字节包头
        uint16_t data_len = data[index] | (data[index+1] << 8);
        uint8_t tag = data[index+2];
        index += 3;

        if (index + data_len > total_len - 6) return -6; // 数据越界

        // 处理数据
        printf("TAG %02X, LEN %d, DATA:", tag, data_len);
        for (uint16_t i = 0; i < data_len; i++) {
            printf(" %02X", data[index + i]);
        }


	switch (tag)
	{
	case WBOT_SDEV_TAG_IMU:
	{
		if ( !parse_spi_imu_data( &data[index+3], data_len) )
		{
			PX4_ERR("wbot main driver parse imu data error");
		}
		break;
	}
	case WBOT_SDEV_TAG_MS5837:
	{
		if ( !parse_spi_ms5837_data( &data[index+3], data_len) )
		{
			PX4_ERR("wbot main driver parse ms5837 data error");
		}
		break;
	}
	case WBOT_SDEV_TAG_MOTO0:
		break;
	case WBOT_SDEV_TAG_MOTO1:
		break;
	case WBOT_SDEV_TAG_MOTO2:
		break;
	case WBOT_SDEV_TAG_MOTO3:
		break;
	default:
		break;
	}
        printf("\n");

        index += data_len;
    }

    return 0; // 成功
}

void WBotMainDriver::print_status()
{
	PX4_INFO("Water Robot Main Driver status");
}

int WBotMainDriver::probe()
{
	PX4_INFO("probe");
	//不用探测， 默认存在
	return PX4_OK;
}

void WBotMainDriver::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}
