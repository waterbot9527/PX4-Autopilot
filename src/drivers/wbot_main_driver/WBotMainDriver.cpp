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
#include "spi_mcu_def.h"
#include <px4_platform_common/time.h>

using namespace time_literals;



WBotMainDriver::WBotMainDriver(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	wbot_crc32_init_table();

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
	}

	//const hrt_abstime now = hrt_absolute_time();

	static uint8_t rs_cache[256];
	// TODO: add cmd
	if (PX4_OK != transfer(rs_cache, rs_cache, 256) )
	{
		PX4_WARN("wbot main spi can't read , spi id=%i", get_device_address());
		return;
	}


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
