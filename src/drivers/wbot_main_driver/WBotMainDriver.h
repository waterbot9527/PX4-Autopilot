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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file WBotMainDriver.h
 *
 * Water Robot Main Driver for PX4.
 */

#pragma once

#include <stdint.h>


#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/conversion/rotation.h>


#include <uORB/topics/wbot_ctrl_moto.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/wbot_ctrl_led.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>


class WBotMainDriver : public ModuleBase<WBotMainDriver>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	WBotMainDriver(uint8_t rotation_value, uint8_t max_dev_id);
	~WBotMainDriver() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int Start();


	static constexpr uint32_t TOTAL_SERIAL_COUNT = 2;

	uint32_t _max_dev_id;

private:
	orb_advert_t _water_press_pub = nullptr;
	orb_advert_t _water_temp_pub = nullptr;

	void Run() override;

	// 针对一个 usb 串口执行操作
	void RunForOne(uint32_t dev_id, uint32_t cmd_size);

	uint32_t check_update(void);

	int parse_mcu_data(uint8_t dev_id, uint8_t *data);
	bool parse_imu_data(uint8_t dev_id, uint8_t *data, uint32_t len);
	bool parse_ms5837_data(uint8_t dev_id, uint8_t *data, uint32_t len);
	bool parse_motor_data(uint8_t dev_id, uint8_t *data, uint32_t moto_index, uint32_t len);

	Rotation rotation{Rotation::ROTATION_NONE};


	int _serial_fd[TOTAL_SERIAL_COUNT] = {-1, -1};
	char _serial_name[TOTAL_SERIAL_COUNT][4096];

	static const int  CMD_BUF_SIZE = 256;
	uint8_t send_cache[2][CMD_BUF_SIZE];
	uint8_t recv_cache[CMD_BUF_SIZE];

	// 设备状态跟踪
	bool _device_connected[TOTAL_SERIAL_COUNT] = {false, false};
	hrt_abstime _last_disconnect_time[TOTAL_SERIAL_COUNT] = {0, 0};
	int _disconnect_count[TOTAL_SERIAL_COUNT] = {0, 0};
	static constexpr uint32_t RECONNECT_INTERVAL_US = 1000000; // 5秒重连间隔
	static constexpr int MAX_DISCONNECT_COUNT = 5; // 最大断开次数

	// 添加辅助函数
	void handle_device_disconnect(uint8_t dev_id);
	bool attempt_reconnect(uint8_t dev_id);

	int _wbot_moto_sub = -1;
	int _wbot_led_sub = -1;

	hrt_abstime _now = hrt_absolute_time();


	PX4Accelerometer *_px4_accel[TOTAL_SERIAL_COUNT] = { nullptr , nullptr};
	PX4Gyroscope *_px4_gyro[TOTAL_SERIAL_COUNT] = { nullptr , nullptr};
	PX4Magnetometer *_px4_mag[TOTAL_SERIAL_COUNT] = { nullptr, nullptr };

	perf_counter_t _bad_packhead_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad packet header")};
	perf_counter_t _bad_packtail_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad packet tail")};
	perf_counter_t _bad_crc_err_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad crc checksum")};
	perf_counter_t _right_perf{perf_alloc(PC_COUNT, MODULE_NAME": all_right")};

};
