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
#include <array>


#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/conversion/rotation.h>

#include <uORB/Subscription.hpp>  // 添加这个头文件以支持uORB::Subscription
#include <uORB/PublicationMulti.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/uuvmotor.h>
#include <uORB/topics/wbot_ctrl_moto.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/wbot_ctrl_led.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/water_depth.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>


class WBotMainDriver : public ModuleBase<WBotMainDriver>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	WBotMainDriver(uint8_t imu_rotation_value, uint8_t mag_rotation_value, uint8_t max_dev_id, int8_t imu_publish_dev);
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
	static constexpr uint8_t MOTOR_MAX_INDEX = 8; // 每个设备最多支持的电机索引数量
	static constexpr uint8_t UUVMOTOR_INSTANCE_COUNT = TOTAL_SERIAL_COUNT * MOTOR_MAX_INDEX;

	std::array<uORB::PublicationMulti<water_depth_s>, TOTAL_SERIAL_COUNT> _water_depth_pub{{
		uORB::PublicationMulti<water_depth_s>(ORB_ID(water_depth)),
		uORB::PublicationMulti<water_depth_s>(ORB_ID(water_depth))
	}}; // 水深数据发布句柄

	void Run() override;

	// 针对一个 usb 串口执行操作
	void RunForOne(uint32_t dev_id, uint32_t cmd_size);

	uint32_t check_update(void);

	int parse_mcu_data(uint8_t dev_id, uint8_t *data);
	bool parse_imu_data(uint8_t dev_id, uint8_t *data, uint32_t len);
	bool parse_ms5837_data(uint8_t dev_id, uint8_t *data, uint32_t len);
	bool parse_motor_data(uint8_t dev_id, uint8_t *data, uint32_t moto_index, uint32_t len);

	// 新增函数用于打印传感器数据
	void printSensorData();

	Rotation _rotation_imu{Rotation::ROTATION_NONE};
	Rotation _rotation_mag{Rotation::ROTATION_NONE};
	int8_t _imu_publish_dev{-1}; // -2: disable publish, -1: publish all, 0/1: publish only selected dev

	// 添加用于监听传感器数据和姿态的订阅者
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _sensor_gyro_sub{ORB_ID(sensor_gyro)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// 添加标志来控制打印频率
	bool _print_once{true};

	// 添加上次打印时间
	hrt_abstime _last_print_time{0};

	int _serial_fd[TOTAL_SERIAL_COUNT] = {-1, -1};
	char _serial_name[TOTAL_SERIAL_COUNT][4096];

	static const int  CMD_BUF_SIZE = 256;
	uint8_t send_cache[2][CMD_BUF_SIZE];
	uint8_t recv_cache[CMD_BUF_SIZE];

	// 设备状态跟踪
	bool _device_connected[TOTAL_SERIAL_COUNT] = {false, false};
	hrt_abstime _last_disconnect_time[TOTAL_SERIAL_COUNT] = {0, 0};
	int _disconnect_count[TOTAL_SERIAL_COUNT] = {0, 0};
	static constexpr uint32_t RECONNECT_INTERVAL_US = 2000000; // 5秒重连间隔
	static constexpr int MAX_DISCONNECT_COUNT = 20; // 最大断开次数


	// 添加辅助函数
	void handle_device_disconnect(uint8_t dev_id);
	bool attempt_reconnect(uint8_t dev_id);

	int _wbot_moto_sub = -1;
	int _wbot_led_sub = -1;

	// debug_key_value 发布句柄，用于通过 NAMED_VALUE_FLOAT MAVLink 发送水深数据到地面站
	orb_advert_t _debug_pressure_pub{nullptr};
	orb_advert_t _debug_temp_pub{nullptr};
	orb_advert_t _debug_depth_pub{nullptr};

	hrt_abstime _now = hrt_absolute_time();


	PX4Accelerometer *_px4_accel[TOTAL_SERIAL_COUNT] = { nullptr , nullptr};
	PX4Gyroscope *_px4_gyro[TOTAL_SERIAL_COUNT] = { nullptr , nullptr};
	PX4Magnetometer *_px4_mag[TOTAL_SERIAL_COUNT] = { nullptr, nullptr };

	perf_counter_t _bad_packhead_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad packet header")};
	perf_counter_t _bad_packtail_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad packet tail")};
	perf_counter_t _bad_crc_err_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad crc checksum")};
	perf_counter_t _right_perf{perf_alloc(PC_COUNT, MODULE_NAME": all_right")};

	// uuv motor publish: one uORB instance per physical motor (dev_id * MOTOR_MAX_INDEX + motor_id)
	std::array<uORB::PublicationMulti<uuvmotor_s> *, UUVMOTOR_INSTANCE_COUNT> _uuvmotor_pub{};

	// last publish time per (dev_id, motor_index) for rate limiting
	hrt_abstime _last_motor_pub[TOTAL_SERIAL_COUNT][MOTOR_MAX_INDEX] = {{0}};

	// minimum interval between publishes per motor (us)
	static constexpr hrt_abstime MOTOR_PUB_INTERVAL_US = 20000; // 50 Hz default

};
