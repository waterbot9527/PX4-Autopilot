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

#include <drivers/device/spi.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/mixer_module/mixer_module.hpp>

#include <uORB/topics/wbot_ctrl_moto.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/wbot_ctrl_led.h>


#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#define MAX_WBOT_ACTUATORS (8)


class WBotMainOutput : public ModuleBase<WBotMainOutput>, public OutputModuleInterface
{
public:
	WBotMainOutput():
		OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
	{

	}
	virtual ~WBotMainOutput()
	{

	}

	int init()
	{
		return 0;
	}

	void Run() {}

	/** @see ModuleBase::print_status() */
	int print_status()
	{
		return 0;
	}

	void update_params()
	{

	}

	void wbot_run_once()
	{
		_mixing_output.update();

		#if 0
		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
		}
		#else

			if ( mycnt == 1000)
			{
				PX4_INFO("fuck0");
				updateParams();

				mycnt = 0xffffffff;
			} else if ( mycnt < 1000 ) {
				mycnt++;
			}

		#endif

		_mixing_output.updateSubscriptions(false);
	}

	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
	{
		// printf("wbot output, num_output=%d\n", num_outputs);

		// printf("wbot output data=");
		// for (int n =0; n < MAX_ACTUATORS; n++)
		// {
		// 	printf("0x%04x,", outputs[n]);
		// }
		// printf("\n");

		return true;
	}

private:
	uint32_t mycnt = 0;
	static constexpr int MAX_ACTUATORS = 8;

	MixingOutput _mixing_output{PARAM_PREFIX, MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};

class WBotMainDriver : public ::device::SPI, public I2CSPIDriver<WBotMainDriver>
{
public:
	WBotMainDriver(const I2CSPIDriverConfig &config);
	~WBotMainDriver() override;

	static void print_usage();

	int init() override;


	void print_status() override;

	void RunImpl() ;


private:
	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;
	PX4Magnetometer _px4_mag;

	WBotMainOutput wbot_output{};

	orb_advert_t _water_press_pub = nullptr;


	hrt_abstime _now = hrt_absolute_time();

	static const int  SPI_BUF_SIZE = 256;
	uint8_t send_recv_cache[SPI_BUF_SIZE];
	uint32_t test_cnt = 0;

	int _wbot_moto_sub = -1;
	int _wbot_led_sub = -1;

	perf_counter_t _bad_packhead_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad packet header")};
	perf_counter_t _bad_packtail_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad packet tail")};
	perf_counter_t _bad_crc_err_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad crc checksum")};

	int parse_spi_data(uint8_t *data);

	bool parse_spi_imu_data(uint8_t *data, uint32_t len);

	bool parse_spi_ms5837_data(uint8_t *data, uint32_t len);

	bool parse_spi_motor_data(uint8_t *data, uint32_t moto_index, uint32_t len);


	void exit_and_cleanup() override;
	int probe() override;

	bool Reset();
};
