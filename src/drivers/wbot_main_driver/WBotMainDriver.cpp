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

using namespace time_literals;

WBotMainDriver::WBotMainDriver(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
}

WBotMainDriver::~WBotMainDriver()
{
}

int WBotMainDriver::init()
{
	PX4_INFO("Water Robot Main Driver Initialized!");
	ScheduleOnInterval(1_s); // Print message every 1 second
	return PX4_OK;
}

void WBotMainDriver::RunImpl()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}
	//const hrt_abstime now = hrt_absolute_time();


	PX4_INFO("Water Robot Main Driver running!");
}

void WBotMainDriver::print_status()
{
	PX4_INFO("Water Robot Main Driver status");
}

int WBotMainDriver::probe()
{
	//不用探测， 默认存在
	return PX4_OK;
}

void WBotMainDriver::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}
