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

WBotMainDriver::WBotMainDriver() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

WBotMainDriver::~WBotMainDriver()
{
}

bool WBotMainDriver::Init()
{
	PX4_INFO("Water Robot Main Driver Initialized!");
	ScheduleOnInterval(1_s); // Print message every 1 second
	return true;
}

void WBotMainDriver::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	PX4_INFO("Water Robot Main Driver running!");
}

int WBotMainDriver::task_spawn(int argc, char *argv[])
{
	WBotMainDriver *instance = new WBotMainDriver();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	if (!instance->Init()) {
		delete instance;
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return PX4_OK;
}

int WBotMainDriver::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "test")) {
		PX4_INFO("Water Robot Main Driver custom command test!");
		return 0;
	}

	return print_usage("unknown command");
}

int WBotMainDriver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Water Robot Main Driver for PX4.

This driver provides main functionality for water robot operations.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wbot_main_driver", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test custom command");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int wbot_main_driver_main(int argc, char *argv[])
{
	return WBotMainDriver::main(argc, argv);
}
