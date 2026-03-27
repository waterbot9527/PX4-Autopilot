/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "wbot_actuator_bridge.hpp"

#include <cmath>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

using namespace time_literals;

WBotActuatorBridge::WBotActuatorBridge() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

WBotActuatorBridge::~WBotActuatorBridge()
{
	_stop_and_unregister();
}

int WBotActuatorBridge::task_spawn(int argc, char *argv[])
{
	WBotActuatorBridge *instance = new WBotActuatorBridge();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int WBotActuatorBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WBotActuatorBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Bridge PX4 control allocation motor outputs to wbot_ctrl_moto messages.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wbot_actuator_bridge", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int WBotActuatorBridge::print_status()
{
	PX4_INFO("running");
	return 0;
}

bool WBotActuatorBridge::init()
{
	_publish_stop();

	if (!_actuator_motors_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void WBotActuatorBridge::Run()
{
	if (should_exit()) {
		_stop_and_unregister();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	actuator_motors_s actuator_motors{};

	if (_actuator_motors_sub.update(&actuator_motors)) {
		_publish_translated(actuator_motors);
	}
}

void WBotActuatorBridge::_stop_and_unregister()
{
	_actuator_motors_sub.unregisterCallback();
	_publish_stop();
}

void WBotActuatorBridge::_publish_stop()
{
	wbot_ctrl_moto_s msg{};
	msg.timestamp = hrt_absolute_time();

	for (int i = 0; i < kMotorCount; ++i) {
		msg.speed[i] = 0;
		msg.direction[i] = 1;
	}

	_wbot_ctrl_moto_pub.publish(msg);
}

void WBotActuatorBridge::_publish_translated(const actuator_motors_s &actuator_motors)
{
	wbot_ctrl_moto_s msg{};
	msg.timestamp = hrt_absolute_time();

	for (int i = 0; i < kMotorCount; ++i) {
		const float control = actuator_motors.control[i];

		if (!PX4_ISFINITE(control)) {
			msg.speed[i] = 0;
			msg.direction[i] = 1;
			continue;
		}

		const float clamped = math::constrain(control, -1.f, 1.f);
		const float magnitude = fabsf(clamped);
		const int speed = math::constrain(static_cast<int>(lroundf(magnitude * kScaleToSpeed)), 0, 255);

		msg.speed[i] = static_cast<uint8_t>(speed);
		msg.direction[i] = (clamped >= 0.f) ? 1 : 0;
	}

	_wbot_ctrl_moto_pub.publish(msg);
}

extern "C" __EXPORT int wbot_actuator_bridge_main(int argc, char *argv[])
{
	return WBotActuatorBridge::main(argc, argv);
}
