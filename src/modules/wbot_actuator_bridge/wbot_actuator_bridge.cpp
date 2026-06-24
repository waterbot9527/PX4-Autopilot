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
	PX4_INFO("bridge init: motor_count=%d scale=%.1f", kMotorCount, (double)kScaleToSpeed);

	if (!_actuator_motors_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_actuator_test_sub.registerCallback()) {
		PX4_ERR("actuator_test callback registration failed");
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
	actuator_test_s actuator_test{};

	if (_actuator_test_sub.update(&actuator_test)) {
		_handle_actuator_test(actuator_test);

		if (actuator_test.function >= actuator_test_s::FUNCTION_MOTOR1
		    && actuator_test.function < actuator_test_s::FUNCTION_MOTOR1 + kMotorCount) {
			const int motor_index = actuator_test.function - actuator_test_s::FUNCTION_MOTOR1;
			PX4_INFO("actuator_test: fn=%u action=%u motor=%d value=%.3f", actuator_test.function,
				 actuator_test.action, motor_index, (double)actuator_test.value);
		}
	}

	const bool has_override = _has_test_override();

	if (has_override != _last_override_state) {
		PX4_INFO("override state changed: %s", has_override ? "ACTIVE" : "INACTIVE");
		_last_override_state = has_override;
	}

	if (has_override) {
		_publish_test_override();

	} else if (_actuator_motors_sub.update(&actuator_motors)) {
		_publish_translated(actuator_motors);
	}
}

void WBotActuatorBridge::_stop_and_unregister()
{
	_actuator_motors_sub.unregisterCallback();
	_actuator_test_sub.unregisterCallback();
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
	float input[kMotorCount] {};

	for (int i = 0; i < kMotorCount; ++i) {
		const float control = actuator_motors.control[i];
		input[i] = control;

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
	_debug_log_output("actuator_motors", input, msg);
}

void WBotActuatorBridge::_handle_actuator_test(const actuator_test_s &actuator_test)
{
	if (actuator_test.function < actuator_test_s::FUNCTION_MOTOR1
	    || actuator_test.function >= actuator_test_s::FUNCTION_MOTOR1 + kMotorCount) {
		return;
	}

	const int motor_index = actuator_test.function - actuator_test_s::FUNCTION_MOTOR1;

	if (actuator_test.action == actuator_test_s::ACTION_DO_CONTROL) {
		_test_override_active[motor_index] = true;
		_test_override_value[motor_index] = actuator_test.value;

	} else if (actuator_test.action == actuator_test_s::ACTION_RELEASE_CONTROL) {
		_test_override_active[motor_index] = false;
	}
}

bool WBotActuatorBridge::_has_test_override() const
{
	for (int i = 0; i < kMotorCount; ++i) {
		if (_test_override_active[i]) {
			return true;
		}
	}

	return false;
}

void WBotActuatorBridge::_publish_test_override()
{
	wbot_ctrl_moto_s msg{};
	msg.timestamp = hrt_absolute_time();
	float input[kMotorCount] {};

	for (int i = 0; i < kMotorCount; ++i) {
		if (!_test_override_active[i]) {
			input[i] = 0.f;
			msg.speed[i] = 0;
			msg.direction[i] = 1;
			continue;
		}

		const float value = _test_override_value[i];
		input[i] = value;

		if (!PX4_ISFINITE(value)) {
			msg.speed[i] = 0;
			msg.direction[i] = 1;
			continue;
		}

		const float clamped = math::constrain(value, -1.f, 1.f);
		const int speed = math::constrain(static_cast<int>(lroundf(fabsf(clamped) * kScaleToSpeed)), 0, 255);

		msg.speed[i] = static_cast<uint8_t>(speed);
		msg.direction[i] = (clamped >= 0.f) ? 1 : 0;
	}

	_wbot_ctrl_moto_pub.publish(msg);
	_debug_log_output("actuator_test", input, msg);
}

void WBotActuatorBridge::_debug_log_output(const char *source, const float *input, const wbot_ctrl_moto_s &msg)
{
	const uint64_t now = hrt_absolute_time();

	if (now - _last_debug_log_us < kDebugLogIntervalUs) {
		return;
	}

	_last_debug_log_us = now;

	PX4_INFO("bridge %s", source);
	PX4_INFO("  in:    [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
		 (double)input[0], (double)input[1], (double)input[2], (double)input[3],
		 (double)input[4], (double)input[5], (double)input[6], (double)input[7]);
	PX4_INFO("  speed: [%u %u %u %u %u %u %u %u] dir: [%u %u %u %u %u %u %u %u]",
		 (unsigned)msg.speed[0], (unsigned)msg.speed[1], (unsigned)msg.speed[2], (unsigned)msg.speed[3],
		 (unsigned)msg.speed[4], (unsigned)msg.speed[5], (unsigned)msg.speed[6], (unsigned)msg.speed[7],
		 (unsigned)msg.direction[0], (unsigned)msg.direction[1], (unsigned)msg.direction[2], (unsigned)msg.direction[3],
		 (unsigned)msg.direction[4], (unsigned)msg.direction[5], (unsigned)msg.direction[6], (unsigned)msg.direction[7]);
}

extern "C" __EXPORT int wbot_actuator_bridge_main(int argc, char *argv[])
{
	return WBotActuatorBridge::main(argc, argv);
}
