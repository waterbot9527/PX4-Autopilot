/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <atomic>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/wbot_ctrl_moto.h>

class WBotActuatorBridge : public ModuleBase<WBotActuatorBridge>,
    public ModuleParams,
    public px4::ScheduledWorkItem
{
public:
    WBotActuatorBridge();
    ~WBotActuatorBridge() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    int print_status() override;

    bool init();

private:
    void Run() override;

    void _stop_and_unregister();
    void _publish_stop();
    void _publish_translated(const actuator_motors_s &actuator_motors);
    void _handle_actuator_test(const actuator_test_s &actuator_test);
    void _publish_test_override();
    bool _has_test_override() const;
    void _debug_log_output(const char *source, const float *input, const wbot_ctrl_moto_s &msg);

    static constexpr int kMotorCount = 8;
    static constexpr float kScaleToSpeed = 255.f;
    static constexpr uint64_t kDebugLogIntervalUs = 100000;

    uORB::SubscriptionCallbackWorkItem _actuator_motors_sub{this, ORB_ID(actuator_motors)};
    uORB::SubscriptionCallbackWorkItem _actuator_test_sub{this, ORB_ID(actuator_test)};
    uORB::Publication<wbot_ctrl_moto_s> _wbot_ctrl_moto_pub{ORB_ID(wbot_ctrl_moto)};

    bool _test_override_active[kMotorCount] {};
    float _test_override_value[kMotorCount] {};
    uint64_t _last_debug_log_us {0};
    bool _last_override_state {false};
};
