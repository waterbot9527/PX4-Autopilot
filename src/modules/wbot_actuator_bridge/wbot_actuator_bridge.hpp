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

    static constexpr int kMotorCount = 8;
    static constexpr float kScaleToSpeed = 255.f;

    uORB::SubscriptionCallbackWorkItem _actuator_motors_sub{this, ORB_ID(actuator_motors)};
    uORB::Publication<wbot_ctrl_moto_s> _wbot_ctrl_moto_pub{ORB_ID(wbot_ctrl_moto)};
};
