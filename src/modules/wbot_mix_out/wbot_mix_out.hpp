#pragma once


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/mixer_module/mixer_module.hpp>

#include <uORB/topics/wbot_ctrl_moto.h>
#include <uORB/topics/wbot_ctrl_led.h>


#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>


#define MAX_WBOT_ACTUATORS (8)


class WBotMixOut : public ModuleBase<WBotMixOut>, public OutputModuleInterface
{
public:
	WBotMixOut();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int init()
	{
		return 0;
	}



	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	void Run() override;

	uint32_t mycnt = 0;

	MixingOutput _mixing_output{PARAM_PREFIX, 8, *this, MixingOutput::SchedulingPolicy::Auto, false};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};
