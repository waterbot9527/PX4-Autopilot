
#include "wbot_mix_out.hpp"

using namespace time_literals;


WBotMixOut::WBotMixOut():
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{

}

int WBotMixOut::print_status()
{
	return 0;
}


void WBotMixOut::Run()
{
	// auto _now = hrt_absolute_time();
	// PX4_INFO("wbot mix out running= %lu", (uint64_t)_now);
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

		if ( mycnt == 100)
		{
			PX4_INFO("fuck0");
			updateParams();

			mycnt = 0xffffffff;
		} else if ( mycnt < 100 ) {
			mycnt++;
		}

	#endif

	_mixing_output.updateSubscriptions(false);
}

bool WBotMixOut::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			unsigned num_outputs, unsigned num_control_groups_updated)
{

#if 1
	printf("wbot output, num_output=%d\n", num_outputs);
	printf("wbot output data=\n");
	// [ 0,1,2,3,4,5 ]
	for (int n = 0; n <= 5; n++)
	{
		printf("0x%04x,", outputs[n]);
		if ( n == 7 ) printf("\n");
	}
	printf("\n");
#endif

	return true;
}

int WBotMixOut::print_usage(const char *reason)
{
	PRINT_MODULE_USAGE_NAME("wbot_mix_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int WBotMixOut::task_spawn(int argc, char *argv[])
{
	PX4_INFO("wbox mix out main task swpan");

	WBotMixOut *instance = new WBotMixOut();

	if (!instance) {
		PX4_ERR("WBotMixOut alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}

int WBotMixOut::custom_command(int argc, char *argv[])
{
	return 0;
}

extern "C" __EXPORT int wbot_mix_out_main(int argc, char *argv[])
{
	return WBotMixOut::main(argc, argv);
}
