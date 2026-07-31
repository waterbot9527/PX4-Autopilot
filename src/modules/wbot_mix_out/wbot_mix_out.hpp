#pragma once


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/mixer_module/mixer_module.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/wbot_ctrl_moto.h>
#include <uORB/topics/wbot_ctrl_led.h>


#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <time.h>


#define MAX_WBOT_ACTUATORS (8)


class WBotMixOut : public ModuleBase<WBotMixOut>, public OutputModuleInterface
{
public:
	WBotMixOut(const char *config_path = nullptr);
	~WBotMixOut() override;

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
	static constexpr unsigned WBOT_MAX_CONTROL_MODE_CNT = 8;
	static constexpr unsigned WBOT_MAX_MOTO_CNT = 8;
	static constexpr unsigned WBOT_CONFIG_PATH_LEN = 128;

	void Run() override;

	uint32_t mycnt = 0;
	orb_advert_t _led_pub;
	orb_advert_t _moto_pub;
	struct wbot_ctrl_moto_s _moto_msg{};
	struct wbot_ctrl_led_s _led_msg{};
	private:
    struct ButtonState {
        uint16_t led_increase : 1;    // bit 0
        uint16_t reboot_MCU : 1;    // bit 1
        uint16_t led_decrease : 1;    // bit 2
        uint16_t reserved4 : 1;    // bit 3
        uint16_t reserved5 : 1;    // bit 4
        uint16_t reserved6 : 1;    // bit 5
        uint16_t reserved7 : 1;    // bit 6
        uint16_t reserved8 : 1;    // bit 7
        uint16_t reserved9 : 1;    // bit 8
        uint16_t reserved10 : 1;   // bit 9
        uint16_t reserved11 : 1;   // bit 10
        uint16_t reserved12 : 1;   // bit 11
        uint16_t reserved13 : 1;   // bit 12
        uint16_t reserved14 : 1;   // bit 13
        uint16_t reserved15 : 1;   // bit 14
        uint16_t reserved16 : 1;   // bit 15
    	};

	ButtonState _current_buttons{0};   // 初始化为0
	ButtonState _previous_buttons{0};  // 初始化为0
	uint16_t led_increase_button_value{0};
	uint16_t led_increase_button_lastvalue{0};
	uint16_t led_decrease_button_value{0};
	uint16_t led_decrease_button_lastvalue{0};


	uint16_t reboot_button_value{0};
	uint16_t reboot_button_lastvalue{0};

	MixingOutput _mixing_output{PARAM_PREFIX, 8, *this, MixingOutput::SchedulingPolicy::Auto, false};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	void _init_orb_publishers();
	void handle_led_brightness_control(int8_t change_value);
	void handle_reboot_mcu();
	void reset_control_mode_config();
	bool load_control_mode_config(const char *path, bool warn_on_missing);
	bool maybe_reload_control_mode_config();

	float _control_mode_config[WBOT_MAX_CONTROL_MODE_CNT][WBOT_MAX_MOTO_CNT]{};
	char _config_path[WBOT_CONFIG_PATH_LEN]{};
	time_t _config_mtime{};
	bool _config_loaded_from_file{false};
	hrt_abstime _last_config_check{0};
};
