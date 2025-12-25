
#include "wbot_mix_out.hpp"
#include <cstdlib>


using namespace time_literals;


static void set_raspberry_led(double pwm_value)
{
    const char *pwm_path = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle";
    const uint32_t max_duty_cycle = 200000-1;

    // 以写模式打开文件（w = 覆盖写入）
    FILE *f = fopen(pwm_path, "w");
    if (!f) {
        perror("Failed to open file");
        return;
    }
    if (pwm_value < 0.0)
    {
	pwm_value = 0.0;
    }

    if (pwm_value > 1.0)
    {
	pwm_value = 1.0;
    }
    fprintf(f, "%u", (unsigned int)(max_duty_cycle * pwm_value));

    fclose(f);  // 关闭文件
}

WBotMixOut::WBotMixOut():
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{

}
WBotMixOut::~WBotMixOut()
{
    if (_moto_pub != nullptr) {
        orb_unadvertise(_moto_pub);
        _moto_pub = nullptr;
    }
    if (_led_pub != nullptr) {
        orb_unadvertise(_led_pub);
        _led_pub = nullptr;
    }
}

void WBotMixOut::_init_orb_publishers()
{
    // 初始化电机消息发布者
    if (_moto_pub == nullptr) {
        _moto_pub = orb_advertise(ORB_ID(wbot_ctrl_moto), &_moto_msg);
        if (_moto_pub == nullptr) {
            PX4_ERR("Failed to advertise wbot_ctrl_moto topic");
        } else {
            PX4_INFO("Successfully advertised wbot_ctrl_moto topic");
        }
    }

    // 初始化 LED 消息发布者
    if (_led_pub == nullptr) {
        _led_pub = orb_advertise(ORB_ID(wbot_ctrl_led), &_led_msg);
        if (_led_pub == nullptr) {
            PX4_ERR("Failed to advertise wbot_ctrl_led topic");
        } else {
            PX4_INFO("Successfully advertised wbot_ctrl_led topic");
        }
    }
}

int WBotMixOut::print_status()
{
	return 0;
}


void WBotMixOut::Run()
{
	// auto _now = hrt_absolute_time();
	// PX4_INFO("wbot mix out running= %lu", (uint64_t)_now);

	static bool _orb_inited = false;
	if (!_orb_inited) {
		_init_orb_publishers();
		_orb_inited = true;  // 标记为已初始化，后续不再执行
	}
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

#if 0
	printf("wbot output, num_output=%d\n", num_outputs);
	printf("wbot output data=\n");
	// [ 0,1,2,3,4,5 ]
	for (int n = 0; n <= 5; n++)
	{
		printf("0x%04x,", outputs[n]);
		if ( n == 7 ) printf("\n");
	}
	printf("\n");
#else
	#define WBOT_MAX_CONTROL_MODE_CNT (8)
	#define WBOT_MAX_MOTO_CNT (8)
	/*
	数组的 值是比例系数
	"摇杆1-X-L" 意思是： 摇杆1 在X 轴上 从中心往左移动，程序采样的值从[0 到 1.0]
	"摇杆1-X-R" 意思是： 摇杆1 在X 轴上 从中心往右移动，程序采样的值从[0 到 1.0]
	其值是个 数组，长度必须为 8 。 数组里的每个值是比例系数
	譬如 ： "摇杆1-X-L" = [ 0, 0.2, 0.5, 0, 0, 0, 0, 0]
	意思是 摇杆1 在X轴上 左移动读出来的数据 是  0.56 时， 转换成电机的数据是
	[ 0*255*0.56, 0.2*255*0.56 .... ]
	*/
	static const float control_mode_config[WBOT_MAX_CONTROL_MODE_CNT][WBOT_MAX_MOTO_CNT] = {
		{ -1.0, +0.0, +0.0, -0.0, +0.0, +0.0, +0.0, +0.0 }, /* 摇杆1-X-L */
		{ +1.0, -0.0, -0.0, +0.0, +0.0, +0.0, +0.0, +0.0 }, /* 摇杆1-X-R */
		{ +0.0, +0.0, +0.0, +0.0, -1.0, +0.0, +0.0, +1.0 }, /* 摇杆1-Y-U */
		{ +0.0, +0.0, -0.0, -0.0, +1.0, +0.0, +0.0, -1.0 }, /* 摇杆1-Y-D */
		{ +0.0, +0.0, -1.0, +1.0, +0.0, +0.0, +0.0, +0.0 }, /* 摇杆2-X-U */
		{ -0.0, +0.0, +1.0, -1.0, +0.0, +0.0, -0.0, -0.0 }, /* 摇杆2-X-D */
		{ +0.0, -1.0, +0.0, +0.0, +0.0, +1.0, -1.0, +0.0 }, /* 摇杆2-Y-L */
		{ +0.0, +1.0, +0.0, +0.0, -0.0, -1.0, +1.0, +0.0 }  /* 摇杆2-Y-R */
	};

	//only for debug
	// for ( int n = 0; n < MAX_ACTUATORS; n++)
	// {
	// 	outputs[n] = 0;
	// }
	// outputs[0] = 256 + 128;
	if (outputs[0] == 0 && outputs[1] == 0 && outputs[2] == 0 && outputs[3] == 0 )
	{
		outputs[0] = outputs[1] = outputs[2] = outputs[3] = 255;
	}
	float f_speed[WBOT_MAX_MOTO_CNT] = { 0 };
	for ( int n = 0; n < 4 ; n++)
	{
		int32_t control_mode_value = outputs[n];
		int32_t control_mode_abs_value = control_mode_value;
		control_mode_value -= 256;

		if (control_mode_value > 255)
			control_mode_value = 255;
		else if ( control_mode_value < -255 )
			control_mode_value = -255;


		control_mode_abs_value = std::abs(control_mode_value);


		int ctrl_mode ;
		if ( control_mode_value > 0 ) {
			//处理
			ctrl_mode = 0 + n*2;
		} else {
			ctrl_mode = 1 + n*2;
		}
		for ( int k = 0; k < WBOT_MAX_MOTO_CNT; k++ )
		{
			f_speed[k] += (control_mode_config[ctrl_mode][k] * ((float)control_mode_abs_value));
		}
	}
	for ( int n = 0; n < WBOT_MAX_MOTO_CNT; n++)
	{
		if ( f_speed[n] >= 0 ) {

			_moto_msg.direction[n] = 1;
		} else {
			_moto_msg.direction[n] = 0;
		}

		int32_t abs_speed = (int32_t)(std::abs(f_speed[n]));
		if ( abs_speed > 255 )
			abs_speed = 255;
		if ( abs_speed < 20 )
			abs_speed = 0;

		_moto_msg.speed[n] = abs_speed;
	}

        _moto_msg.timestamp = hrt_absolute_time();
        if (_moto_pub != nullptr) {  // 检查发布者是否有效
        orb_publish(ORB_ID(wbot_ctrl_moto), _moto_pub, &_moto_msg);
	} else {
		PX4_ERR("Failed to publish wbot_ctrl_moto: publisher not inited");
	}


	led_increase_button_value = 0x1 & outputs[5];
	if((led_increase_button_value == 0x1 )&&  (led_increase_button_value!= led_increase_button_lastvalue))
	{
		double raspberry_pwm;
		_led_msg.led_id = 0;

		if (_led_msg.light_value >= 240)
		{
			_led_msg.light_value = 240;
		}
		else
		{
			_led_msg.light_value += 16;
		}

		raspberry_pwm = _led_msg.light_value / 255.0;
		set_raspberry_led(raspberry_pwm);
		_led_msg.timestamp = hrt_absolute_time();
		if (_led_pub != nullptr) {
            		orb_publish(ORB_ID(wbot_ctrl_led), _led_pub, &_led_msg);
            		PX4_INFO("Published wbot_led message (LED button %d)",_led_msg.light_value);  // 调试用

		} else {
			PX4_ERR("Failed to publish wbot_led: publisher not inited");
		}
	}
		led_increase_button_lastvalue = led_increase_button_value;

	led_decrease_button_value = 0x4 & outputs[5];
	if((led_decrease_button_value == 0x4 )&&  (led_decrease_button_value!= led_decrease_button_lastvalue))
	{
		double raspberry_pwm;
		_led_msg.led_id = 0;
		if (_led_msg.light_value <= 0)
		{
			_led_msg.light_value = 0;
		}
		else
		{
			_led_msg.light_value -= 16;
		}

		raspberry_pwm = _led_msg.light_value / 255.0;
		set_raspberry_led(raspberry_pwm);
		_led_msg.timestamp = hrt_absolute_time();
		if (_led_pub != nullptr) {
            		orb_publish(ORB_ID(wbot_ctrl_led), _led_pub, &_led_msg);
            		PX4_INFO("Published wbot_led message (LED button %d)",_led_msg.light_value);  // 调试用

		} else {
			PX4_ERR("Failed to publish wbot_led: publisher not inited");
		}
	}
		led_decrease_button_lastvalue = led_decrease_button_value;

	reboot_button_value = 0b10 & outputs[5]; // 按钮？？
	if(reboot_button_value != reboot_button_lastvalue)
	{
		_led_msg.led_id = 1;
		_led_msg.timestamp = hrt_absolute_time();
		if (_led_pub != nullptr) {
			orb_publish(ORB_ID(wbot_ctrl_led), _led_pub, &_led_msg);
			// PX4_INFO("Published wbot_led message (Reboot button)");  // 调试用
		} else {
			PX4_ERR("Failed to publish wbot_led: publisher not inited");
		}
		reboot_button_lastvalue = reboot_button_value;
	}
	// PX4_INFO(" reboot_button_value = %d\n ", reboot_button_value);

        // 发布LED消息

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
