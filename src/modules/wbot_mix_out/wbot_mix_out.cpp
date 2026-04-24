
#include "wbot_mix_out.hpp"
#include <cstdlib>
// #define MOTOR_TEST

using namespace time_literals;


static void set_raspberry_led(double pwm_value)
{
    const char *pwm_path = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle";
    const uint32_t max_duty_cycle = 200000-1;

    // 确保pwm_value在[0.0, 1.0]范围内
    if (pwm_value < 0.0) {
        pwm_value = 0.0;
    } else if (pwm_value > 1.0) {
        pwm_value = 1.0;
    }

    // 以写模式打开文件（w = 覆盖写入）
    FILE *f = fopen(pwm_path, "w");
    if (!f) {
        PX4_ERR("Failed to open PWM file: %s", pwm_path);
        return;
    }

    unsigned int duty_cycle_value = (unsigned int)(max_duty_cycle * pwm_value);
    int result = fprintf(f, "%u", duty_cycle_value);

    if (result < 0) {
        PX4_ERR("Failed to write to PWM file: %s", strerror(errno));
    }

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

#ifdef MOTOR_TEST
	static const float control_mode_config[WBOT_MAX_CONTROL_MODE_CNT][WBOT_MAX_MOTO_CNT] = {
		//{主推1，上浮下潜1，主推2，上浮下潜2，左右转，上浮下潜，俯仰1，俯仰2}
		{ -1.0, +0.0, +0.0, -0.0, +0.0, +0.0, +0.0, +0.0 }, /* 摇杆2-X-R */
		{ +0.0, -1.0, -0.0, +0.0, +0.0, +0.0, +0.0, +0.0 }, /* 摇杆2-X-L */
		{ +0.0, +0.0, +1.0, +0.0, -0.0, +0.0, +0.0, +0.0 }, /* 摇杆2-Y-U */
		{ +0.0, +0.0, -0.0, -1.0, +0.0, +0.0, +0.0, -0.0 }, /* 摇杆2-Y-D */
		{ +0.0, +0.0, -0.0, +0.0, +1.0, +0.0, +0.0, +0.0 }, /* 摇杆1-Y-U */
		{ -0.0, +0.0, +0.0, -0.0, +0.0, +1.0, -0.0, -0.0 }, /* 摇杆1-Y-D */
		{ +0.0, -0.0, +0.0, +0.0, +0.0, +0.0, -1.0, +0.0 }, /* 摇杆1-X-R */
		{ +0.0, +0.0, +0.0, +0.0, -0.0, -0.0, +0.0, +1.0 }  /* 摇杆1-X-L */
	};
		// { -0.0  -0.0, +0.0, +0.0, +0.0, -0.0, +0.0, +1.0 }, /* 摇杆1-X-R */ 125daqiu
		// { +0.0, +0.0, -0.0, -0.0, -0.0, +0.0, +0.0, -1.0 }, /* 摇杆1-X-L */
		// { +0.0, +1.0, +1.0, +0.0, -0.0, +0.0, -0.0, +0.0 }, /* 摇杆1-Y-U */
		// { -0.0, -1.0, -1.0, -0.0, +0.0, +0.0, +0.0, -0.0 }, /* 摇杆1-Y-D */
		// { +0.0, +0.0, -0.0, +0.0, +1.0, -1.0, +0.0, +0.0 }, /* 摇杆2-Y-U */
		// { -0.0, -0.0, +0.0, -0.0, -1.0, +1.0, -0.0, -0.0 }, /* 摇杆2-Y-D */
		// { +1.0, +0.0, +0.0, +0.0, +0.0, +0.0, -1.0, +0.0 }, /* 摇杆2-X-R */
		// { -1.0, -0.0, +0.0, +0.0, -0.0, -0.0, +1.0, +0.0 }  /* 摇杆2-X-L */

		//{下潜上浮1，前进后退1，前进后退2,上浮下潜2，右左转，俯仰1，俯仰2，空} 1号，不带注入
		{ -0.0,	+0.0, +0.0, +0.0, +1.0, -0.0, +0.0, +0.0 }, /* 摇杆1-X-R */
		{ +0.0, -0.0, -0.0, -0.0, -1.0, +0.0, +0.0, -0.0 }, /* 摇杆1-X-L */
		{ +0.0, -1.0, -0.0, -1.0, -0.0, +0.0, -0.0, +0.0 }, /* 摇杆1-Y-U */
		{ -0.0, +1.0, +0.0, +1.0, +0.0, +0.0, +0.0, -0.0 }, /* 摇杆1-Y-D */
		{ +0.0, +0.0, -0.0, +0.0, +0.0, +1.0, +1.0, +0.0 }, /* 摇杆2-Y-U */
		{ -0.0, -0.0, +0.0, -0.0, -0.0, -1.0, -1.0, -0.0 }, /* 摇杆2-Y-D */
		{ -1.0, +0.0, -1.0, +0.0, +0.0, +0.0, -0.0, +0.0 }, /* 摇杆2-X-R */
		{ +1.0, -0.0, +1.0, -0.0, -0.0, -0.0, +0.0, +0.0 }  /* 摇杆2-X-L */
#else
	static const float control_mode_config[WBOT_MAX_CONTROL_MODE_CNT][WBOT_MAX_MOTO_CNT] = {
		//{下潜上浮1，前进后退1，上浮下潜2,前进后退2,右左转，俯仰1，俯仰2，空} 带注入
		{ -0.0,	+0.0, +0.0, +0.0, +1.0, -0.0, +0.0, +0.0 }, /* 摇杆1-X-R */
		{ +0.0, -0.0, -0.0, -0.0, -1.0, +0.0, +0.0, -0.0 }, /* 摇杆1-X-L */
		{ +0.0, -1.0, -0.0, -1.0, -0.0, +0.0, -0.0, +0.0 }, /* 摇杆1-Y-U */
		{ -0.0, +1.0, +0.0, +1.0, +0.0, +0.0, +0.0, -0.0 }, /* 摇杆1-Y-D */
		{ +0.0, +0.0, -0.0, +0.0, +0.0, -1.0, +1.0, +0.0 }, /* 摇杆2-Y-U */
		{ -0.0, -0.0, +0.0, -0.0, -0.0, +1.0, -1.0, -0.0 }, /* 摇杆2-Y-D */
		{ -1.0, +0.0, +1.0, +0.0, +0.0, +0.0, -0.0, +0.0 }, /* 摇杆2-X-R */
		{ +1.0, -0.0, -1.0, -0.0, -0.0, -0.0, +0.0, +0.0 }  /* 摇杆2-X-L */
	};
#endif // MOTOR_TEST


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
		// PX4_INFO("mix publish data: %i %i %i %i %i %i %i %i",
		// _moto_msg.speed[0], _moto_msg.speed[1],_moto_msg.speed[2],_moto_msg.speed[3],_moto_msg.speed[4],_moto_msg.speed[5],_moto_msg.speed[6],_moto_msg.speed[7]);
	} else {
		PX4_ERR("Failed to publish wbot_ctrl_moto: publisher not inited");
	}
	// 添加按钮处理逻辑
	if (num_outputs > 5) {  // 确保outputs[5]存在
	uint16_t raw_buttons = outputs[5];

	_current_buttons.led_increase = (raw_buttons >> 0) & 0x1;      // bit 0: 0x1
	_current_buttons.reboot_MCU = (raw_buttons >> 1) & 0x1;        // bit 1: 0x2
	_current_buttons.led_decrease = (raw_buttons >> 2) & 0x1;      // bit 2: 0x4

	// LED亮度增加控制 - 检测边沿触发（从0变到1）
	if (_current_buttons.led_increase && !_previous_buttons.led_increase) {
		handle_led_brightness_control(16);  // 增加亮度
	}

	// LED亮度减少控制 - 检测边沿触发（从0变到1）
	if (_current_buttons.led_decrease && !_previous_buttons.led_decrease) {
		handle_led_brightness_control(-16);  // 减少亮度
	}

	// MCU重启控制 - 检测边沿触发（从0变到1）
	if (_current_buttons.reboot_MCU && !_previous_buttons.reboot_MCU) {
		handle_reboot_mcu();
	}

	// 更新上一次的按钮状态
	_previous_buttons = _current_buttons;
	}

#endif

	return true;
}

// 在cpp文件中添加辅助函数
void WBotMixOut::handle_led_brightness_control(int8_t change_value) {
    double raspberry_pwm;
    _led_msg.led_id = 0;

    int32_t new_value = _led_msg.light_value + change_value;

    // 限制范围
    if (new_value >= 240) {
        new_value = 240;
    } else if (new_value <= 0) {
        new_value = 0;
    }

    _led_msg.light_value = new_value;
    raspberry_pwm = _led_msg.light_value / 255.0;
    set_raspberry_led(raspberry_pwm);
	// 前后灯型号一致，直接发布 Raspberry LED 的亮度值
	_led_msg.timestamp = hrt_absolute_time();
	if (_led_pub != nullptr) {
		orb_publish(ORB_ID(wbot_ctrl_led), _led_pub, &_led_msg);
		PX4_INFO("Published wbot_led message (LED brightness: %d)", _led_msg.light_value);
	} else {
		PX4_ERR("Failed to publish wbot_ctrl_led: publisher not inited");
	}
}

void WBotMixOut::handle_reboot_mcu()
{
	_led_msg.led_id = 1;  // 重启命令
	_led_msg.timestamp = hrt_absolute_time();
	if (_led_pub != nullptr) {
		orb_publish(ORB_ID(wbot_ctrl_led), _led_pub, &_led_msg);
		PX4_INFO("Published MCU reboot command");
	} else {
		PX4_ERR("Failed to publish wbot_ctrl_led: publisher not inited");
	}
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
