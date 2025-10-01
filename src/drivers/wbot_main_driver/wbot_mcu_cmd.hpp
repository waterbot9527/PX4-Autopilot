#pragma once

enum MCU_CMD_TYPE {
	// 和单片机里 对应
	MCU_CMD_TYPE_STOP_MOTO = 1,
	MCU_CMD_TYPE_START_MOTO = 2 ,
	MCU_CMD_TYPE_REBOOT = 4 ,
	MCU_CMD_TYPE_SET_LED = 6,
};

class McuCmdHelper
{
public :

	static bool set_motor_cmd(uint8_t *cmd, uint8_t speed, uint8_t direction, uint8_t i2c_index )
	{
		if ( speed > 0)
		{
			// 仅仅低 4位 被设置了， 其他bit 都是 0
			cmd[0] = MCU_CMD_TYPE_START_MOTO;
		} else
		{
			cmd[0] = MCU_CMD_TYPE_STOP_MOTO ;
		}

		if ( direction == 0 )
		{
			//反转
		} else {

			//正转
			cmd[0] |= 1 << 4;
		}

		if ( i2c_index < 4 )
		{
			cmd[0] |= i2c_index<<6;
		} else {
			return false;
		}

		cmd[1] = speed;
		return true;
	}

	static bool set_led_value(uint8_t *cmd, uint8_t value)
	{
		cmd[0] = MCU_CMD_TYPE_SET_LED;
		cmd[1] = value;
		return true;
	}


	static bool set_reboot_value(uint8_t *cmd, uint8_t value )
	{
		cmd[0] = MCU_CMD_TYPE_REBOOT;
		cmd[1] = value;
		return true;
	}


};
