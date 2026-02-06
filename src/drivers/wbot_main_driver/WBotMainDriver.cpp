/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

#include <termios.h>
#include <glob.h>
#include <unistd.h>
#include <limits.h>
#include <stdint.h>

#include "wbot_mcu_cmd.hpp"
#include "wbot_mcu_def.h"
#include "WBotMainDriver.h"
#include <px4_platform_common/getopt.h>
#include <drivers/device/device.h>
#include <lib/drivers/st_lsm6dsv16x_common/lsm6dsv16x_reg.h>
#include <lib/drivers/st_lis2mdl_common/lis2mdl_reg.h>
#include "lsm6dsv16_utils.h"

#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/water_depth.h>

using namespace time_literals;

// Local guard for packet buffer size (must match CMD_BUF_SIZE)
static constexpr int kCmdBufSize = 256;

/*
查找所有的 usb 串口设备
*/

#if 1
static uint32_t list_tty(char serial_name[][PATH_MAX])
{

	//const char * dev0= "/sys/devices/platform/axi/1000480000.usb/usb1/1-1/1-1.4/1-1.4:1.0/tty";
	const char* dev0 =  "/sys/devices/platform/axi/1000480000.usb/usb5/5-1/5-1.4/5-1.4:1.0";
	const char* dev1 = "/sys/devices/platform/axi/1000480000.usb/usb5/5-1/5-1.3/5-1.3.4/5-1.3.4:1.0";

	serial_name[0][0] = serial_name[1][0] = '\0';

	const char *pattern = "/sys/class/tty/ttyACM*";
	glob_t g;
	uint32_t i = 0;
	uint32_t ret = 0;
	memset(&g, 0, sizeof(g));

	if (glob(pattern, 0, NULL, &g) != 0) {
		globfree(&g);
		return 0;
	}

	for (i = 0; i < g.gl_pathc && i < 2; i++) {
		const char *path = g.gl_pathv[i];
		const char *name = strrchr(path, '/');
		name = name ? name + 1 : path;

		char devnode[PATH_MAX];
		char realdev[PATH_MAX];

		snprintf(devnode, sizeof(devnode), "/dev/%s", name);
		if (realpath(path, realdev) == NULL) {
			strncpy(realdev, "(unresolved)", sizeof(realdev));
		} else {
			if ( 0 == strncmp(realdev, dev0, strlen(dev0))) {
				strcpy(serial_name[0], devnode);
				ret++;
			}
			else
			if ( 0 == strncmp(realdev, dev1, strlen(dev1))) {
				strcpy(serial_name[1], devnode);
				ret++;
			}
			printf("  sysfs    : %s\n\n", realdev);
		}
	}

	globfree(&g);
	return ret;
}
#endif

static int read_nonblock(int fd, uint8_t *data, int size)
{
	int already_read = 0;
	int ret = 0;
	for(int n = 0; n < 256 && already_read < size ; n++)
	{
		ret = ::read(fd, data + already_read, size-already_read);
		if (ret <= 0 )
		{
			if (errno == EAGAIN)
				continue;
			else
				return ret;
		}

		already_read += ret;
	}
	if ( already_read == 0)
		already_read = ret;

	return already_read;
}

static int read_full_packet(int serial_fd, uint8_t *data)
{
	int ret;
	int n;

	for (n = 0; n < 128;n++)
	{
		ret = read_nonblock(serial_fd, data, 1);
		if ( ret != 1 )
			return -1;

		if (data[0] == 0x5a) {
		 	break;
		}
	}
	if ( n == 128 )
		return -2;


	ret = read_nonblock(serial_fd, data+1, 3 );
	if ( ret != 3 )
		return -3;

	if (data[0] != 0x5a || data[1] != 0x5b || data[2] != 0x5c || data[3] != 0x5d )
	{
		printf("bad header = %x %x %x %x \n", data[0], data[1], data[2], data[3]);
		return -4;
	}

	ret = read_nonblock(serial_fd, data+4, 1 );
	if ( ret != 1 )
		return -5;
	uint8_t total_length = data[4];
	// basic length guard to avoid buffer overflow
	if (total_length < 5 || total_length > kCmdBufSize) {
		return -7;
	}

	uint8_t left_length = total_length - 5 ;
	do {
		ret = read_nonblock(serial_fd, data + total_length - left_length, left_length);
		if ( ret < 0 )
		{
			printf("errno=%d, ret=%d left_length=%d, total_length=%d\n", errno, ret, left_length, total_length);
			return -6 ;
		}
		if ( ret == 0 )
		{
			break;
		}
		left_length -= ret;
	} while ( left_length > 0 );

	return total_length;
}

static int open_serial_fd(const char *serial_port)
{

	struct termios tty;
	// 打开串口
	int serial_fd = ::open(serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK );
	if (serial_fd < 0) {
		fprintf(stderr, "错误: 无法打开串口 %s: %s\n", serial_port, strerror(errno));
		return -1;
	}

	// 配置串口参数
	if (tcgetattr(serial_fd, &tty) != 0) {
		fprintf(stderr, "错误: 无法获取串口属性: %s\n", strerror(errno));
		close(serial_fd);
		return -1 ;
	}

	// 设置为原始模式，禁用所有处理
	cfmakeraw(&tty);

	// 设置波特率为 115200
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// 设置串口参数为 8N1 (8数据位, 无校验, 1停止位)
	tty.c_cflag &= ~CSIZE;        // 清除数据位设置
	tty.c_cflag |= CS8;           // 8个数据位
	tty.c_cflag &= ~PARENB;       // 无校验位
	tty.c_cflag &= ~CSTOPB;       // 1个停止位 (CSTOPB=0 表示1个停止位)
	tty.c_cflag &= ~CRTSCTS;      // 禁用硬件流控
	tty.c_cflag |= CREAD | CLOCAL; // 启用接收器，忽略调制解调器控制线

	// 应用设置
	if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "错误: 无法设置串口属性: %s\n", strerror(errno));
		close(serial_fd);
		return -1;
	}
	return serial_fd;
}

WBotMainDriver::WBotMainDriver(uint8_t imu_rotation_value, uint8_t mag_rotation_value, uint8_t max_dev_id,
					 int8_t imu_publish_dev) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_rotation_imu(static_cast<Rotation>(imu_rotation_value)),
	_rotation_mag(static_cast<Rotation>(mag_rotation_value)),
	_imu_publish_dev(imu_publish_dev)
{
	_wbot_moto_sub = orb_subscribe(ORB_ID(wbot_ctrl_moto) );
	_wbot_led_sub = orb_subscribe(ORB_ID(wbot_ctrl_led));

	this->_max_dev_id = max_dev_id;
	if (_imu_publish_dev >= 0) {
		if (_imu_publish_dev >= static_cast<int8_t>(TOTAL_SERIAL_COUNT)) {
			PX4_WARN("imu_publish_dev=%d out of range, fallback to all", _imu_publish_dev);
			_imu_publish_dev = -1;

		} else if (_imu_publish_dev > static_cast<int8_t>(_max_dev_id)) {
			// ensure selected dev is initialized even if -d was smaller
			_max_dev_id = static_cast<uint32_t>(_imu_publish_dev);
		}
	}

	for(uint32_t dev_id = 0; dev_id <= max_dev_id; dev_id++)
	{
		_px4_accel[dev_id] = new PX4Accelerometer(14123200 + dev_id, _rotation_imu);
		_px4_gyro[dev_id] = new PX4Gyroscope(14123300 + dev_id, _rotation_imu);
		_px4_mag[dev_id] = new PX4Magnetometer(14123400 + dev_id, _rotation_mag);


		_px4_accel[dev_id]->set_scale(0.061f * CONSTANTS_ONE_G/1000); // 0.061 参考 lsm6dsv16x.pdf 的 Table 3
		_px4_accel[dev_id]->set_range(2.f * CONSTANTS_ONE_G);

		_px4_gyro[dev_id]->set_scale(math::radians(35.f / 1000.f)); // 70 mdps/LSB
		_px4_gyro[dev_id]->set_range(math::radians(2000.f));

		_px4_mag[dev_id]->set_scale(1.5f/1000.f); // 设置适当的刻度值
	}



	wbot_crc32_init_table();
}

WBotMainDriver::~WBotMainDriver()
{
	ScheduleClear();

	// release allocated sensor objects
	for (uint32_t dev_id = 0; dev_id < TOTAL_SERIAL_COUNT; dev_id++) {
		delete _px4_accel[dev_id];
		_px4_accel[dev_id] = nullptr;
		delete _px4_gyro[dev_id];
		_px4_gyro[dev_id] = nullptr;
		delete _px4_mag[dev_id];
		_px4_mag[dev_id] = nullptr;
	}
}

int WBotMainDriver::Start()
{
	uint32_t ret = list_tty(this->_serial_name);

	PX4_INFO("list tty ret=%d\n",ret);
	for (uint32_t n = 0; n < TOTAL_SERIAL_COUNT; n++)
	{
		// skip empty device slots
		if (this->_serial_name[n][0] == '\0') {
			_device_connected[n] = false;
			continue;
		}

		this->_serial_fd[n] = open_serial_fd(this->_serial_name[n]);
		if ( this->_serial_fd[n] < 0 )
		{
			_device_connected[n] = false;  // 标记为未连接
			printf("open serial=%d failed\n", n);
		}
		else
		{
			_device_connected[n] = true;
			PX4_INFO("start serial=%s\n", this->_serial_name[n]);
		}

	}

	ScheduleOnInterval(5000_us); // 5ms interval
	return PX4_OK;
}

void WBotMainDriver::handle_device_disconnect(uint8_t dev_id) {
    _device_connected[dev_id] = false;
    _last_disconnect_time[dev_id] = hrt_absolute_time();
    _disconnect_count[dev_id]++;

    PX4_WARN("Device %d disconnected, disconnect count: %d", dev_id, _disconnect_count[dev_id]);

    // 关闭当前的文件描述符
    if (_serial_fd[dev_id] >= 0) {
        close(_serial_fd[dev_id]);
    }
}

uint32_t WBotMainDriver::check_update(void)
{
// PX4_INFO("wbot_main_driver dev-id=%d RunForOne running\n",dev_id);
	bool updated;
	uint32_t dev_id;
	uint32_t cmd_size=3; // 3字节对应 ： 2字节包头 + 1字节长度
	orb_check(_wbot_moto_sub, &updated);  // 检查订阅的 topic 是否有新数据
	if (updated) {
		// PX4_INFO("MOTOR Control update\n");
		struct wbot_ctrl_moto_s data;
		orb_copy(ORB_ID(wbot_ctrl_moto), _wbot_moto_sub, &data);
		// PX4_INFO("dev Got new data: %i %i %i %i %i %i %i %i",
		// 	data.speed[0], data.speed[1],data.speed[2],data.speed[3],data.speed[4],data.speed[5],data.speed[6],data.speed[7]);
		for (dev_id = 0; dev_id < TOTAL_SERIAL_COUNT; dev_id++)
		{
			for ( int i2c_index = 0; i2c_index<4 ; i2c_index++)
			{
				uint8_t speed = data.speed[i2c_index + dev_id*4];
				uint8_t direction = data.direction[i2c_index + dev_id*4];

				McuCmdHelper::set_motor_cmd( &send_cache[dev_id][cmd_size + i2c_index*2] , speed, direction, i2c_index);
				// printf("dev_id = %d,speed =%d ,i2c_index=%d\n",dev_id,speed,i2c_index);
			}
		}
		cmd_size += 2*4;
	}

	orb_check(_wbot_led_sub, &updated);  // 检查订阅的 topic 是否有新数据
	if (updated) {
		PX4_INFO("LED Control update\n");
		struct wbot_ctrl_led_s data;
		orb_copy(ORB_ID(wbot_ctrl_led), _wbot_led_sub, &data);
		for (dev_id = 0; dev_id < TOTAL_SERIAL_COUNT; dev_id++)
		{
			if ( data.led_id == 0 )
			{
				McuCmdHelper::set_led_value( &send_cache[dev_id][cmd_size] , data.light_value );
				PX4_INFO("set LED ,light_value = %d\n",data.light_value);
			}
			if ( data.led_id == 1 )
			{
				McuCmdHelper::set_reboot_value( &send_cache[dev_id][cmd_size] , data.light_value );
				PX4_INFO("reboot MCU\n");
			}

		}
		cmd_size += 2;

	}
	if ( cmd_size >= 3 )
	{
		for (dev_id = 0; dev_id < TOTAL_SERIAL_COUNT; dev_id++)
		{
			send_cache[dev_id][0] = 0x5a;
			send_cache[dev_id][1] = 0x5b;
			send_cache[dev_id][2] = cmd_size;
			uint32_t crc_calc = wbot_crc32(send_cache[dev_id], cmd_size);
			memcpy( &send_cache[dev_id][cmd_size], &crc_calc, sizeof(uint32_t));
			// printf("dev_id (%d)= \n", dev_id );
			// for(uint32_t n = 0; n < cmd_size; n++)
			// {
			// 	printf("0x%02x,", send_cache[dev_id][n]);
			// }
			// printf("0x%x\n",crc_calc);
		}


		cmd_size += sizeof(uint32_t);
	} else {
		for (dev_id = 0; dev_id < TOTAL_SERIAL_COUNT; dev_id++)
			send_cache[dev_id][0] = 0;
	}
	return cmd_size;

}

void WBotMainDriver::RunForOne(uint32_t dev_id, uint32_t cmd_size)
{
	// 检查设备是否已连接
	if (!_device_connected[dev_id])  {
		// printf("Device %d not connected\n", dev_id);
		// 尝试重连设备
		if (attempt_reconnect(dev_id)) {
			// printf("Device %d reconnected\n", dev_id);
		} else {
			// printf("Device %d reconnect failed\n", dev_id);
		return;
		}
		return;
	}

	// TODO: add cmd
	int write_length = ::write(this->_serial_fd[dev_id], send_cache[dev_id], cmd_size);
	// printf("dev = %d writting, write_length = %d ",dev_id,write_length , cmd_size);
	// for (int n = 0; n < write_length; n++)
	// {
	// 	printf(" 0x%02x, ", send_cache[dev_id][n]);
	// 	if ( (n+1) % 16 == 0 ) {
	// 		printf("\n");
	// 	}
	// }
	//printf("\n");
	if ( write_length != (int)cmd_size) {
		PX4_WARN("wbot main can't write , dev id=%i", dev_id);
		handle_device_disconnect(dev_id);
		return;
	}

	int read_length = read_full_packet(this->_serial_fd[dev_id], recv_cache);
	if ( read_length <= 0 )
	{
		PX4_WARN("wbot main can't read , dev id=%i, ret=%d fd=%d", dev_id, read_length, this->_serial_fd[dev_id]);
	 	return;
	} else {
		//printf("read data from %d mcu:\n",dev_id);
		// for (int n = 0; n < read_length; n++)
		// {
		// 	printf(" 0x%02x, ", recv_cache[n]);
		// 	if ( (n+1) % 16 == 0 ) {
		// 		printf("\n");
		// 	}
		// }
		// printf("\n");
	}

	_now = hrt_absolute_time();
	int ret = parse_mcu_data(dev_id, recv_cache);
	//PX4_INFO("parse_mcu_data: %d\n",ret);

	switch (ret)
	{
	case -2:
		perf_count(_bad_packhead_perf);
		break;
	case -4:
		perf_count(_bad_packtail_perf);
		break;
	case -5:
		perf_count(_bad_crc_err_perf);
		break;
	default:
		perf_count(_right_perf);
		break;
	}
}

bool WBotMainDriver::attempt_reconnect(uint8_t dev_id) {
    // 检查是否到了重连时间
    if (hrt_elapsed_time(&_last_disconnect_time[dev_id]) < RECONNECT_INTERVAL_US) {
	// printf("[WBotMainDriver] %s: %s: %s\n", __FUNCTION__, "重连间隔未到", "取消重连");
        return false;
    }
//     printf("[WBotMainDriver] Attempting to reconnect to device %d\n", dev_id);

    uint32_t ret = list_tty(this->_serial_name);

	PX4_INFO("list tty ret=%d\n",ret);
	for (uint32_t n = 0; n < 2; n++)
	{
		if (!_device_connected[n])
		{
			this->_serial_fd[n] = open_serial_fd(this->_serial_name[n]);
			if ( this->_serial_fd[n] < 0 )
			{
				_device_connected[n] = false;  // 标记为未连接
				_last_disconnect_time[dev_id] = hrt_absolute_time();
				printf("open serial=%d failed\n", n);
				return false;
			}
			else
			{
				_device_connected[n] = true;
				PX4_INFO("start serial=%s\n", this->_serial_name[n]);
				return true;
			}
		}
	}
    return false;
}


bool WBotMainDriver::parse_motor_data(uint8_t dev_id, uint8_t *data, uint32_t moto_index, uint32_t len)
{
	if  ( len != MOTOR_DATA_SIZE || ( moto_index >= MOTOR_MAX_NUM ) )
	{
		PX4_DEBUG("parse motor error spi addr=%i motor_index=%i buf_len=%i",
			dev_id, moto_index, len
			);
		return false;
	}

	// TODO only report data[0] to upper computer

	return true;
}

int WBotMainDriver::parse_mcu_data(uint8_t dev_id, uint8_t *data) {

    constexpr uint32_t packet_head_size = 5 ;
    constexpr uint32_t sensor_data_head_size = 3 ;

    if (!data) return -1; // 缓冲区太小

    // 检查包头
    if (data[0] != 0x5a || data[1] != 0x5b || data[2] != 0x5c || data[3] != 0x5d ) return -2;

    uint32_t total_len = data[4]; // copy_data 填写的总长度
    if (total_len < 11 ) return -3; // 长度非法

    // 检查包尾
    if (data[total_len-6] != 0xa5 || data[total_len-5] != 0xa5) return -4;

    // CRC 校验
    uint32_t crc_recv = data[total_len-4] | (data[total_len-3]<<8) |
                        (data[total_len-2]<<16) | (data[total_len-1]<<24);
    uint32_t crc_calc = wbot_crc32(data, total_len-4);

    if (crc_recv != crc_calc)
    {
	// PX4_INFO("crc_recv = 0x%04x ,crc_calc = 0x%04x", crc_recv, crc_calc);
	return -5;
    }

    // 解析有效数据
    uint32_t index = packet_head_size; // 跳过包头 字段
    while (index + packet_head_size <= total_len - 6) { // skip 包头
        uint16_t data_len = data[index] | (data[index+1] << 8);
        uint8_t tag = data[index+2];
        index += sensor_data_head_size;  // index is the  real data offset now

        if (index + data_len > total_len - 6) {
		PX4_ERR("index,data_len= %i %i, total_len-6=%i\n", index, data_len, total_len-6);
		return -6; // 数据越界
	}

        // 处理数据
        PX4_DEBUG("TAG %02X, LEN %d, DATA:", tag, data_len);
        for (uint16_t i = 0; i < data_len; i++) {
            PX4_DEBUG(" %02X", data[index + i]);
        }
	PX4_DEBUG("\n");

	switch (tag)
	{
	case WBOT_SDEV_TAG_IMU:
	{
		if ( !parse_imu_data(dev_id, &data[index], data_len) )
		{
			PX4_DEBUG("wbot main driver parse imu data error");
		}
		break;
	}
	case WBOT_SDEV_TAG_MS5837:
	{
		if ( !parse_ms5837_data(dev_id, &data[index], data_len) )
		{
			PX4_DEBUG("wbot main driver parse ms5837 data error");
		}
		break;
	}
	case WBOT_SDEV_TAG_MOTO0:
		if (!parse_motor_data(dev_id,  &data[index], MOTOR_INDEX_0, data_len))
		{
			PX4_DEBUG("motor %d data recv error",MOTOR_INDEX_0);
		}
		break;
	case WBOT_SDEV_TAG_MOTO1:
		if (!parse_motor_data(dev_id, &data[index], MOTOR_INDEX_1, data_len))
		{
			PX4_DEBUG("motor %d data recv error",MOTOR_INDEX_1);
		}
		break;
	case WBOT_SDEV_TAG_MOTO2:
		if (!parse_motor_data(dev_id,  &data[index], MOTOR_INDEX_2, data_len))
		{
			PX4_DEBUG("motor %d data recv error ",MOTOR_INDEX_2);
		}
		break;
	case WBOT_SDEV_TAG_MOTO3:
		if (!parse_motor_data(dev_id, &data[index], MOTOR_INDEX_3, data_len))
		{
			PX4_DEBUG("motor %d data recv error",MOTOR_INDEX_3);
		}
		break;
	default:
		break;
	}

        index += data_len;
    }

    return 0; // 成功
}


bool WBotMainDriver::parse_ms5837_data(uint8_t dev_id, uint8_t *data, uint32_t len)
{
	if (len != 8) {
		PX4_INFO("ms5837_data != 8, len = %d", len);
		return false;
	}

	uint32_t pressure_raw = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	uint32_t temperature_raw = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);

	float temperature_celsius = temperature_raw / 100.0;
	float pressure_mbar = pressure_raw / 10.0;
	const float pressure_pa = pressure_mbar * 100.0f;

	PX4_DEBUG("temperature = %f, pressure_mbar = %f", (double)temperature_celsius, (double)pressure_mbar);

	// 由压力计算淡水深度并发布（每个MCU独立实例）
	constexpr float kFreshwaterDensity = 1000.0f;   // kg/m^3
	constexpr float kGravity = 9.80665f;            // m/s^2
	constexpr float kSurfacePressurePa = 1013.25f * 100.0f; // Pa

	float depth_m = (pressure_pa - kSurfacePressurePa) / (kFreshwaterDensity * kGravity);
	if (depth_m < 0.0f) {
		depth_m = 0.0f;
	}

	water_depth_s depth_data{};
	depth_data.timestamp = _now;
	depth_data.device_id = 14123500 + dev_id;
	depth_data.pressure_mbar = pressure_mbar;
	depth_data.temperature_celsius = temperature_celsius;
	depth_data.depth_m = depth_m;

	// publish per MCU instance
	if (dev_id < TOTAL_SERIAL_COUNT) {
		_water_depth_pub[dev_id].publish(depth_data);
	}


	return true;
}

bool WBotMainDriver::parse_imu_data(uint8_t dev_id, uint8_t *data, uint32_t len)
{
	uint32_t cnt = len / 7;
	if (len % 7 != 0) {
		return false;
	}

	// 防止 MCU 发来的 FIFO 样本数超过 uORB 消息缓冲区上限，避免数组越界
	const uint32_t max_samples = sensor_accel_fifo_s::MAX_SAMPLES;
	if (cnt > max_samples) {
		PX4_WARN("imu fifo samples (%u) > MAX_SAMPLES (%u), clamping", cnt, max_samples);
		cnt = max_samples;
	}

	int16_t *datax, *datay, *dataz;
	(void)datax;(void)datay;(void)dataz;

	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = _now;
	accel.samples = 0;

	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = _now;
	gyro.samples = 0;

	float temperature_celsius = NAN;  // 存储从 IMU 数据中解析的温度


	for (uint32_t n=0; n<cnt; n++)
	{
		lsm6dsv16x_fifo_out_raw_t f_data;
		lsm6dsv16x_fifo_out_raw_parse(&f_data, &data[7*n]);
		datax = (int16_t *)&f_data.data[0];
		datay = (int16_t *)&f_data.data[2];
		dataz = (int16_t *)&f_data.data[4];
		switch (f_data.tag) {
		case 2: //LSM6DSV16X_XL_NC_TAG:
		{
			// float tx = lsm6dsv16x_from_fs2_to_mg(*datax);
			// float ty = lsm6dsv16x_from_fs2_to_mg(*datay);
			// float tz = lsm6dsv16x_from_fs2_to_mg(*dataz);
			// printf("test accel %f %f %f\n", (double)tx, (double)ty, (double)tz);

			// 处理加速度计数据（原始轴，旋转由PX4Accelerometer处理）
			accel.x[accel.samples] = *datax;
			accel.y[accel.samples] = *datay;
			accel.z[accel.samples] = ((*dataz) == INT16_MIN) ? INT16_MAX : *dataz;
			accel.samples++;

			break;
		}
		case 4: //LSM6DSV16X_TIMESTAMP_TAG:
		{
			int32_t *ts = (int32_t *)&f_data.data[0];
			float_t imu_ts = lsm6dsv16x_from_lsb_to_nsec(*ts)/1000;
			(void)imu_ts;
			// 注：如果需要从 FIFO timestamp 中提取温度，在此处理
			// 当前做法：如果 MCU 单独发送温度数据，应在其他消息中处理
			break;
		}
		case 1: //LSM6DSV16X_GY_NC_TAG:
		{
			// 处理陀螺仪数据（原始轴，旋转由PX4Gyroscope处理）
			gyro.x[gyro.samples] = *datax;
			gyro.y[gyro.samples] = *datay;
			gyro.z[gyro.samples] = ((*dataz) == INT16_MIN) ? INT16_MAX : *dataz;
			gyro.samples++;

          		break;
		}
		case 0xE: //LSM6DSV16X_SENSORHUB_SLAVE0_TAG:
		{
			// 无干扰下， 正常数据应在 400 mG 数量级（几百毫高斯）

			/*
			float x_gauss = lis2mdl_from_lsb_to_mgauss(*datax) / 1000.0f;
			float y_gauss = lis2mdl_from_lsb_to_mgauss(*datay) / 1000.0f;
			float z_gauss = lis2mdl_from_lsb_to_mgauss(*dataz) / 1000.0f;
			*/

			if ( dev_id <= this->_max_dev_id)
			{
				// LIS2MDL: X前、Y左、Z下 -> 机体系X前、Y右、Z下，需要Y取反
				this->_px4_mag[dev_id]->update(this->_now, *datax, -(*datay), *dataz);
			}

			break;
		}
		case 0: //LSM6DSV16X_FIFO_EMPTY:
		{
			break;
		}
		default:
			break;
		}

	}

	if (dev_id <= this->_max_dev_id
	    && (_imu_publish_dev < 0 || dev_id == static_cast<uint32_t>(_imu_publish_dev)))
	{
		if (accel.samples > 0) {
			_px4_accel[dev_id]->set_temperature(temperature_celsius);
			_px4_accel[dev_id]->updateFIFO(accel);
		}

		if (gyro.samples > 0) {
			_px4_gyro[dev_id]->set_temperature(temperature_celsius);
			_px4_gyro[dev_id]->updateFIFO(gyro);
		}

	}



	return true;
}


void WBotMainDriver::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	uint32_t cmd_size = check_update();

	for (uint32_t dev_id = 0; dev_id < TOTAL_SERIAL_COUNT ; dev_id++)
	{
		RunForOne(dev_id, cmd_size);
		// PX4_INFO("wbot_main_driver running\n");
	}
}

int WBotMainDriver::task_spawn(int argc, char *argv[])
{
	int imu_rotation_value = 0;
	int mag_rotation_value = -1;
	int dev_value = 1;
	int imu_publish_dev = -1;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "r:m:d:i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			dev_value = atoi(myoptarg);
			break;
		case 'i':
			imu_publish_dev = atoi(myoptarg);
			break;
		case 'r':
			imu_rotation_value = atoi(myoptarg);
			break;
		case 'm':
			mag_rotation_value = atoi(myoptarg);
			break;

		default:
			print_usage("unknown option");
			return -1;
		}
	}

	if (mag_rotation_value < 0) {
		mag_rotation_value = imu_rotation_value;
	}

	WBotMainDriver *instance = new WBotMainDriver(imu_rotation_value, mag_rotation_value, dev_value,
						   static_cast<int8_t>(imu_publish_dev));

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	int ret = instance->Start();

	if (ret != PX4_OK) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return ret;
	}

	return PX4_OK;
}

int WBotMainDriver::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	if (!strcmp(argv[0], "print_data") || !strcmp(argv[0], "pd")) {
		WBotMainDriver *instance = _object.load();  // 使用.load()方法获取实例
		if (instance == nullptr) {
			PX4_WARN("Not running");
			return -1;
		}

		instance->printSensorData();
		return 0;
	}

	return print_usage("unknown command");
}

int WBotMainDriver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Water Robot Main Driver module.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wbot_main_driver", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 100, "IMU rotation N value", true);
	PRINT_MODULE_USAGE_PARAM_INT('m', 0, 0, 100, "Mag rotation N value (default: same as -r)", true);
	PRINT_MODULE_USAGE_PARAM_INT('d', 0, 0, 1, "max enable dev id", true);
	PRINT_MODULE_USAGE_PARAM_INT('i', -1, -1, 1, "IMU publish dev id (-1: all, 0/1: only one)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("print_data", "Print current sensor data (accel, gyro, attitude) and calibration params");
	PRINT_MODULE_USAGE_COMMAND("pd");  // Short alias for print_data
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void WBotMainDriver::printSensorData()
{
	// 获取并打印加速度计数据
	sensor_accel_s accel_data;
	if (_sensor_accel_sub.copy(&accel_data)) {
		PX4_INFO("sensor_accel: x=%.3f, y=%.3f, z=%.3f (m/s^2)",
		         (double)accel_data.x, (double)accel_data.y, (double)accel_data.z);
	} else {
		PX4_WARN("Failed to get sensor_accel data");
	}

	// 获取并打印陀螺仪数据
	sensor_gyro_s gyro_data;
	if (_sensor_gyro_sub.copy(&gyro_data)) {
		PX4_INFO("sensor_gyro: x=%.3f, y=%.3f, z=%.3f (rad/s)",
		         (double)gyro_data.x, (double)gyro_data.y, (double)gyro_data.z);
	} else {
		PX4_WARN("Failed to get sensor_gyro data");
	}

	// 获取并打印姿态数据
	vehicle_attitude_s attitude_data;
	if (_vehicle_attitude_sub.copy(&attitude_data)) {
		// 将四元数转换为欧拉角 (Roll, Pitch, Yaw)
		matrix::Eulerf euler{matrix::Quatf{attitude_data.q}};
		float roll_deg = math::degrees(euler(0));  // Roll
		float pitch_deg = math::degrees(euler(1)); // Pitch
		float yaw_deg = math::degrees(euler(2));   // Yaw

		PX4_INFO("vehicle_attitude: Roll=%.2f deg, Pitch=%.2f deg, Yaw=%.2f deg",
		         (double)roll_deg, (double)pitch_deg, (double)yaw_deg);
	} else {
		PX4_WARN("Failed to get vehicle_attitude data");
	}

	// 获取并打印校准参数
	// 获取CAL_ACC0_ROT参数
	int32_t cal_acc0_rot = 0;
	int result = param_get(param_find("CAL_ACC0_ROT"), &cal_acc0_rot);
	if (result == 0) {
		PX4_INFO("CAL_ACC0_ROT: %d", cal_acc0_rot);
	} else {
		PX4_WARN("Failed to get CAL_ACC0_ROT parameter");
	}

	// 获取CAL_GYRO0_ROT参数
	int32_t cal_gyro0_rot = 0;
	result = param_get(param_find("CAL_GYRO0_ROT"), &cal_gyro0_rot);
	if (result == 0) {
		PX4_INFO("CAL_GYRO0_ROT: %d", cal_gyro0_rot);
	} else {
		PX4_WARN("Failed to get CAL_GYRO0_ROT parameter");
	}
}
