#include "LSM6DSV16X.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
    return (msb << 8u) | lsb;
}

// 构造函数修改
LSM6DSV16X(const I2CSPIDriverConfig &config) :
    I2C(config),  // 从SPI改为I2C
    I2CSPIDriver(config),
    _px4_accel(get_device_id(), config.rotation),
    _px4_gyro(get_device_id(), config.rotation)
    _px4_mag(get_device_id(), config.rotation)  // 初始化磁力计
{
    ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

LSM6DSV16X::~LSM6DSV16X()
{
    perf_free(_bad_register_perf);
    perf_free(_bad_transfer_perf);
    perf_free(_fifo_empty_perf);
    perf_free(_fifo_overflow_perf);
    perf_free(_fifo_reset_perf);
}

int LSM6DSV16X::init()
{
    int ret = SPI::init();

    if (ret != PX4_OK) {
        DEVICE_DEBUG("SPI::init failed (%i)", ret);
        return ret;
    }

    return Reset() ? 0 : -1;
}

bool LSM6DSV16X::Reset()
{
    _state = STATE::RESET;
    ScheduleClear();
    ScheduleNow();
    return true;
}

void LSM6DSV16X::exit_and_cleanup()
{
    I2CSPIDriverBase::exit_and_cleanup();
}

void LSM6DSV16X::print_status()
{
    I2CSPIDriverBase::print_status();

    PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

    perf_print_counter(_bad_register_perf);
    perf_print_counter(_bad_transfer_perf);
    perf_print_counter(_fifo_empty_perf);
    perf_print_counter(_fifo_overflow_perf);
    perf_print_counter(_fifo_reset_perf);
}

int LSM6DSV16X::probe()
{
    const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

    if (whoami != WHO_AM_I_ID) {
        DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
        return PX4_ERROR;
    }

    return PX4_OK;
}

void LSM6DSV16X::RunImpl()
{
    const hrt_abstime now = hrt_absolute_time();

    switch (_state) {
    case STATE::RESET:
        // 软件复位
        RegisterWrite(Register::CTRL10_C, CTRL10_C_BIT::SW_RESET);
        _reset_timestamp = now;
        _failure_count = 0;
        _state = STATE::WAIT_FOR_RESET;
        ScheduleDelayed(100_ms);
        break;

    case STATE::WAIT_FOR_RESET:
        if (RegisterRead(Register::WHO_AM_I) == WHO_AM_I_ID) {
            _state = STATE::CONFIGURE;
            ScheduleDelayed(100_ms);

        } else {
            if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
                PX4_DEBUG("Reset failed, retrying");
                _state = STATE::RESET;

            } else {
                ScheduleDelayed(10_ms);
            }
        }
        break;

    case STATE::CONFIGURE:
        if (Configure()) {
            _state = STATE::FIFO_READ;
            ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
            FIFOReset();

        } else {
            if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
                PX4_DEBUG("Configure failed, resetting");
                _state = STATE::RESET;

            } else {
                PX4_DEBUG("Configure failed, retrying");
            }

            ScheduleDelayed(100_ms);
        }
        break;

    case STATE::FIFO_READ: {
            hrt_abstime timestamp_sample = now;
            bool success = false;

            // 读取FIFO状态
            const uint8_t fifo_status = RegisterRead(Register::FIFO_STATUS1);
            uint8_t samples = (fifo_status & 0x3F);  // 假设低6位为样本数

            if (fifo_status & (1 << 7)) {  // 溢出标志
                FIFOReset();
                perf_count(_fifo_overflow_perf);

            } else if (samples == 0) {
                perf_count(_fifo_empty_perf);

            } else {
                if (samples > FIFO_MAX_SAMPLES) {
                    FIFOReset();
                    perf_count(_fifo_overflow_perf);

                } else if (samples >= 1) {
                    if (FIFORead(timestamp_sample, samples)) {
                        success = true;
                        if (_failure_count > 0) _failure_count--;
                    }
                }
            }

            if (!success) {
                if (++_failure_count > 10) {
                    Reset();
                    return;
                }
            }

            // 周期性检查配置
            if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
                if (RegisterCheck(_register_cfg[_checked_register])) {
                    _last_config_check_timestamp = now;
                    _checked_register = (_checked_register + 1) % size_register_cfg;

                } else {
                    perf_count(_bad_register_perf);
                    Reset();
                }

            } else {
                // 周期性更新温度
                if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
                    UpdateTemperature();
                    _temperature_update_timestamp = now;
                }
            }
        }
        break;
    }
}

void LSM6DSV16X::ConfigureSampleRate(int sample_rate)
{
    const float min_interval = FIFO_SAMPLE_DT;
    _fifo_empty_interval_us = math::max(roundf((1e6f / sample_rate) / min_interval) * min_interval, min_interval);
    _fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));
    _fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);
}

bool LSM6DSV16X::Configure()
{
    // 配置寄存器
    for (const auto &reg_cfg : _register_cfg) {
        RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
    }

    // 验证配置
    bool success = true;
    for (const auto &reg_cfg : _register_cfg) {
        if (!RegisterCheck(reg_cfg)) success = false;
    }

    if (!InitSensorHub() || !InitLIS2MDL()) {
        PX4_ERR("Sensor Hub or LIS2MDL init failed");
        success = false;
    }

    // 设置量程和缩放因子
    _px4_gyro.set_scale(math::radians(70.f / 1000.f));  // 2000dps对应70mdps/LSB
    _px4_gyro.set_range(math::radians(2000.f));
    _px4_accel.set_scale(0.732f * (CONSTANTS_ONE_G / 1000.f));  // 16G对应0.732mg/LSB
    _px4_accel.set_range(16.f * CONSTANTS_ONE_G);
    _px4_mag.set_scale(0.15f);  // LIS2MDL默认16位模式，0.15μT/LSB
    _px4_mag.set_range(5000.f); // 最大量程±5000μT

    return success;
}

bool LSM6DSV16X::RegisterCheck(const register_config_t &reg_cfg)
{
    const uint8_t reg_value = RegisterRead(reg_cfg.reg);
    bool success = true;

    if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
        PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
        success = false;
    }

    if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
        PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
        success = false;
    }

    return success;
}

uint8_t LSM6DSV16X::RegisterRead(Register reg)
{
    uint8_t data = 0;
    uint8_t reg_addr = static_cast<uint8_t>(reg);

    // I2C读取流程：先发送寄存器地址，再读取数据
    if (transfer(&reg_addr, 1, &data, 1) != PX4_OK) {
        perf_count(_bad_transfer_perf);
    }
    return data;
}

void LSM6DSV16X::RegisterWrite(Register reg, uint8_t value)
{
    uint8_t data[2] = {static_cast<uint8_t>(reg), value};

    // I2C写入流程：发送寄存器地址+数据
    if (transfer(data, 2, nullptr, 0) != PX4_OK) {
        perf_count(_bad_transfer_perf);
    }
}

void LSM6DSV16X::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
    const uint8_t orig_val = RegisterRead(reg);
    uint8_t val = (orig_val & ~clearbits) | setbits;
    if (orig_val != val) RegisterWrite(reg, val);
}

bool LSM6DSV16X::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
    sensor_gyro_fifo_s gyro{};
    gyro.timestamp_sample = timestamp_sample;
    gyro.samples = samples;
    gyro.dt = FIFO_SAMPLE_DT;

    sensor_accel_fifo_s accel{};
    accel.timestamp_sample = timestamp_sample;
    accel.samples = samples;
    accel.dt = FIFO_SAMPLE_DT;

    //todo
    // 读取从设备（LIS2MDL）数据

    for (int i = 0; i < samples; i++) {
        // 读取陀螺仪数据
        struct GyroData {
            uint8_t cmd{static_cast<uint8_t>(Register::OUTX_L_G) | 0x80 | 0x40};  // 自动增量
            uint8_t outx_l;
            uint8_t outx_h;
            uint8_t outy_l;
            uint8_t outy_h;
            uint8_t outz_l;
            uint8_t outz_h;
        } gyro_buf{};

        if (transfer((uint8_t *)&gyro_buf, (uint8_t *)&gyro_buf, sizeof(gyro_buf)) != PX4_OK) {
            perf_count(_bad_transfer_perf);
            return false;
        }
		// 读取陀螺仪+加速度计数据（每次6字节，共samples组）
		uint8_t data[12 * samples] = {0};  // 每组数据： gyro(6字节) + accel(6字节)

		// 从陀螺仪起始寄存器开始连续读取
		if (RegisterBatchRead(Register::OUTX_L_G, data, 12 * samples) != PX4_OK) {
			return false;
		}

		// 解析数据
		for (int i = 0; i < samples; i++) {
			int idx = i * 12;  // 每组数据偏移量

			// 陀螺仪数据（OUTX_L_G到OUTZ_H_G）
			int16_t gx = combine(data[idx+1], data[idx]);
			int16_t gy = combine(data[idx+3], data[idx+2]);
			int16_t gz = combine(data[idx+5], data[idx+4]);

			// 加速度计数据（OUTX_L_XL到OUTZ_H_XL）
			int16_t ax = combine(data[idx+7], data[idx+6]);
			int16_t ay = combine(data[idx+9], data[idx+8]);
			int16_t az = combine(data[idx+11], data[idx+10]);

			// 填充FIFO
			gyro.x[i] = gx;
			gyro.y[i] = gy;
			gyro.z[i] = gz;

			accel.x[i] = ax;
			accel.y[i] = ay;
			accel.z[i] = az;
		}

		// 发布数据
		gyro.timestamp = hrt_absolute_time();
		_px4_gyro.publish(gyro);

		accel.timestamp = hrt_absolute_time();
		_px4_accel.publish(accel);

		return true;
    }
}


void LSM6DSV16X::FIFOReset()
{
    RegisterSetAndClearBits(Register::FIFO_CTRL1, 0x00, 0x3F);  // 停止FIFO
    RegisterSetAndClearBits(Register::FIFO_CTRL1, FIFO_CTRL1_BIT::FMODE_CONTINUOUS, 0);  // 重启FIFO
    perf_count(_fifo_reset_perf);
}

void LSM6DSV16X::UpdateTemperature()
{
    // 读取温度数据 (需根据datasheet确认转换公式)
    const uint8_t t_l = RegisterRead(Register::OUT_TEMP_L);
    const uint8_t t_h = RegisterRead(Register::OUT_TEMP_H);
    int16_t temp_raw = combine(t_h, t_l);

    // 温度转换 (示例公式)
    float temperature = 25.0f + (temp_raw / 256.0f);
    _px4_gyro.set_temperature(temperature);
    _px4_accel.set_temperature(temperature);
}


bool LSM6DSV16X::InitLIS2MDL()
{
    //todo :
    return true;
}
