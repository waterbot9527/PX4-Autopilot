#pragma once

#include "ST_LSM6DSV16X_Registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_LSM6DSV16X;

class LSM6DSV16X : public device::I2C, public I2CSPIDriver<LSM6DSV16X>
{
public:
    LSM6DSV16X(const I2CSPIDriverConfig &config);
    ~LSM6DSV16X() override;

    static void print_usage();
    void RunImpl() override;
    int init() override;
    void print_status() override;

private:
    void exit_and_cleanup() override;

    // 传感器配置参数
    static constexpr float FIFO_SAMPLE_DT{1e6f / G_ODR};  // 采样间隔(us)
    static constexpr float GYRO_RATE{G_ODR};
    static constexpr float ACCEL_RATE{XL_ODR};

    // FIFO最大采样数 (受限于FIFO深度和数据结构)
    static constexpr int32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / 12, 32)};

    struct register_config_t {
        Register reg;
        uint8_t set_bits{0};
        uint8_t clear_bits{0};
    };

    int probe() override;
    bool Reset();
    bool Configure();
    void ConfigureSampleRate(int sample_rate);
    bool RegisterCheck(const register_config_t &reg_cfg);

    uint8_t RegisterRead(Register reg);
    void RegisterWrite(Register reg, uint8_t value);
    void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

    bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
    void FIFOReset();
    void UpdateTemperature();

    PX4Accelerometer _px4_accel;
    PX4Gyroscope _px4_gyro;

    // 性能计数器
    perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
    perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
    perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
    perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
    perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};

    hrt_abstime _reset_timestamp{0};
    hrt_abstime _last_config_check_timestamp{0};
    hrt_abstime _temperature_update_timestamp{0};
    int _failure_count{0};

    enum class STATE : uint8_t {
        RESET,
        WAIT_FOR_RESET,
        CONFIGURE,
        FIFO_READ,
    } _state{STATE::RESET};

    uint16_t _fifo_empty_interval_us{625};  // 默认1600Hz采样间隔
    int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

    uint8_t _checked_register{0};
    static constexpr uint8_t size_register_cfg{6};
    register_config_t _register_cfg[size_register_cfg] {
        // 寄存器配置 (需根据datasheet确认)
        { Register::CTRL1_G,  CTRL1_G_BIT::ODR_G_1600HZ | CTRL1_G_BIT::FS_G_2000DPS, 0 },
        { Register::CTRL3_XL, CTRL3_XL_BIT::ODR_XL_1600HZ | CTRL3_XL_BIT::FS_XL_16G, 0 },
        { Register::CTRL10_C, CTRL10_C_BIT::IF_ADD_INC, CTRL10_C_BIT::SW_RESET },
        { Register::FIFO_CTRL1, FIFO_CTRL1_BIT::FMODE_CONTINUOUS, 0 },
        { Register::CTRL12_C, 0x03, 0 },  // 使能陀螺仪和加速度计
        { Register::STATUS_REG, 0, 0 },   // 状态寄存器检查
    };

    PX4Magnetometer _px4_mag;  // 磁力计对象

    // Sensor Hub初始化函数
    bool InitSensorHub();
    // 从设备（LIS2MDL）初始化函数
    bool InitLIS2MDL();
    // 读取从设备（LIS2MDL）数据
    bool ReadLIS2MDL(const hrt_abstime &timestamp);
};
