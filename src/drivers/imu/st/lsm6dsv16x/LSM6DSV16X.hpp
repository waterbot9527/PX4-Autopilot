#pragma once


#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/st_lsm6dsv16x_common/lsm6dsv16x_reg.h>
#include <lib/drivers/st_lis2mdl_common/lis2mdl_reg.h>


class LSM6DSV16X : public device::I2C, public I2CSPIDriver<LSM6DSV16X>
{
public:
    LSM6DSV16X(const I2CSPIDriverConfig &config);
    ~LSM6DSV16X() override;

    static void print_usage();
    void RunImpl();
    int init() override;
    void print_status() override;

private:
    void exit_and_cleanup() override;

    // 传感器配置参数
    // static constexpr float FIFO_SAMPLE_DT{1e6f};  // 采样间隔(us)
    // static constexpr float GYRO_RATE{1};
    // static constexpr float ACCEL_RATE{1};

    // FIFO最大采样数 (受限于FIFO深度和数据结构)
    //static constexpr int32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / 12, 32)};

    struct register_config_t {
        //Register reg;
        uint8_t set_bits{0};
        uint8_t clear_bits{0};
    };

    int probe() override;
    bool Reset();
    bool Configure();
    void ConfigureSampleRate(int sample_rate);


    PX4Accelerometer _px4_accel;
    PX4Gyroscope _px4_gyro;
    PX4Magnetometer _px4_mag;

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

    // Sensor Hub初始化函数
    //bool InitSensorHub();
    // 从设备（LIS2MDL）初始化函数
    //bool InitLIS2MDL();

};
