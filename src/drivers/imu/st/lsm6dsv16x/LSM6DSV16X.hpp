#pragma once


#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/st_lsm6dsv16x_common/lsm6dsv16x_reg.h>
#include <lib/drivers/st_lis2mdl_common/lis2mdl_reg.h>
#include <time.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/sensor_mag.h>


class LSM6DSV16X : public device::SPI, public I2CSPIDriver<LSM6DSV16X>
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

    // Sensor configuration
    // static constexpr float FIFO_SAMPLE_DT{1e6f};  // sampling interval (us)
    // static constexpr float GYRO_RATE{1};
    // static constexpr float ACCEL_RATE{1};

    // Maximum FIFO samples (limited by FIFO depth and data structure size)
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

    // uORB publishers
    uORB::PublicationMulti<sensor_accel_fifo_s> _accel_fifo_pub{ORB_ID(sensor_accel_fifo)};
    uORB::PublicationMulti<sensor_gyro_fifo_s> _gyro_fifo_pub{ORB_ID(sensor_gyro_fifo)};
    uORB::PublicationMulti<sensor_mag_s> _mag_pub{ORB_ID(sensor_mag)};

    // FIFO batch data buffers
    sensor_accel_fifo_s _accel_fifo_data{};
    sensor_gyro_fifo_s _gyro_fifo_data{};
    sensor_mag_s _mag_data{};

    // FIFO sample counters
    uint8_t _accel_samples{0};
    uint8_t _gyro_samples{0};
    uint8_t _mag_samples{0};

    /* SPI and sensor hub device contexts */
    static stmdev_ctx_t lsm6dsv16x_ctx;
    static stmdev_ctx_t lis2mdl_ctx;
    /* Platform SPI callbacks and sensor hub I2C proxy functions */
    static int platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
    static int platform_write( void *handle,uint8_t reg, uint8_t *bufp, uint16_t len);
    static int lsm6dsv16x_write_lis2mdl_cx(void *ctx, uint8_t reg, uint8_t *data, uint16_t len);
    static int lsm6dsv16x_read_lis2mdl_cx(void *ctx, uint8_t reg, uint8_t *data, uint16_t len);
    static int lsm6dsv16x_write_target_cx(void *ctx, uint8_t i2c_add, uint8_t reg,
                                            const uint8_t *data, uint16_t len);
    static int lsm6dsv16x_read_target_cx(void *ctx, uint8_t i2c_add, uint8_t reg,
                                            uint8_t *data, uint16_t len);
    static void platform_delay(uint32_t ms);


    PX4Accelerometer _px4_accel;
    PX4Gyroscope _px4_gyro;
    PX4Magnetometer _px4_mag;

    // Performance counters
    perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
    perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
    perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
    perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
    perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
    perf_counter_t _accel_pub_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": accel updates")};
    perf_counter_t _gyro_pub_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": gyro updates")};
    perf_counter_t _fifo_read_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": FIFO reads")};

    uint32_t _total_samples_published{0};

    hrt_abstime _reset_timestamp{0};
    hrt_abstime _last_config_check_timestamp{0};
    hrt_abstime _temperature_update_timestamp{0};
    hrt_abstime _last_timestamp{0};
    int _failure_count{0};

    enum class STATE : uint8_t {
        RESET,
        WAIT_FOR_RESET,
        CONFIGURE,
        FIFO_READ,
    } _state{STATE::RESET};

    // Sensor Hub initialization
    // bool InitSensorHub();
    // Slave device (LIS2MDL) initialization
    bool InitLIS2MDL();

};
