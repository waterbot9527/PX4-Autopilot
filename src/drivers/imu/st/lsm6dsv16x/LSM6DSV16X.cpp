#include "LSM6DSV16X.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
    return (msb << 8u) | lsb;
}

// 构造函数修改
LSM6DSV16X::LSM6DSV16X(const I2CSPIDriverConfig &config) :
    I2C(config),  // 从SPI改为I2C
    I2CSPIDriver(config),
    _px4_accel(get_device_id(), config.rotation),
    _px4_gyro(get_device_id(), config.rotation),
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
    int ret = I2C::init();

    if (ret != PX4_OK) {
        DEVICE_DEBUG("I2C::init failed (%i)", ret);
        return ret;
    }
    DEVICE_DEBUG("I2C::init seccess (%i)", ret);

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


    perf_print_counter(_bad_register_perf);
    perf_print_counter(_bad_transfer_perf);
    perf_print_counter(_fifo_empty_perf);
    perf_print_counter(_fifo_overflow_perf);
    perf_print_counter(_fifo_reset_perf);
}

int LSM6DSV16X::probe()
{
    //根据单片机代码来 probe

    // PX4_INFO("Using bus %d, address 0x%02X", get_device_bus(), get_device_address());

    // uint8_t whoami = 0;

    // // 发送 1 字节寄存器地址，然后读 1 字节
    // // Platform_read(,);

    // PX4_INFO("WHO_AM_I = 0x%02X", whoami);

    // if (whoami != 0x70) {
    //     PX4_ERR("Unexpected WHO_AM_I 0x%02X", whoami);
    //     return PX4_ERROR;
    // }
    // PX4_INFO("READ LSM6DSV16X WHO_AM_I SUCCESS");

    return PX4_OK;
}

void LSM6DSV16X::RunImpl()
{
    //const hrt_abstime now = hrt_absolute_time();

}

void LSM6DSV16X::ConfigureSampleRate(int sample_rate)
{

}

bool LSM6DSV16X::Configure()
{


    // 验证配置
    bool success = true;

    return success;
}



// bool LSM6DSV16X::InitLIS2MDL()
// {
//     //todo :
//     return true;
// }

// int32_t LSM6DSV16X::Platform_read(uint8_t reg, uint8_t *bufp, uint16_t len)
// {
//     // 先写入要读取的寄存器地址
//     uint8_t Register = reg;

//     uint8_t ret = transfer(&Register, sizeof(Register), bufp, len);
//     if (ret != PX4_OK) {
//         PX4_ERR("I2C transfer failed");
//         return PX4_ERROR;
//     }
//     return PX4_OK;
// }
