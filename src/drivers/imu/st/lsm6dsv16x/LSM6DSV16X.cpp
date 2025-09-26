#include "LSM6DSV16X.hpp"

using namespace time_literals;

stmdev_ctx_t LSM6DSV16X::lsm6dsv16x_ctx = {};
stmdev_ctx_t LSM6DSV16X::lis2mdl_ctx = {};

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
        PX4_INFO("I2C::init failed (%i)", ret);
        return ret;
    }
    PX4_INFO("I2C::init seccess (%i)", ret);

    return Reset() ? 0 : -1;
}

bool LSM6DSV16X::Reset()
{
    _state = STATE::RESET;
    ScheduleClear();

	uint32_t interva_delay_us = 2*1000;
	ScheduleOnInterval(interva_delay_us, interva_delay_us);
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

    PX4_INFO("Using bus %d", get_device_bus());
        /* 初始化设备上下文 */
    lsm6dsv16x_ctx.write_reg = LSM6DSV16X::platform_write;
    lsm6dsv16x_ctx.read_reg = LSM6DSV16X::platform_read;
    lsm6dsv16x_ctx.mdelay = LSM6DSV16X::platform_delay;
    lsm6dsv16x_ctx.handle =this;

    lis2mdl_ctx.read_reg = LSM6DSV16X::lsm6dsv16x_read_lis2mdl_cx;
    lis2mdl_ctx.write_reg = LSM6DSV16X::lsm6dsv16x_write_lis2mdl_cx;
    lis2mdl_ctx.mdelay = LSM6DSV16X::platform_delay;
    lis2mdl_ctx.handle = this;

    uint8_t whoami = 0;

    // 发送 1 字节寄存器地址，然后读 1 字节
    lsm6dsv16x_device_id_get(&lsm6dsv16x_ctx, &whoami);

    PX4_INFO("WHO_AM_I = 0x%02X", whoami);

    if (whoami != 0x70) {
        PX4_ERR("Unexpected WHO_AM_I 0x%02X", whoami);
        return PX4_ERROR;
    }
    PX4_INFO("READ LSM6DSV16X WHO_AM_I SUCCESS");

    return PX4_OK;
}

void LSM6DSV16X::RunImpl()
{
    //const hrt_abstime now = hrt_absolute_time();

    PX4_INFO("running");

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



bool LSM6DSV16X::InitLIS2MDL()
{
    //todo :
    bool success = true;
    return success;
}

int LSM6DSV16X::platform_read(void *handle,uint8_t reg, uint8_t *bufp, uint16_t len)
{
    LSM6DSV16X *dev = reinterpret_cast<LSM6DSV16X *>(handle);

    uint8_t Register = reg;
    int ret = dev->transfer(&Register, sizeof(Register), bufp, len);
    if (ret != PX4_OK) {
        PX4_ERR("I2C traregnsfer failed reg=0x%02X len=%u", reg,len);
        return PX4_ERROR;
    }
    return PX4_OK;
}

/* I2C写函数 */
int LSM6DSV16X::platform_write(void *handle,uint8_t reg, uint8_t *bufp, uint16_t len)
{
    // 把 handle 转成对象指针
    LSM6DSV16X *dev = reinterpret_cast<LSM6DSV16X *>(handle);

    // 构造一个临时 buffer，把寄存器地址和数据拼在一起
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], bufp, len);

    // 调用对象的 I2C 写函数
    int ret = dev->transfer(buffer, sizeof(buffer), nullptr, 0);
    if (ret != PX4_OK) {
        PX4_ERR("I2C traregnsfer failed reg=0x%02X len=%u", reg,len);
        return PX4_ERROR;
    }
    return PX4_OK;
}



/* LIS2MDL写函数 */
int LSM6DSV16X::lsm6dsv16x_write_lis2mdl_cx(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
  return lsm6dsv16x_write_target_cx(ctx, LIS2MDL_I2C_ADD, reg, data, len);
}

int LSM6DSV16X::lsm6dsv16x_write_target_cx(void *ctx, uint8_t i2c_add, uint8_t reg,
                                          const uint8_t *data, uint16_t len)
{
  int16_t raw_xl[3];
  int32_t ret;
  lsm6dsv16x_data_ready_t drdy;
  lsm6dsv16x_status_master_t master_status;
  lsm6dsv16x_sh_cfg_write_t sh_cfg_write;

  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_write.slv0_add = (i2c_add & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = lsm6dsv16x_sh_cfg_write(&lsm6dsv16x_ctx, &sh_cfg_write);

  /* Disable accelerometer. */
  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);

  /* Enable I2C Master. */
  lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_120Hz);

  /* Wait Sensor Hub operation flag set. */
  lsm6dsv16x_acceleration_raw_get(&lsm6dsv16x_ctx, raw_xl);

  do {
    px4_usleep(1000);
    lsm6dsv16x_flag_data_ready_get(&lsm6dsv16x_ctx, &drdy);
  } while (!drdy.drdy_xl);

  do {
    px4_usleep(1000);
    lsm6dsv16x_sh_status_get(&lsm6dsv16x_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_DISABLE);
  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);

  return ret;
}

/* LIS2MDL读函数 */
int LSM6DSV16X::lsm6dsv16x_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                          uint8_t *data, uint16_t len)
{
  return lsm6dsv16x_read_target_cx(ctx, LIS2MDL_I2C_ADD, reg, data, len);
}

int LSM6DSV16X::lsm6dsv16x_read_target_cx(void *ctx, uint8_t i2c_add, uint8_t reg,
                                         uint8_t *data, uint16_t len)
{
  lsm6dsv16x_sh_cfg_read_t sh_cfg_read;
  int16_t raw_xl[3];
  int32_t ret;
  lsm6dsv16x_data_ready_t drdy;
  lsm6dsv16x_status_master_t master_status;

  /* Disable accelerometer. */
  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);

  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_read.slv_add = (i2c_add & 0xFEU) >> 1; /* 7bit I2C address */
  printf("slv_add: 0x%02X",sh_cfg_read.slv_add);
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dsv16x_sh_slv_cfg_read(&lsm6dsv16x_ctx, 0, &sh_cfg_read);
  printf("lsm6dsv16x_sh_slv_cfg_read = %d\n", ret);

  ret = lsm6dsv16x_sh_slave_connected_set(&lsm6dsv16x_ctx, LSM6DSV16X_SLV_0);
  printf("lsm6dsv16x_sh_slave_connected_set = %d\n", ret);

  /* Enable I2C Master and I2C master. */
  ret = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
  printf("lsm6dsv16x_sh_master_set = %d\n", ret);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_120Hz);
  printf("lsm6dsv16x_xl_data_rate_set = %d\n", ret);

  /* Wait Sensor Hub operation flag set. */
  ret = lsm6dsv16x_acceleration_raw_get(&lsm6dsv16x_ctx, raw_xl);
  printf("lsm6dsv16x_acceleration_raw_get = %d\n", ret);

  do {
    px4_usleep(1000);
    lsm6dsv16x_flag_data_ready_get(&lsm6dsv16x_ctx, &drdy);
  } while (!drdy.drdy_xl);

  do {
    lsm6dsv16x_sh_status_get(&lsm6dsv16x_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_DISABLE);
  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
  /* Read SensorHub registers. */
  ret = lsm6dsv16x_sh_read_data_raw_get(&lsm6dsv16x_ctx, data, len);
  printf("lsm6dsv16x_sh_read_data_raw_get = %d data=%d \n",ret, data[0]);

  return ret;
}

/* 延迟函数 */
void LSM6DSV16X::platform_delay(uint32_t ms)
{
    px4_usleep(ms);
}


