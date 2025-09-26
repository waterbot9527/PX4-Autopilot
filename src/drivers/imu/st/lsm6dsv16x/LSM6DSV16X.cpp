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

	uint32_t interva_delay_us = 50*1000;
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

    lsm6dsv16x_reset_t rst;
    lsm6dsv16x_sh_cfg_read_t sh_cfg_read;
    uint8_t lis2mdl_rst;
    // lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
    uint8_t whoamI = 0;

    lsm6dsv16x_device_id_get(&lsm6dsv16x_ctx, &whoamI);
    if (whoamI != LSM6DSV16X_ID) {
        DEVICE_LOG("LSM6DSV16X not found! ID: 0x%02X", whoamI);
        return PX4_ERROR;
    }
    PX4_INFO("LSM6DSV16X whoamI read : 0x%02X \n", whoamI);

    /* 恢复默认配置 */
    lsm6dsv16x_reset_set(&lsm6dsv16x_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
    do {
        lsm6dsv16x_reset_get(&lsm6dsv16x_ctx, &rst);
    } while (rst != LSM6DSV16X_READY);
    PX4_INFO("lsm6dsv16x reset set \n");

    /* 使能块数据更新 */
    lsm6dsv16x_block_data_update_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);

    /* 设置加速度计满量程 */
    lsm6dsv16x_xl_full_scale_set(&lsm6dsv16x_ctx, LSM6DSV16X_2g);

    uint8_t func_cfg_access = 0x01; // 0x01表示允许访问hub配置寄存器
    lsm6dsv16x_write_reg(&lsm6dsv16x_ctx, LSM6DSV16X_FUNC_CFG_ACCESS, &func_cfg_access, 1);
    platform_delay(10); // 等待配置生效

    /* 配置LIS2MDL */
    lis2mdl_device_id_get(&lis2mdl_ctx, &whoamI);
    if (whoamI != LIS2MDL_ID) {
        DEVICE_LOG("LIS2MDL not found! ID: 0x%02X\n", whoamI);
        return PX4_ERROR;
    }
    DEVICE_LOG("LIS2MDL whoamI read : 0x%02X \n", whoamI);

    /* 恢复默认配置 */
    lis2mdl_reset_set(&lis2mdl_ctx, PROPERTY_ENABLE);
    do {
        lis2mdl_reset_get(&lis2mdl_ctx, &lis2mdl_rst);
    } while (lis2mdl_rst);

    lis2mdl_block_data_update_set(&lis2mdl_ctx, PROPERTY_ENABLE);
    lis2mdl_offset_temp_comp_set(&lis2mdl_ctx, PROPERTY_ENABLE);
    lis2mdl_operating_mode_set(&lis2mdl_ctx, LIS2MDL_CONTINUOUS_MODE);
    lis2mdl_data_rate_set(&lis2mdl_ctx, LIS2MDL_ODR_100Hz);

    /*
    * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
    * stored in FIFO) to FIFO_WATERMARK samples
    */
    const uint8_t FIFO_WATERMARK = 255;
    lsm6dsv16x_fifo_watermark_set(&lsm6dsv16x_ctx, FIFO_WATERMARK);

    /* Set FIFO batch XL/Gyro ODR to 60Hz */
    lsm6dsv16x_fifo_xl_batch_set(&lsm6dsv16x_ctx, (lsm6dsv16x_fifo_xl_batch_t)LSM6DSV16X_XL_BATCHED_AT_1920Hz);
    lsm6dsv16x_fifo_gy_batch_set(&lsm6dsv16x_ctx, (lsm6dsv16x_fifo_gy_batch_t)LSM6DSV16X_XL_BATCHED_AT_1920Hz);


    /* Set FIFO mode to Stream mode (aka Continuous Mode) */
    lsm6dsv16x_fifo_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_STREAM_MODE);

    // pin_int.fifo_th = PROPERTY_ENABLE;
    // lsm6dsv16x_pin_int1_route_set(&lsm6dsv16x_ctx, &pin_int);
    //lsm6dsv16x_pin_int2_route_set(&lsm6dsv16x_ctx, &pin_int);


    /* Set Output Data Rate */
    lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_1920Hz);
    lsm6dsv16x_gy_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_1920Hz);
    lsm6dsv16x_fifo_timestamp_batch_set(&lsm6dsv16x_ctx, LSM6DSV16X_TMSTMP_DEC_32);
    lsm6dsv16x_timestamp_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);

    /* Set full scale */
    lsm6dsv16x_xl_full_scale_set(&lsm6dsv16x_ctx, LSM6DSV16X_2g);
    lsm6dsv16x_gy_full_scale_set(&lsm6dsv16x_ctx, LSM6DSV16X_1000dps);

    /* Configure filtering chain */
    // filt_settling_mask.drdy = PROPERTY_ENABLE;
    // filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    // filt_settling_mask.irq_g = PROPERTY_ENABLE;
    // lsm6dsv16x_filt_settling_mask_set(&lsm6dsv16x_ctx, filt_settling_mask);
    // lsm6dsv16x_filt_xl_lp2_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
    // lsm6dsv16x_filt_xl_lp2_bandwidth_set(&lsm6dsv16x_ctx, LSM6DSV16X_XL_STRONG);

    lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
    lsm6dsv16x_gy_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);

    /*
    * Prepare sensor hub to read data from external slave0 (lis2mdl) and
    * slave1 (lps22df) continuously in order to store data in FIFO.
    */
    sh_cfg_read.slv_add = (LIS2MDL_I2C_ADD & 0xFEU) >> 1; /* 7bit I2C address */
    sh_cfg_read.slv_subadd = LIS2MDL_OUTX_L_REG;
    sh_cfg_read.slv_len = 6;
    lsm6dsv16x_sh_slv_cfg_read(&lsm6dsv16x_ctx, 0, &sh_cfg_read);
    lsm6dsv16x_fifo_sh_batch_slave_set(&lsm6dsv16x_ctx, 0, PROPERTY_ENABLE);


    /* Configure Sensor Hub data rate */
    lsm6dsv16x_sh_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_SH_120Hz);

    /* Configure Sensor Hub to read one slave. */
    lsm6dsv16x_sh_slave_connected_set(&lsm6dsv16x_ctx, LSM6DSV16X_SLV_0);

    /* set SHUB write_once bit */
    lsm6dsv16x_sh_write_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_ONLY_FIRST_CYCLE);

    /* Enable I2C Master. */
    lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);

    /* Set Output Data Rate.
    * Selected data rate have to be equal or greater with respect
    * with MLC data rate.
    */
    lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_1920Hz);
    lsm6dsv16x_gy_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_1920Hz);
    PX4_INFO("IMU CONFIG SUCCESS \n");
    return PX4_OK;
}

void LSM6DSV16X::RunImpl()
{
    lsm6dsv16x_fifo_status_t fifo_status;
    lsm6dsv16x_fifo_status_get(&lsm6dsv16x_ctx, &fifo_status);
    // DEVICE_LOG("fifo_status.fifo_bdr = %d \nfifo_status.fifo_full = %d \nfifo_status.fifo_level = %d \nfifo_status.fifo_ovr = %d \nfifo_status.fifo_th = %d \n",fifo_status.fifo_bdr,fifo_status.fifo_full,fifo_status.fifo_level,fifo_status.fifo_ovr,fifo_status.fifo_th);
    if (fifo_status.fifo_th) {
        uint16_t num = 0;
        int16_t *datax;
        int16_t *datay;
        int16_t *dataz;
        int32_t *ts;
        /* 读取FIFO状态 */
        lsm6dsv16x_fifo_status_get(&lsm6dsv16x_ctx, &fifo_status);
        num = fifo_status.fifo_level;

        DEVICE_LOG("-- FIFO num %d \r\n", num);

        while (num--) {
            lsm6dsv16x_fifo_out_raw_t f_data;
            float_t ts_usec;

            /* 读取FIFO数据 */
            lsm6dsv16x_fifo_out_raw_get(&lsm6dsv16x_ctx, &f_data);
            datax = (int16_t *)&f_data.data[0];
            datay = (int16_t *)&f_data.data[2];
            dataz = (int16_t *)&f_data.data[4];
            ts = (int32_t *)&f_data.data[0];

            switch (f_data.tag) {
            case 1: //LSM6DSV16X_GY_NC_TAG:
                    {
                        // datasheet : Table 3. Mechanical characteristics
                        lsm6dsv16x_from_fs1000_to_mdps(*datax);
                        lsm6dsv16x_from_fs1000_to_mdps(*datay);
                        lsm6dsv16x_from_fs1000_to_mdps(*dataz);
                        DEVICE_LOG("gray:\t%4.2f\t%4.2f\t%4.2f\r\n",
                            (double)lsm6dsv16x_from_fs1000_to_mdps(*datax),
                            (double)lsm6dsv16x_from_fs1000_to_mdps(*datay),
                            (double)lsm6dsv16x_from_fs1000_to_mdps(*dataz)
                        );
                        break;
                    }
            case 0x2:
                DEVICE_LOG("ACC:\t%4.2f\t%4.2f\t%4.2f[mg]\r\n",
                        (double)lsm6dsv16x_from_fs2_to_mg(*datax),
                        (double)lsm6dsv16x_from_fs2_to_mg(*datay),
                        (double)lsm6dsv16x_from_fs2_to_mg(*dataz));
                break;
            case 0x4:
                ts_usec = lsm6dsv16x_from_lsb_to_nsec(*ts)/1000;
                DEVICE_LOG("TIMESTAMP %6.1f [us] (lsb: %d)\r\n", (double)ts_usec, *ts);
                break;
            case 0xE:
                DEVICE_LOG("LIS2MDL:\t%4.2f\t%4.2f\t%4.2f[mGa]\r\n",
                        (double)lis2mdl_from_lsb_to_mgauss(*datax),
                        (double)lis2mdl_from_lsb_to_mgauss(*datay),
                        (double)lis2mdl_from_lsb_to_mgauss(*dataz));
                break;
            default:
                DEVICE_LOG("Invalid TAG %02x\r\n", f_data.tag);
                break;
            }
        }
        DEVICE_LOG("------ \r\n\r\n");
    }
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
//   PX4_INFO("slv_add: 0x%02X",sh_cfg_read.slv_add);
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dsv16x_sh_slv_cfg_read(&lsm6dsv16x_ctx, 0, &sh_cfg_read);
//   PX4_INFO("lsm6dsv16x_sh_slv_cfg_read = %d\n", ret);

  ret = lsm6dsv16x_sh_slave_connected_set(&lsm6dsv16x_ctx, LSM6DSV16X_SLV_0);
//   PX4_INFO("lsm6dsv16x_sh_slave_connected_set = %d\n", ret);

  /* Enable I2C Master and I2C master. */
  ret = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
//   PX4_INFO("lsm6dsv16x_sh_master_set = %d\n", ret);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_120Hz);
//   PX4_INFO("lsm6dsv16x_xl_data_rate_set = %d\n", ret);

  /* Wait Sensor Hub operation flag set. */
  ret = lsm6dsv16x_acceleration_raw_get(&lsm6dsv16x_ctx, raw_xl);
//   PX4_INFO("lsm6dsv16x_acceleration_raw_get = %d\n", ret);

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
//   PX4_INFO("lsm6dsv16x_sh_read_data_raw_get = %d data=%d \n",ret, data[0]);

  return ret;
}

/* 延迟函数 */
void LSM6DSV16X::platform_delay(uint32_t ms)
{
    px4_usleep(ms);
}


