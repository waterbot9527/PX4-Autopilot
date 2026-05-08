#include "LSM6DSV16X.hpp"

// FIFO tag definitions (matches ST register values)
#define LSM6DSV16X_GY_NC_TAG             1
#define LSM6DSV16X_XL_NC_TAG             2
#define LSM6DSV16X_TEMPERATURE_TAG       3
#define LSM6DSV16X_TIMESTAMP_TAG         4
#define LSM6DSV16X_SENSORHUB_SLAVE0_TAG  0xE
#define LSM6DSV16X_SENSORHUB_SLAVE1_TAG  0xF
#define LSM6DSV16X_SENSORHUB_SLAVE2_TAG  0x10
#define LSM6DSV16X_SENSORHUB_SLAVE3_TAG  0x11


using namespace time_literals;

stmdev_ctx_t LSM6DSV16X::lsm6dsv16x_ctx = {};
stmdev_ctx_t LSM6DSV16X::lis2mdl_ctx = {};

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
    return (msb << 8u) | lsb;
}

LSM6DSV16X::LSM6DSV16X(const I2CSPIDriverConfig &config) :
    SPI(config),
    I2CSPIDriver(config),
    _px4_accel(get_device_id(), config.rotation),
    _px4_gyro(get_device_id(), config.rotation),
    _px4_mag(get_device_id(), config.rotation)
{
    // Initialize FIFO data structures
    memset(&_accel_fifo_data, 0, sizeof(_accel_fifo_data));
    memset(&_gyro_fifo_data, 0, sizeof(_gyro_fifo_data));
    memset(&_mag_data, 0, sizeof(_mag_data));

    // Sensor calibration: scale and range settings
    _px4_accel.set_scale(0.061f * CONSTANTS_ONE_G / 1000.0f); // 0.061 mg/LSB at ±2g
    _px4_accel.set_range(2.f * CONSTANTS_ONE_G);

    _px4_gyro.set_scale(math::radians(35.f / 1000.f)); // 35 mdps/LSB
    _px4_gyro.set_range(math::radians(1000.f));         // LSM6DSV16X_1000dps

    _px4_mag.set_scale(1.5f / 1000.f); // 1.5 mG/LSB
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
        PX4_INFO("SPI::init failed (%i)", ret);
        return ret;
    }
    PX4_INFO("SPI::init success (%i)", ret);

    return Reset() ? 0 : -1;
}

bool LSM6DSV16X::Reset()
{
    _state = STATE::RESET;
    ScheduleClear();

	// Schedule at 5ms (200Hz) to drain FIFO before overflow
	uint32_t interval_delay_us = 5 * 1000;
	ScheduleOnInterval(interval_delay_us, interval_delay_us);
    return true;
}

void LSM6DSV16X::exit_and_cleanup()
{
    I2CSPIDriverBase::exit_and_cleanup();
}

void LSM6DSV16X::print_status()
{
    I2CSPIDriverBase::print_status();

    PX4_INFO("\n=== LSM6DSV16X Performance ===");
    PX4_INFO("Total samples published: %u", _total_samples_published);

    perf_print_counter(_fifo_read_perf);
    perf_print_counter(_accel_pub_perf);
    perf_print_counter(_gyro_pub_perf);

    PX4_INFO("\n=== Error Counters ===");
    perf_print_counter(_bad_register_perf);
    perf_print_counter(_bad_transfer_perf);
    perf_print_counter(_fifo_empty_perf);
    perf_print_counter(_fifo_overflow_perf);
    perf_print_counter(_fifo_reset_perf);
}

int LSM6DSV16X::probe()
{
    PX4_INFO("Using bus %d", get_device_bus());

    /* Initialize SPI device context */
    lsm6dsv16x_ctx.write_reg = LSM6DSV16X::platform_write;
    lsm6dsv16x_ctx.read_reg = LSM6DSV16X::platform_read;
    lsm6dsv16x_ctx.mdelay = LSM6DSV16X::platform_delay;
    lsm6dsv16x_ctx.handle = this;

    /* Initialize sensor hub (I2C master) context for LIS2MDL */
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

    /* Restore default configuration */
    lsm6dsv16x_reset_set(&lsm6dsv16x_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
    do {
        lsm6dsv16x_reset_get(&lsm6dsv16x_ctx, &rst);
    } while (rst != LSM6DSV16X_READY);
    PX4_INFO("lsm6dsv16x reset done");

    /* Enable block data update */
    lsm6dsv16x_block_data_update_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);

    /* Set accelerometer full scale */
    lsm6dsv16x_xl_full_scale_set(&lsm6dsv16x_ctx, LSM6DSV16X_2g);

    uint8_t func_cfg_access = 0x01; // Enable access to sensor hub config registers
    lsm6dsv16x_write_reg(&lsm6dsv16x_ctx, LSM6DSV16X_FUNC_CFG_ACCESS, &func_cfg_access, 1);
    platform_delay(10);

    /* Configure LIS2MDL magnetometer via sensor hub */
    lis2mdl_device_id_get(&lis2mdl_ctx, &whoamI);
    if (whoamI != LIS2MDL_ID) {
        DEVICE_LOG("LIS2MDL not found! ID: 0x%02X\n", whoamI);
        return PX4_ERROR;
    }
    DEVICE_LOG("LIS2MDL whoamI read : 0x%02X \n", whoamI);

    /* Restore LIS2MDL default configuration */
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

    /* Set FIFO batch XL/Gyro ODR to 1920Hz */
    lsm6dsv16x_fifo_xl_batch_set(&lsm6dsv16x_ctx, (lsm6dsv16x_fifo_xl_batch_t)LSM6DSV16X_XL_BATCHED_AT_1920Hz);
    lsm6dsv16x_fifo_gy_batch_set(&lsm6dsv16x_ctx, (lsm6dsv16x_fifo_gy_batch_t)LSM6DSV16X_GY_BATCHED_AT_1920Hz);


    /* Set FIFO mode to Stream mode (aka Continuous Mode) */
    lsm6dsv16x_fifo_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_STREAM_MODE);

    // pin_int.fifo_th = PROPERTY_ENABLE;
    // lsm6dsv16x_pin_int1_route_set(&lsm6dsv16x_ctx, &pin_int);
    //lsm6dsv16x_pin_int2_route_set(&lsm6dsv16x_ctx, &pin_int);


    /* Set Output Data Rate to 1920Hz */
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

    /* Enable sensor hub I2C master. */
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

    if (fifo_status.fifo_th) {
        perf_count(_fifo_read_perf);  // count each FIFO read
        uint16_t num = 0;
        int16_t *datax;
        int16_t *datay;
        int16_t *dataz;

        lsm6dsv16x_fifo_status_get(&lsm6dsv16x_ctx, &fifo_status);
        num = fifo_status.fifo_level;

        hrt_abstime current_time = hrt_absolute_time();

        // Reset FIFO sample counters
        _accel_samples = 0;
        _gyro_samples = 0;
        _mag_samples = 0;

        while (num-- && (_accel_samples < sensor_accel_fifo_s::MAX_SAMPLES ||
                         _gyro_samples < sensor_gyro_fifo_s::MAX_SAMPLES ||
                         _mag_samples < 1)) { // magnetometer processed once per batch

            lsm6dsv16x_fifo_out_raw_t f_data;

            /* Read one FIFO entry */
            lsm6dsv16x_fifo_out_raw_get(&lsm6dsv16x_ctx, &f_data);
            datax = (int16_t *)&f_data.data[0];
            datay = (int16_t *)&f_data.data[2];
            dataz = (int16_t *)&f_data.data[4];

            switch (f_data.tag) {
            case LSM6DSV16X_GY_NC_TAG: // Gyroscope data
                if (_gyro_samples < sensor_gyro_fifo_s::MAX_SAMPLES) {
                    _gyro_fifo_data.x[_gyro_samples] = *datax;
                    _gyro_fifo_data.y[_gyro_samples] = *datay;
                    _gyro_fifo_data.z[_gyro_samples] = *dataz;
                    _gyro_samples++;
                }
                break;

            case LSM6DSV16X_XL_NC_TAG: // Accelerometer data
                if (_accel_samples < sensor_accel_fifo_s::MAX_SAMPLES) {
                    _accel_fifo_data.x[_accel_samples] = *datax;
                    _accel_fifo_data.y[_accel_samples] = *datay;
                    _accel_fifo_data.z[_accel_samples] = *dataz;
                    _accel_samples++;
                }
                break;

            case LSM6DSV16X_SENSORHUB_SLAVE0_TAG: // Magnetometer data via sensor hub (LIS2MDL)
                if (_mag_samples < 1) { // Process one magnetometer sample at a time
                    // Magnetometer axes: X=right, Y=forward, Z=down -> body frame: X=forward, Y=right, Z=down
                    const float mx = static_cast<float>(*datay);
                    const float my = static_cast<float>(*datax);
                    const float mz = static_cast<float>(*dataz);
                    _px4_mag.update(current_time, mx, my, mz);
                    _mag_samples++;
                }
                break;

            case LSM6DSV16X_TIMESTAMP_TAG: // Hardware timestamp
                {
                    int32_t *ts = (int32_t *)&f_data.data[0];
                    float ts_usec = lsm6dsv16x_from_lsb_to_nsec(*ts) / 1000.0f;
                    (void)ts_usec;
                    _last_timestamp = current_time;
                }
                break;

            default:
                // Unknown tag, ignore
                break;
            }
        }

        // Publish accelerometer FIFO
        if (_accel_samples > 0) {
            _accel_fifo_data.timestamp_sample = current_time;
            _accel_fifo_data.samples = _accel_samples;
            _accel_fifo_data.dt = 1e6f / 1920.f; // 1920 Hz -> ~521 us per sample
            _px4_accel.updateFIFO(_accel_fifo_data);
            perf_count(_accel_pub_perf);
        }

        // Publish gyroscope FIFO
        if (_gyro_samples > 0) {
            _gyro_fifo_data.timestamp_sample = current_time;
            _gyro_fifo_data.samples = _gyro_samples;
            _gyro_fifo_data.dt = 1e6f / 1920.f;
            _px4_gyro.updateFIFO(_gyro_fifo_data);
            perf_count(_gyro_pub_perf);
        }

        _total_samples_published += _accel_samples;
    }
}
void LSM6DSV16X::ConfigureSampleRate(int sample_rate)
{

}

bool LSM6DSV16X::Configure()
{


    bool success = true;

    return success;
}



bool LSM6DSV16X::InitLIS2MDL()
{
    // TODO: implement LIS2MDL standalone init if needed
    bool success = true;
    return success;
}

/* SPI read function */
int LSM6DSV16X::platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    LSM6DSV16X *dev = reinterpret_cast<LSM6DSV16X *>(handle);

    constexpr uint16_t max_transfer_len = 64;

    if ((len == 0) || (len > max_transfer_len)) {
        return PX4_ERROR;
    }

    uint8_t buffer[max_transfer_len + 1] {};
    buffer[0] = reg | 0x80; // SPI read bit

    int ret = dev->transfer(buffer, buffer, len + 1);
    if (ret != PX4_OK) {
        PX4_ERR("SPI transfer failed reg=0x%02X len=%u", reg, len);
        return PX4_ERROR;
    }


    memcpy(bufp, &buffer[1], len);
    return PX4_OK;
}

/* SPI write function */
int LSM6DSV16X::platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    LSM6DSV16X *dev = reinterpret_cast<LSM6DSV16X *>(handle);

    constexpr uint16_t max_transfer_len = 64;

    if (len > max_transfer_len) {
        return PX4_ERROR;
    }

    uint8_t buffer[max_transfer_len + 1] {};
    buffer[0] = reg & 0x7F; // SPI write clears read bit
    memcpy(&buffer[1], bufp, len);

    int ret = dev->transfer(buffer, buffer, len + 1);
    if (ret != PX4_OK) {
        PX4_ERR("SPI transfer failed reg=0x%02X len=%u", reg, len);
        return PX4_ERROR;
    }
    return PX4_OK;
}



/* LIS2MDL write via sensor hub */
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

  /* Configure sensor hub write to target device. */
  sh_cfg_write.slv0_add = (i2c_add & 0xFEU) >> 1; /* 7-bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = lsm6dsv16x_sh_cfg_write(&lsm6dsv16x_ctx, &sh_cfg_write);

  /* Disable accelerometer. */
  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);

  /* Enable sensor hub I2C master. */
  lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_120Hz);

  /* Wait for accelerometer data ready. */
  lsm6dsv16x_acceleration_raw_get(&lsm6dsv16x_ctx, raw_xl);

  do {
    px4_usleep(1000);
    lsm6dsv16x_flag_data_ready_get(&lsm6dsv16x_ctx, &drdy);
  } while (!drdy.drdy_xl);

  do {
    px4_usleep(1000);
    lsm6dsv16x_sh_status_get(&lsm6dsv16x_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable sensor hub I2C master and XL trigger. */
  lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_DISABLE);
  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);

  return ret;
}

/* LIS2MDL read via sensor hub */
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

  /* Configure sensor hub read from target device. */
  sh_cfg_read.slv_add = (i2c_add & 0xFEU) >> 1; /* 7-bit I2C address */
//   PX4_INFO("slv_add: 0x%02X",sh_cfg_read.slv_add);
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dsv16x_sh_slv_cfg_read(&lsm6dsv16x_ctx, 0, &sh_cfg_read);
//   PX4_INFO("lsm6dsv16x_sh_slv_cfg_read = %d\n", ret);

  ret = lsm6dsv16x_sh_slave_connected_set(&lsm6dsv16x_ctx, LSM6DSV16X_SLV_0);
//   PX4_INFO("lsm6dsv16x_sh_slave_connected_set = %d\n", ret);

  /* Enable sensor hub I2C master. */
  ret = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
//   PX4_INFO("lsm6dsv16x_sh_master_set = %d\n", ret);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_120Hz);
//   PX4_INFO("lsm6dsv16x_xl_data_rate_set = %d\n", ret);

  /* Wait for accelerometer data ready. */
  ret = lsm6dsv16x_acceleration_raw_get(&lsm6dsv16x_ctx, raw_xl);
//   PX4_INFO("lsm6dsv16x_acceleration_raw_get = %d\n", ret);

  do {
    px4_usleep(1000);
    lsm6dsv16x_flag_data_ready_get(&lsm6dsv16x_ctx, &drdy);
  } while (!drdy.drdy_xl);

  do {
    lsm6dsv16x_sh_status_get(&lsm6dsv16x_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable sensor hub I2C master and XL trigger. */
  lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_DISABLE);
  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
  /* Read sensor hub output registers. */
  ret = lsm6dsv16x_sh_read_data_raw_get(&lsm6dsv16x_ctx, data, len);
//   PX4_INFO("lsm6dsv16x_sh_read_data_raw_get = %d data=%d \n",ret, data[0]);

  return ret;
}

/* Platform delay (microseconds) */
void LSM6DSV16X::platform_delay(uint32_t ms)
{
    px4_usleep(ms);
}


