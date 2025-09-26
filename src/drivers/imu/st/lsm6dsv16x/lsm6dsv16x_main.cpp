#include "LSM6DSV16X.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

void LSM6DSV16X::print_usage()
{
    PRINT_MODULE_USAGE_NAME("lsm6dsv16x", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("imu");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int lsm6dsv16x_main(int argc, char *argv[])
{
    int ch;
    using ThisDriver = LSM6DSV16X;
    BusCLIArguments cli{true, false};  // 启用I2C，禁用SPI
    cli.requested_bus = 1; // I2C-1
    cli.default_i2c_frequency = 400;  // I2C默认频率400kHz
    cli.i2c_address = 0x6B;  // 默认I2C地址
    cli.bus_option = I2CSPIBusOption::I2CInternal;
    PX4_INFO("IMU START\n");


    while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
        switch (ch) {
        case 'R':
            cli.rotation = (enum Rotation)atoi(cli.optArg());
            break;
        }
    }

    const char *verb = cli.optArg();
    if (!verb) {
        ThisDriver::print_usage();
        return -1;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, DRV_IMU_DEVTYPE_ST_LSM6DSV16X);

    if (!strcmp(verb, "start")) {
        return ThisDriver::module_start(cli, iterator);
    }

    if (!strcmp(verb, "stop")) {
        return ThisDriver::module_stop(iterator);
    }

    if (!strcmp(verb, "status")) {
        return ThisDriver::module_status(iterator);
    }

    ThisDriver::print_usage();
    return -1;
}
