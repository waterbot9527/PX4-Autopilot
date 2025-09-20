px4_add_module(
    MODULE drivers__imu__st__lsm6dsv16x
    MAIN lsm6dsv16x
    SRCS
        LSM6DSV16X.cpp
        LSM6DSV16X.hpp
        lsm6dsv16x_main.cpp
        ST_LSM6DSV16X_Registers.hpp
    DEPENDS
        drivers_accelerometer
        drivers_gyroscope
        px4_work_queue
)
