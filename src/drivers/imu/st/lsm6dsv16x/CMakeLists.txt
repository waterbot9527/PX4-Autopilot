px4_add_module(
    MODULE drivers__imu__st__lsm6dsv16x
    MAIN lsm6dsv16x
    SRCS
        LSM6DSV16X.cpp
        LSM6DSV16X.hpp
        lsm6dsv16x_main.cpp
    DEPENDS
        st_lsm6dsv16x_common
		st_lis2mdl_common
        drivers_accelerometer
		drivers_gyroscope
        drivers_magnetometer
        px4_work_queue
)
