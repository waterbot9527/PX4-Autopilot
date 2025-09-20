#pragma once

#include <cstdint>

namespace ST_LSM6DSV16X {

enum class Register : uint8_t {
    WHO_AM_I        = 0x0F,  // 设备ID寄存器

    // 陀螺仪控制寄存器
    CTRL1_G         = 0x10,  // 陀螺仪输出数据率、量程配置
    CTRL2_G         = 0x11,  // 陀螺仪滤波配置

    // 加速度计控制寄存器
    CTRL3_XL        = 0x12,  // 加速度计输出数据率、量程配置
    CTRL4_XL        = 0x13,  // 加速度计滤波配置

    // 温度传感器
    OUT_TEMP_L      = 0x20,
    OUT_TEMP_H      = 0x21,

    // 陀螺仪数据输出
    OUTX_L_G        = 0x22,
    OUTX_H_G        = 0x23,
    OUTY_L_G        = 0x24,
    OUTY_H_G        = 0x25,
    OUTZ_L_G        = 0x26,
    OUTZ_H_G        = 0x27,

    // 加速度计数据输出
    OUTX_L_XL       = 0x28,
    OUTX_H_XL       = 0x29,
    OUTY_L_XL       = 0x2A,
    OUTY_H_XL       = 0x2B,
    OUTZ_L_XL       = 0x2C,
    OUTZ_H_XL       = 0x2D,

    // 状态寄存器
    STATUS_REG      = 0x1E,

    // FIFO控制寄存器
    FIFO_CTRL1      = 0x06,
    FIFO_CTRL2      = 0x07,
    FIFO_STATUS1    = 0x3A,
    FIFO_STATUS2    = 0x3B,

    // 系统控制寄存器
    CTRL10_C        = 0x19,  // 复位、接口配置
    CTRL12_C        = 0x1B   // 传感器使能

    CTRL11_C        = 0x1A,  // I2C主机模式配置
    CTRL13_C        = 0x1C,  // 从设备1配置（LIS2MDL）
    CTRL14_C        = 0x1D,  // 从设备1数据读取配置
    CTRL15_C        = 0x1E,  // 从设备1地址配置
    SUB_ADDR        = 0x20,  // 从设备寄存器地址（写入时用）
    SUB_DATA        = 0x21,  // 从设备数据（读写缓冲区）
    MASTER_STATUS   = 0x22,  // I2C主机状态寄存器
};

static constexpr uint8_t I2C_ADDR_PRIMARY = 0x6A;   // SDO/SA0引脚接GND时的地址
static constexpr uint8_t I2C_ADDR_SECONDARY = 0x6B; // SDO/SA0引脚接VCC时的地址

// WHO_AM_I默认值 (需根据datasheet确认)
static constexpr uint8_t WHO_AM_I_ID = 0x70;

// 输出数据率定义 (Hz)
static constexpr float G_ODR = 1600.0f;    // 陀螺仪默认采样率
static constexpr float XL_ODR = 1600.0f;   // 加速度计默认采样率

// FIFO配置
namespace FIFO {
    static constexpr uint8_t SIZE = 512;     // FIFO深度 (需确认)
}

// 控制寄存器位定义 (示例，需根据datasheet完善)
namespace CTRL1_G_BIT {
    static constexpr uint8_t ODR_G_1600HZ = 0x70;  // 1600Hz输出率
    static constexpr uint8_t FS_G_2000DPS = 0x00;  // 2000dps量程
}

namespace CTRL3_XL_BIT {
    static constexpr uint8_t ODR_XL_1600HZ = 0x70; // 1600Hz输出率
    static constexpr uint8_t FS_XL_16G = 0x08;     // 16G量程
}

namespace CTRL10_C_BIT {
    static constexpr uint8_t SW_RESET = 0x01;      // 软件复位
    static constexpr uint8_t IF_ADD_INC = 0x04;    // 自动地址递增
}

namespace FIFO_CTRL1_BIT {
    static constexpr uint8_t FMODE_CONTINUOUS = 0x40; // 连续模式
}

// LIS2MDL相关定义（从设备）
namespace LIS2MDL {
    static constexpr uint8_t WHO_AM_I_ID = 0x40;
    static constexpr uint8_t I2C_ADDR = 0x1E;  // LIS2MDL默认I2C地址
    enum class Register : uint8_t {
        WHO_AM_I    = 0x4F,
        CTRL_REG1   = 0x20,  // 配置寄存器
        OUTX_L      = 0x28,  // X轴数据低字节
        OUTX_H      = 0x29,
        OUTY_L      = 0x2A,
        OUTY_H      = 0x2B,
        OUTZ_L      = 0x2C,
        OUTZ_H      = 0x2D,
    };
}

// Sensor Hub配置位定义
namespace CTRL11_C_BIT {
    static constexpr uint8_t I2C_MASTER_EN = 0x80;  // 启用I2C主机模式
    static constexpr uint8_t I2C_MASTER_CLK_400K = 0x02;  // 400kHz I2C时钟
}

namespace CTRL13_C_BIT {
    static constexpr uint8_t SLV1_EN = 0x80;  // 启用从设备1（LIS2MDL）
    static constexpr uint8_t SLV1_REG_ADDR = 0x00;  // 从设备寄存器地址长度（8位）
}

namespace CTRL14_C_BIT {
    static constexpr uint8_t SLV1_RD_LEN = 0x06;  // 每次读取6字节（XYZ轴数据）
    static constexpr uint8_t SLV1_AUTO_RD = 0x40;  // 自动读取模式
}

namespace MASTER_STATUS_BIT {
    static constexpr uint8_t SLV1_DRDY = 0x01;  // 从设备1数据就绪
    static constexpr uint8_t SLV1_ERR = 0x02;   // 从设备1通信错误
}

} // namespace ST_LSM6DSV16X


