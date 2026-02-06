#ifndef PARSE_DATA_H
#define PARSE_DATA_H

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

#define WBOT_SDEV_TAG_IMU    0x1
#define WBOT_SDEV_TAG_MS5837 0x2
#define WBOT_SDEV_TAG_MOTO0 0x3
#define WBOT_SDEV_TAG_MOTO1 0x4
#define WBOT_SDEV_TAG_MOTO2 0x5
#define WBOT_SDEV_TAG_MOTO3 0x6

#define  MOTOR_INDEX_0 (0)
#define  MOTOR_INDEX_1 (1)
#define  MOTOR_INDEX_2 (2)
#define  MOTOR_INDEX_3 (3)


#define  MAX_SPI_BUF_LEN (256)
#define  MOTOR_DATA_SIZE (10)
#define  MOTOR_MAX_NUM (4)




// 用于存放生成的 table
static uint32_t crc32_table[256];
static int crc32_table_inited = 0;

// 生成 lookup table（只需调用一次）
static void wbot_crc32_init_table(void)
{
    if (crc32_table_inited) return;
    const uint32_t poly = 0xEDB88320u; // 反射多项式
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t crc = i;
        for (int j = 0; j < 8; ++j) {
            if (crc & 1) crc = (crc >> 1) ^ poly;
            else         crc >>= 1;
        }
        crc32_table[i] = crc;
    }
    crc32_table_inited = 1;
}

// 计算 CRC32，语义与 python zlib.crc32(data) 等价
static uint32_t wbot_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu; // 内部使用反转初值
    while (len--) {
        crc = (crc >> 8) ^ crc32_table[(crc ^ *p++) & 0xFFu];
    }
    return crc ^ 0xFFFFFFFFu; // 结束时再反转 —— 与 zlib 返回一致
}

#define MCU_CMD_MOTO_STOP (1)
#define MCU_CMD_MOTO_START (2)
#define MCU_CMD_REBOOT  (4)
#define MCU_CMD_SetLight  (6)

struct mcu_cmd_motor {
    uint8_t cmd ; // start or stop cmd
    uint8_t board_id; // [0-1]
    uint8_t motor_id; // [0-3]
    uint8_t speed; //
    uint8_t direction:1 ; // 1 : 正转， 0: 反转
    uint32_t last_send_time;
};

constexpr const uint32_t MAX_MOTO_CNT = 4;
constexpr const uint32_t MAX_BOARD_CNT = 2;

class motor_cmd_manager
{
public :
    mcu_cmd_motor cmd[MAX_BOARD_CNT][MAX_MOTO_CNT];

    motor_cmd_manager()
    {
        for (uint32_t b = 0; b < MAX_BOARD_CNT; b++)
            for (uint32_t n = 0; n < MAX_MOTO_CNT; n++)
            {
                cmd[b][n].cmd = MCU_CMD_MOTO_STOP;
                cmd[b][n].motor_id = n;
                cmd[b][n].board_id = b;
                cmd[b][n].speed = 0;
                cmd[b][n].direction = 0;
                cmd[b][n].last_send_time = 0;
            }
    }

    int32_t get_cmd_buffer_for_send(uint8_t input_buffer, uint32_t max_input_len, uint32_t board_id)
    {
        if ( board_id >= MAX_BOARD_CNT )
            return -1;

        // TODO: build command buffer here when needed
        (void)input_buffer;
        (void)max_input_len;
        return -1;
    }


private :

};


#endif
