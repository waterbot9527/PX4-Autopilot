#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/topics/wbot_ctrl_moto.h>
#include <uORB/topics/wbot_ctrl_led.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <algorithm>
//#include <px4_platform_common/px4_log.h>
extern "C" __EXPORT int wbot_moto_test_main(int argc, char *argv[]);
extern "C" __EXPORT int wbot_moto_test_main(int argc, char *argv[])
{
    PX4_INFO("=== wbot test publisher start ===");

    // 支持同时测试电机和 LED：
    // 用法示例：
    // 1) 同时测试电机和 LED: wbot_moto_test <moto_index 1-8> <moto_speed 0-255> <led_id> <led_value 0-240>
    // 2) 仅测试电机: wbot_moto_test <moto_index> <moto_speed>
    // 3) 仅测试 LED: wbot_moto_test 0 0 <led_id> <led_value>
    // 4) 无参数: 停止所有电机并将 LED 设为 0

    int moto_index = 0; // 1..8, 0 表示不操作电机（或在无参数时停止所有）
    int moto_speed = 0; // 0..255
    int led_id = -1;    // -1 表示不操作 LED
    int led_value = -1; // 0..240

    if (argc > 2) {
        moto_index = atoi(argv[1]);
        moto_speed = atoi(argv[2]);
    }

    if (argc > 4) {
        led_id = atoi(argv[3]);
        led_value = atoi(argv[4]);
    }

    // 如果没有传入任何参数，执行停止动作（停止所有电机，设置 LED 为0）
    const bool no_args = (argc <= 1);

    if (no_args) {
        // 停止所有电机
        struct wbot_ctrl_moto_s moto_msg{};
        moto_msg.timestamp = hrt_absolute_time();
        for (int n = 0; n < 8; n++) {
            moto_msg.speed[n] = 0;
            moto_msg.direction[n] = 0;
        }

        orb_advert_t moto_pub = orb_advertise(ORB_ID(wbot_ctrl_moto), &moto_msg);
        if (moto_pub != nullptr) {
            PX4_INFO("Published stop message to wbot_moto (no-arg)");
        } else {
            PX4_ERR("Failed to publish stop message to wbot_moto");
        }

        // 将默认 LED 设为 0（如果需要更精细的控制，可在命令行提供 LED id/value）
        struct wbot_ctrl_led_s led_msg{};
        led_msg.led_id = 0;
        led_msg.light_value = 0;
        led_msg.timestamp = hrt_absolute_time();
        orb_advert_t led_pub = orb_advertise(ORB_ID(wbot_ctrl_led), &led_msg);
        if (led_pub != nullptr) {
            PX4_INFO("Published default LED off message (no-arg)");
        } else {
            PX4_ERR("Failed to publish default LED off message");
        }

        return 0;
    }

    // 如果有提供电机参数，单独发布电机消息（范围检查并容错）
    if (moto_index != 0) {
        if (moto_index < 1 || moto_index > 8) {
            PX4_WARN("moto_index out of range (1-8): %d, clamping", moto_index);
            moto_index = std::min(8, std::max(1, moto_index));
        }

        struct wbot_ctrl_moto_s moto_msg{};
        uint8_t idx = static_cast<uint8_t>(moto_index - 1);

        // 支持负值速度：范围 -255 .. +255
        if (moto_speed < -255) moto_speed = -255;
        if (moto_speed > 255) moto_speed = 255;

        // 负值表示反向，将 direction 设为 1，并使用绝对值作为 speed
        if (moto_speed < 0) {
            moto_msg.direction[idx] = 1;
            moto_msg.speed[idx] = static_cast<uint8_t>(-moto_speed);
        } else {
            moto_msg.direction[idx] = 0;
            moto_msg.speed[idx] = static_cast<uint8_t>(moto_speed);
        }
        moto_msg.timestamp = hrt_absolute_time();

        orb_advert_t moto_pub = orb_advertise(ORB_ID(wbot_ctrl_moto), &moto_msg);
        if (moto_pub != nullptr) {
            PX4_INFO("Published wbot_moto message index=%d speed=%d", moto_index, moto_speed);
        } else {
            PX4_ERR("Failed to publish wbot_moto message");
        }
    }

    // 如果提供了 LED 参数，则发布 LED 消息
    if (led_id >= 0) {
        struct wbot_ctrl_led_s led_msg{};
        led_msg.led_id = led_id;

        if (led_value < 0) led_value = 0;
        if (led_value > 240) led_value = 240;

        led_msg.light_value = led_value;
        led_msg.timestamp = hrt_absolute_time();

        orb_advert_t led_pub = orb_advertise(ORB_ID(wbot_ctrl_led), &led_msg);
        if (led_pub != nullptr) {
            PX4_INFO("Published wbot_led message id=%d value=%d", led_id, led_value);
        } else {
            PX4_ERR("Failed to publish wbot_led message");
        }
    }

    return 0;
}


