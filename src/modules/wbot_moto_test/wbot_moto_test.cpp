#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/topics/wbot_ctrl_moto.h>
#include <uORB/topics/wbot_ctrl_led.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
//#include <px4_platform_common/px4_log.h>
#define  MOTO_TEST
// #define  LED_TEST
extern "C" __EXPORT int wbot_moto_test_main(int argc, char *argv[]);

int wbot_moto_test_main(int argc, char *argv[])
{
    PX4_INFO("=== wbot test publisher start ===");

#ifdef MOTO_TEST

    int cmd1 = 0;
    int cmd2 = 0;

    if (argc > 2) {
        cmd1 = atoi(argv[1]);
        cmd2 = atoi(argv[2]);
    }
#endif

#ifdef LED_TEST

    int cmd1 = 0;
    int cmd2 = 0;
    int value = 0;

    if (argc > 3) {
        cmd1 = atoi(argv[1]);
        cmd2 = atoi(argv[2]);
        value = atoi(argv[3]);
    }
#endif

    if (cmd1 == 1) {
#ifdef LED_TEST
        // LED测试功能（仅当定义LED_TEST时编译）
        struct wbot_ctrl_led_s led_msg{};
        led_msg.led_id = cmd2;
        led_msg.light_value = value;
        led_msg.timestamp = hrt_absolute_time();

        // 发布LED消息
        orb_advert_t led_pub = orb_advertise(ORB_ID(wbot_ctrl_led), &led_msg);
        if (led_pub != nullptr) {
            PX4_INFO("Published wbot_led message to cmd1 %d", cmd1);
        } else {
            PX4_ERR("Failed to publish wbot_led message");
        }
#endif

#ifdef MOTO_TEST
        // MOTO测试功能（仅当定义MOTO_TEST时编译）
        struct wbot_ctrl_moto_s moto_msg{};
        // 设置电机速度（根据需要调整）
        for (int n = 0; n < 8; n++) {
            moto_msg.speed[n] = 60;
            moto_msg.direction[n] = cmd2;
        }
        moto_msg.timestamp = hrt_absolute_time();

        // 发布MOTO消息
        orb_advert_t moto_pub = orb_advertise(ORB_ID(wbot_ctrl_moto), &moto_msg);
        if (moto_pub != nullptr) {
            PX4_INFO("Published wbot_moto message to cmd1 %d", cmd1);
        } else {
            PX4_ERR("Failed to publish wbot_moto message");
        }
#endif

    } else {
#ifdef LED_TEST
        // LED测试功能（仅当定义LED_TEST时编译）
        struct wbot_ctrl_led_s led_msg{};
        led_msg.led_id = cmd2;
        led_msg.light_value = 0;
        led_msg.timestamp = hrt_absolute_time();

        // 发布LED消息
        orb_advert_t led_pub = orb_advertise(ORB_ID(wbot_ctrl_led), &led_msg);
        if (led_pub != nullptr) {
            PX4_INFO("Published wbot_led message to cmd1 %d", cmd1);
        } else {
            PX4_ERR("Failed to publish wbot_led messagsrc/modules/e");
        }
#endif


#ifdef MOTO_TEST
        // 关闭电机的消息（仅当定义MOTO_TEST时编译）
        struct wbot_ctrl_moto_s moto_msg{};
        moto_msg.timestamp = hrt_absolute_time();

        for (int n = 0; n < 8; n++) {
            moto_msg.speed[n] = 0;  // 停止电机
        }

        orb_advert_t moto_pub = orb_advertise(ORB_ID(wbot_ctrl_moto), &moto_msg);
        if (moto_pub != nullptr) {
            PX4_INFO("Published stop message to wbot_moto (cmd1 %d)", cmd1);
        } else {
            PX4_ERR("Failed to publish stop message to wbot_moto");
        }
#endif
    }

    return 0;
}


