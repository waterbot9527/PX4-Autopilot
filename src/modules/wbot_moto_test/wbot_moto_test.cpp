#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/topics/wbot_ctrl_moto.h>
#include <uORB/topics/wbot_ctrl_led.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <algorithm>
//#include <px4_platform_common/px4_log.h>

static volatile px4_task_t g_toggle2_task{-1};
static px4::atomic_bool g_toggle2_should_exit{false};

static void publish_all_motor_stop()
{
    struct wbot_ctrl_moto_s moto_msg{};
    moto_msg.timestamp = hrt_absolute_time();

    for (int n = 0; n < 8; n++) {
        moto_msg.speed[n] = 0;
        moto_msg.direction[n] = 0;
    }

    orb_advert_t moto_pub = orb_advertise(ORB_ID(wbot_ctrl_moto), &moto_msg);

    if (moto_pub != nullptr) {
        orb_publish(ORB_ID(wbot_ctrl_moto), moto_pub, &moto_msg);
    }
}

static int toggle2_worker_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    orb_advert_t moto_pub = nullptr;
    struct wbot_ctrl_moto_s moto_msg{};

    moto_msg.timestamp = hrt_absolute_time();
    for (int i = 0; i < 8; i++) {
        moto_msg.speed[i] = 0;
        moto_msg.direction[i] = 0;
    }

    moto_pub = orb_advertise(ORB_ID(wbot_ctrl_moto), &moto_msg);
    if (moto_pub == nullptr) {
        PX4_ERR("toggle2: advertise failed");
        g_toggle2_task = -1;
        return -1;
    }

    const int idx = 1; // motor index 2 -> array index 1
    int16_t speeds[2] = {255, -255};
    int cur = 0;

    while (!g_toggle2_should_exit.load()) {
        int16_t v = speeds[cur];

        if (v < 0) {
            moto_msg.direction[idx] = 1;
            moto_msg.speed[idx] = static_cast<uint8_t>(-v);
        } else {
            moto_msg.direction[idx] = 0;
            moto_msg.speed[idx] = static_cast<uint8_t>(v);
        }

        moto_msg.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(wbot_ctrl_moto), moto_pub, &moto_msg);

        cur = 1 - cur;
        px4_usleep(10000000); // 10s 切换周期
    }

    for (int n = 0; n < 8; n++) {
        moto_msg.speed[n] = 0;
        moto_msg.direction[n] = 0;
    }
    moto_msg.timestamp = hrt_absolute_time();
    orb_publish(ORB_ID(wbot_ctrl_moto), moto_pub, &moto_msg);

    g_toggle2_task = -1;
    PX4_INFO("toggle2 worker exited");
    return 0;
}

extern "C" __EXPORT int wbot_moto_test_main(int argc, char *argv[]);
extern "C" __EXPORT int wbot_moto_test_main(int argc, char *argv[])
{
    PX4_INFO("=== wbot test publisher start ===");

    // 背景切换命令：
    //  - toggle2: 启动后台任务并立即返回 shell
    //  - toggle2_stop: 请求后台任务退出并发布全停消息
    if (argc > 1 && strcmp(argv[1], "toggle2_stop") == 0) {
        if (g_toggle2_task < 0) {
            PX4_WARN("toggle2 is not running");
        } else {
            g_toggle2_should_exit.store(true);
            PX4_INFO("toggle2 stop requested");
        }

        publish_all_motor_stop();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "toggle2") == 0) {
        if (g_toggle2_task >= 0) {
            PX4_WARN("toggle2 is already running");
            return 0;
        }

        g_toggle2_should_exit.store(false);
        g_toggle2_task = px4_task_spawn_cmd("wbot_toggle2",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_DEFAULT,
                                             1500,
                                             (px4_main_t)toggle2_worker_main,
                                             nullptr);

        if (g_toggle2_task < 0) {
            PX4_ERR("Failed to start toggle2 worker");
            g_toggle2_task = -1;
            return -1;
        }

        PX4_INFO("toggle2 started in background");
        return 0;
    }
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


