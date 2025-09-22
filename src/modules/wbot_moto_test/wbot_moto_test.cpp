#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/topics/wbot_ctrl_moto.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
//#include <px4_platform_common/px4_log.h>

extern "C" __EXPORT int wbot_moto_test_main(int argc, char *argv[]);

int wbot_moto_test_main(int argc, char *argv[])
{
    PX4_INFO("=== wbot_moto test publisher ===");

    int instance = 0;

    if (argc > 1) {
        instance = atoi(argv[1]);
    }

    // 创建消息
    struct wbot_ctrl_moto_s msg{};
//     msg.speed[0] = 100;
//     msg.speed[1] = 110;
//     msg.speed[2] = 120;
//     msg.speed[3] = 130;

//     msg.direction[0] = 0;
//     msg.direction[1] = 1;
//     msg.direction[2] = 0;
//     msg.direction[3] = 1;

    msg.timestamp = hrt_absolute_time();


    // 发布 topic
    orb_advert_t pub = orb_advertise_multi(ORB_ID(wbot_ctrl_moto), &msg, &instance);

    if (pub != nullptr) {
        PX4_INFO("Published wbot_moto message to instance %d", instance);
    } else {
        PX4_ERR("Failed to publish wbot_moto message");
    }

    return 0;
}


