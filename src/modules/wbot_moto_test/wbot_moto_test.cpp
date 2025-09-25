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
    int direction = 0;

    if (argc > 2) {
        instance = atoi(argv[1]);
        direction = atoi(argv[2]);
    }

    if ( instance ==  1 )
    {
        // 创建消息
        struct wbot_ctrl_moto_s msg{};
        msg.speed[0] = 60;
        msg.speed[1] = 60;
        msg.speed[2] = 60;
        msg.speed[3] = 60;
        msg.speed[4] = 60;
        msg.speed[5] = 60;
        msg.speed[6] = 60;
        msg.speed[7] = 60;

        msg.direction[0] = direction;
        msg.direction[1] = direction;
        msg.direction[2] = direction;
        msg.direction[3] = direction;

        msg.timestamp = hrt_absolute_time();

        // 发布 topic
        orb_advert_t pub = orb_advertise(ORB_ID(wbot_ctrl_moto), &msg);

        if (pub != nullptr) {
            PX4_INFO("Published wbot_moto message to instance %d", instance);
        } else {
            PX4_ERR("Failed to publish wbot_moto message");
        }

    } else {
        struct wbot_ctrl_moto_s msg{};
        msg.timestamp = hrt_absolute_time();

        for ( int n = 0; n < 8; n++)
            msg.speed[n] = 0;

        // 发布 topic
        orb_advert_t pub = orb_advertise(ORB_ID(wbot_ctrl_moto), &msg);

        if (pub != nullptr) {
            PX4_INFO("Published wbot_moto message to instance %d", instance);
        } else {
            PX4_ERR("Failed to publish wbot_moto message");
        }
    }



    return 0;
}


