px4_add_module(
	MODULE wbot_actuator_bridge
	MAIN wbot_actuator_bridge
	SRCS
		wbot_actuator_bridge.cpp
	DEPENDS
		px4_work_queue
	)
