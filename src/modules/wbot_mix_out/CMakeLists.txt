
px4_add_module(
	MODULE wbot_mix_out
	MAIN wbot_mix_out
	COMPILE_FLAGS
		-DPARAM_PREFIX="WOUT"
	SRCS
		wbot_mix_out.cpp
		wbot_mix_out.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
		mixer_module
	)
