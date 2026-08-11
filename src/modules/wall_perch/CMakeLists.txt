############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__wall_perch
	MAIN wall_perch
	SRCS
		wall_perch.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
	)
