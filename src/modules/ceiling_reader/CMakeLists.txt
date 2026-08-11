############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__CeilingReader
	MAIN CeilingReader
	SRCS
		CeilingReader.cpp
		CeilingReader.hpp
	DEPENDS
		px4_work_queue
		uorb_msgs
	)
