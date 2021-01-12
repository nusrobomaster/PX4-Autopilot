px4_add_module(
	MODULE drivers__djican
	MAIN djican
	SRCS
		djican.cpp
	DEPENDS
		px4_work_queue
	)
