# 2D LiDAR gimbal servo stablizer

px4_add_module(
	MODULE drivers__pitch_servo
	MAIN pitch_servo
	COMPILE_FLAGS
	SRCS
		pitch_servo.cpp
	DEPENDS
		px4_work_queue
		uORB
		mathlib
)
