#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#define MODULE_NAME "pitch_servo"

class PitchServoDriver : public px4::ScheduledWorkItem, public ModuleBase<PitchServoDriver> {
public:
	PitchServoDriver();
	~PitchServoDriver() override;

	int init();
	void Run() override;

	static int task_spawn(int argc, char *argv[]);
	static PitchServoDriver *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
private:
	// Manual debugging
	float _manual_servo_value{0.0f};
	bool _manual_mode{false};

	// uORB subscriptions
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};

	// uORB publications
	uORB::Publication<actuator_servos_s> _actuator_servo_pub{ORB_ID(actuator_servos)};

	// Data storage
	vehicle_attitude_s _attitude{};
	actuator_servos_s _actuator_servo{};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	// PWM output state
	void update_servo_output();
};
