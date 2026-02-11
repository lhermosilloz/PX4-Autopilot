#include "pitch_servo.hpp"

// Add these missing includes:
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace time_literals;

PitchServoDriver::PitchServoDriver() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// Initialize actuator outputs structure
	_actuator_servo.timestamp = 0;
	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		_actuator_servo.control[i] = NAN;
	}
}

PitchServoDriver::~PitchServoDriver()
{
	perf_free(_loop_perf);
}

int PitchServoDriver::init()
{
	// Start the work queue task
	ScheduleNow();

	return PX4_OK;
}

void PitchServoDriver::Run()
{
	// Performance monitoring
	perf_begin(_loop_perf);

	// Check if attitude data is available
	if (_attitude_sub.update(&_attitude)) {
		// Call our servo update function
		update_servo_output();
	}

	ScheduleDelayed(2500);	// Run at 400 Hz

	perf_end(_loop_perf);
}

void PitchServoDriver::update_servo_output()
{
	// Get current attitude quaternion and convert to Euler angles
	matrix::Eulerf euler = matrix::Quatf(_attitude.q);

	// Get pitch angle (in radians)
	float current_pitch_rad = euler.theta();  // theta = pitch

	// Formula for linear interpolation: actuator_cmd = -1.06 * drone_pitch - 0.8
	float servo_command = -1.06f * current_pitch_rad - 0.8f;

	// Constrain servo command from (-0.8 to 0.5)
	servo_command = math::constrain(servo_command, -0.8f, 0.5f);

	// In update_servo_output():
	_actuator_servo.timestamp = hrt_absolute_time();
	_actuator_servo.control[0] = servo_command;  // Servo 1 = index 0
	_actuator_servo_pub.publish(_actuator_servo);
}

int PitchServoDriver::task_spawn(int argc, char *argv[]) {
	PitchServoDriver *instance = new PitchServoDriver();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

PitchServoDriver *PitchServoDriver::instantiate(int argc, char *argv[]) {
	return new PitchServoDriver();
}

int PitchServoDriver::custom_command(int argc, char *argv[]) {
	PX4_INFO("custom command called with %d args", argc);
	for (int i = 0; i < argc; i++) {
		PX4_INFO("argv[%d]: %s", i, argv[i]);
	}

	// Get the running instance
    	PitchServoDriver *instance = (PitchServoDriver *)_object.load();

	if (!instance) {
		PX4_ERR("Driver not running");
		return PX4_ERROR;
	}

	/*
	// Add more functionality later, for example: manual set, auto mode toggling, etc.

	if (argc >= 2) {
		float value = atof(argv[1]);
		instance->_manual_servo_value = value;
		instance->_manual_mode = true;  // Also need to set manual mode
		PX4_INFO("Set manual servo to: %.2f", (double)instance->_manual_servo_value);
		return 0;
	}

	*/

	return print_usage("unknown command!");
}

int PitchServoDriver::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
	R"DESCR_STR(
	### Description
	Pitch servo gimbal stabilizer driver.

	Automatically adjusts servo position to counter aircraft pitch movements,
	keeping a gimbal level relative to the horizon.

	)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("pitch_servo", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");
    PRINT_MODULE_USAGE_COMMAND("set <value>");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

// Module entry point
extern "C" __EXPORT int pitch_servo_main(int argc, char *argv[]) {
	return ModuleBase<PitchServoDriver>::main(argc, argv);
}
