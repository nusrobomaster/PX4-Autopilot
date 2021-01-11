/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "move_motor.h"
// #include "modules/rover_pos_control/RoverPositionControl.hpp"
// #include "drivers/pwm_out/PWMOut.hpp"

#include <parameters/param.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <sys/ioctl.h>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
// #include <uORB/topics/actuator_controls.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/pwm_input.h>
// #include <uORB/topics/manual_control_setpoint.h>
#include <drivers/drv_hrt.h>
// #include <drivers/drv_pwm_output.h>

int Module::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Module::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command"); // custom function, not the printf statement
}


int Module::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Module *Module::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Module *instance = new Module(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Module::Module(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void Module::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_combined_sub, 1000);

	//creating struct for pwm before advertising
	struct pwm_input_s pwm = {};
	// advertising the pwm topic
	orb_advert_t pwm_pub = orb_advertise(ORB_ID(pwm_input), &pwm);
	//subscribing to the pwm_input topic on the loop
	int pwm_input_sub = orb_subscribe(ORB_ID(pwm_input));
	orb_set_interval(pwm_input_sub, 1000);

	//trying out actuator_controls
	// struct actuator_controls_s act_controls = {}; 
	// orb_advert_t _actuator_controls_pub = orb_advertise(ORB_ID(actuator_controls_0), &act_controls);

	//manual control
	// struct manual_control_setpoint_s _manual_control_setpoint{};	
	// int mc_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	// orb_advert_t _mc_pub = orb_advertise(ORB_ID(manual_control_setpoint), &_manual_control_setpoint);


	px4_pollfd_struct_t fds[2]; // later change this to 2 so that they can subscribe to more polls
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;
	fds[1].fd = pwm_input_sub;
	fds[1].events = POLLIN; 

	// initialize parameters
	parameters_update(true);

	int decay = 2000;

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			// TODO: do something with the data...

				PX4_INFO("Orginal Value: %d", decay);
				if(decay<1000){
					PX4_INFO("The decay has stopped for the gimbal.");
					break;
				}
				decay -= 50; 
				PX4_INFO("New Value: %d", decay);
				// act_controls.timestamp = hrt_absolute_time();
				// act_controls.control[0] = 2000;
				// act_controls.control[1] = 2000;
				// act_controls.control[2] = 2000;
				// act_controls.control[3] = 2000;
				// act_controls.control[4] = 2000;
				// act_controls.control[5] = 2000;
				// act_controls.control[6] = 2000;
				// act_controls.control[7] = 2000;
				// _manual_control_setpoint.timestamp = hrt_absolute_time();
				// _manual_control_setpoint.x = decay;
				// _manual_control_setpoint.y = decay;
				// _manual_control_setpoint.z = decay;
				// _manual_control_setpoint.r = decay;
				pwm.timestamp = hrt_absolute_time();
				pwm.pulse_width = decay;
				pwm.period = 200;
				orb_publish(ORB_ID(pwm_input), pwm_pub, &pwm);
				// orb_publish(ORB_ID(actuator_controls_0), _actuator_controls_pub, &act_controls);
				// orb_publish(ORB_ID(manual_control_setpoint), _mc_pub, &_manual_control_setpoint);

		}

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
}

void Module::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int Module::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.
This will Work now.
### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int move_motor_main(int argc, char *argv[])
{
	return Module::main(argc, argv);
}
