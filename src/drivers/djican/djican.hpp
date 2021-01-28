#pragma once

#include <lib/perf/perf_counter.h>
#include <lib/cdev/CDev.hpp>
#include <drivers/drv_mixer.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/mixer_module/mixer_module.hpp>
#include <uORB/topics/can_motor.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

using namespace time_literals;

class DjiCan : public cdev::CDev, public ModuleBase<DjiCan>, public OutputModuleInterface
{
public:
	DjiCan();
	virtual ~DjiCan();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	virtual int	init();

	int print_status() override;

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	int start_can();

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	void Run() override;

	uint32_t	_backup_schedule_interval_us{1_s};

	uORB::Publication<can_motor_s> _orb_can_pub{ORB_ID(can_motor)};

	MixingOutput _mixing_output{4, *this, MixingOutput::SchedulingPolicy::Auto, true};

	int		_class_instance{-1};

	int		can_ioctl(file *filp, int cmd, unsigned long arg);

	void 	send_motor_data(size_t channel, uint16_t output_data);

	// uORB::SubscriptionData<sensor_accel_s> _sensor_accel_sub{ORB_ID(sensor_accel)};

	// perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	// perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	perf_counter_t	_cycle_perf;
	perf_counter_t	_interval_perf;
};
