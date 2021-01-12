#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/can_motor.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

class DjiCan : public ModuleBase<DjiCan>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DjiCan();
	~DjiCan() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	int start_can();

private:
	void Run() override;

	uORB::Publication<can_motor_s> _orb_can_pub{ORB_ID(can_motor)};

	// uORB::SubscriptionData<sensor_accel_s> _sensor_accel_sub{ORB_ID(sensor_accel)};

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
