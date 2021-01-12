
#define CAN_DEVPATH "/dev/can0"
#define CAN_OFLAGS O_RDWR

#include "djican.hpp"

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <drivers/drv_hrt.h>

using namespace time_literals;

DjiCan::DjiCan() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

DjiCan::~DjiCan()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool DjiCan::init()
{
	ScheduleOnInterval(100_ms); // 10Hz rate

	return true;
}

int DjiCan::start_can()
{
	PX4_INFO("CAN driver start reading");
	struct canioc_bittiming_s bt;

	struct can_msg_s rxmsg;

	size_t msgsize;
  size_t nbytes;
  long nmsgs    = CONFIG_EXAMPLES_CAN_NMSGS;
  long msgno;
  int fd;
  int errval    = 0;
  int ret;

	/* Initialization of the CAN hardware is performed by board-specific,
	* logic external prior to running this test.
	*/

  /* Open the CAN device for reading */

  fd = open(CAN_DEVPATH, CAN_OFLAGS);
  if (fd < 0)
	{
		printf("ERROR: open %s failed: %d\n",
						CAN_DEVPATH, errno);
		errval = 2;
		goto errout_with_dev;
	}

	/* Show bit timing information if provided by the driver.  Not all CAN
	* drivers will support this IOCTL.
	*/

  ret = ioctl(fd, CANIOC_GET_BITTIMING, (unsigned long)((uintptr_t)&bt));

	if (ret < 0)
	{
		printf("Bit timing not available: %d\n", errno);
	}
  else
	{
		printf("Bit timing:\n");
		printf("   Baud: %lu\n", (unsigned long)bt.bt_baud);
		printf("  TSEG1: %u\n", bt.bt_tseg1);
		printf("  TSEG2: %u\n", bt.bt_tseg2);
		printf("    SJW: %u\n", bt.bt_sjw);
	}

	for (msgno = 0; !nmsgs || msgno < nmsgs; msgno++)
	{
		msgsize = sizeof(struct can_msg_s);
		nbytes = read(fd, &rxmsg, msgsize);
		if (nbytes < CAN_MSGLEN(0) || nbytes > msgsize)
		{
			printf("ERROR: read(%ld) returned %ld\n",
							(long)msgsize, (long)nbytes);
			errval = 4;
			goto errout_with_dev;
		}

		// show raw data as debug message
		printf("  ID: %4u DLC: %u\n",
						rxmsg.cm_hdr.ch_id, rxmsg.cm_hdr.ch_dlc);
		printf("Data0: %d\n", rxmsg.cm_data[0]);
		printf("Data1: %d\n", rxmsg.cm_data[1]);
		printf("Data2: %d\n", rxmsg.cm_data[2]);
		printf("Data3: %d\n", rxmsg.cm_data[3]);
		printf("Data4: %d\n", rxmsg.cm_data[4]);
		printf("Data5: %d\n", rxmsg.cm_data[5]);
		printf("Data6: %d\n", rxmsg.cm_data[6]);
		printf("Data7: %d\n", rxmsg.cm_data[7]);

		// publish can motor data
		can_motor_s data{};
		data.timestamp = hrt_absolute_time();
		data.mech_angle = (rxmsg.cm_data[0] << 8 | rxmsg.cm_data[1]);
		data.rot_speed = (rxmsg.cm_data[2] << 8 | rxmsg.cm_data[3]);
		data.torque = (rxmsg.cm_data[4] << 8 | rxmsg.cm_data[5]);
		data.temp = rxmsg.cm_data[6];

		_orb_can_pub.publish(data);
	}

errout_with_dev:
  close(fd);

  printf("Terminating!\n");
  fflush(stdout);

	return errval;
}

void DjiCan::Run()
{
	PX4_INFO("CAN driver running routine");
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// code to start can communication
	this->start_can();

	uint64_t time_now = hrt_absolute_time();
	PX4_INFO("%llu\n", time_now); // this should print out from syslog


	perf_end(_loop_perf);
}

int DjiCan::task_spawn(int argc, char *argv[])
{
	PX4_INFO("djican spawn task");
	DjiCan *instance = new DjiCan();
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
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

int DjiCan::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int DjiCan::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DjiCan::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Start can driver communication.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("djican", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" int djican_main(int argc, char *argv[])
{
	PX4_INFO("CAN driver start");
	return DjiCan::main(argc, argv);
}
