
#define CAN_DEVPATH "/dev/can_output"
#define PX4FMU_DEVICE_PATH "/dev/px4fmu"
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

#define CAN_MOTOR_CURRENT_MIN 0

// for now uses work queue configuration: test1
DjiCan::DjiCan() :
	CDev(PX4FMU_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::test1),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
	_mixing_output.setAllMinValues(CAN_MOTOR_CURRENT_MIN);
	_mixing_output.setAllMaxValues(0);
}

DjiCan::~DjiCan()
{
	/* clean up the alternate device node */
	unregister_class_devname(CAN_DEVPATH, _class_instance);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int DjiCan::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(CAN_DEVPATH);

	if (_class_instance == 0) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	ScheduleNow();

	return 0;
}

int DjiCan::start_can()
{
	PX4_INFO("CAN driver start reading");
	struct canioc_bittiming_s bt;

	struct can_msg_s rxmsg;

	size_t msgsize;
	size_t nbytes;
	long nmsgs    = 20;
	long msgno;
	int fd;
	int errval    = 0;
	int ret;

	/* Initialization of the CAN hardware is performed by board-specific,
	* logic external prior to running this test.
	*/

	/* Open the CAN device for reading
	* Will hang if not connected to anything: to fix soon
	*/

	fd = ::open("/dev/can0", CAN_OFLAGS);
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

  	ret = ::ioctl(fd, CANIOC_GET_BITTIMING, (unsigned long)((uintptr_t)&bt));

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
		nbytes = ::read(fd, &rxmsg, msgsize);
		if (nbytes < CAN_MSGLEN(0) || nbytes > msgsize)
		{
			printf("ERROR: read(%ld) returned %ld\n",
							(long)msgsize, (long)nbytes);
			errval = 4;
			goto errout_with_dev;
		}

		PX4_INFO("Motor number: %d", rxmsg.cm_hdr.ch_id);

		// show raw data as debug message
		// printf("  ID: %4u DLC: %u\n",
		// 				rxmsg.cm_hdr.ch_id, rxmsg.cm_hdr.ch_dlc);
		// printf("Data0: %d\n", rxmsg.cm_data[0]);
		// printf("Data1: %d\n", rxmsg.cm_data[1]);
		// printf("Data2: %d\n", rxmsg.cm_data[2]);
		// printf("Data3: %d\n", rxmsg.cm_data[3]);
		// printf("Data4: %d\n", rxmsg.cm_data[4]);
		// printf("Data5: %d\n", rxmsg.cm_data[5]);
		// printf("Data6: %d\n", rxmsg.cm_data[6]);
		// printf("Data7: %d\n", rxmsg.cm_data[7]);

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
	::close(fd);

	printf("Terminating!\n");
	fflush(stdout);

	return errval;
}

void DjiCan::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// push backup schedule
	ScheduleDelayed(_backup_schedule_interval_us);

	_mixing_output.update();

	// code to start can communication
	//this->start_can();

	// uint64_t time_now = hrt_absolute_time();
	// PX4_INFO("%llu\n", time_now); // this should print out from syslog

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true, true);

	perf_end(_cycle_perf);
}

int DjiCan::task_spawn(int argc, char *argv[])
{
	DjiCan *instance = new DjiCan();
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

int DjiCan::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
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
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "listen to can motor signals");
	PRINT_MODULE_USAGE_COMMAND_DESCR("send", "send data to can motor");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

bool DjiCan::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	/* output to the motors */
	// must set value for each individual motors
	for (size_t i = 0; i < num_outputs; i++) {
		// send message to can motor
		send_motor_data(i, outputs[i]);
	}
	// PX4_INFO("subscribed actuator control0: %d", _mixing_output._control_subs[0].control);
	_mixing_output.printStatus();
	return true;
}

void DjiCan::send_motor_data(size_t channel, uint16_t output_data) {
	if (output_data != 0) {
		PX4_INFO("Data for channel %d is: %d", channel, output_data);
	}
}

static void send_can(char* string_value)
{
	PX4_INFO("Starting to send data: %s", string_value);
	struct can_msg_s txmsg;
	int msgdlc = 8;
	size_t msgsize;
	int fd;
	size_t nbytes;

	fd = open(CAN_DEVPATH, CAN_OFLAGS);
	if (fd < 0)
	{
		printf("ERROR: open %s failed: %d\n",
			CAN_DEVPATH, errno);
	}

	/* Construct the next TX message */
	txmsg.cm_hdr.ch_id     = 0x200;
	txmsg.cm_hdr.ch_rtr    = true;
	txmsg.cm_hdr.ch_dlc    = msgdlc;

	for (int i = 0; i < msgdlc; i++)
        {
          txmsg.cm_data[i] = 0;
        }

	int16_t actual_value = atoi(string_value);

	// hard wired to only write to motor device id 2
	txmsg.cm_data[2] = (actual_value >> 8);
	txmsg.cm_data[3] = (actual_value & 0xff);

	/* Send the TX message */

	msgsize = CAN_MSGLEN(msgdlc);
	nbytes = write(fd, &txmsg, msgsize);
	if (nbytes != msgsize)
	{
		PX4_INFO("ERROR: write(%ld) returned %ld\n",
			(long)msgsize, (long)nbytes);
		// return 3
	}
	PX4_INFO("Value wrote to ESC");
	// return 0;

	close(fd);
}

int DjiCan::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;
	ret = can_ioctl(filp, cmd, arg);
	return ret;
}

int DjiCan::can_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_INFO("ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();
		break;
	case MIXERIOCLOADBUF: {
		const char *buf = (const char *)arg;
		unsigned buflen = strlen(buf);
		ret = _mixing_output.loadMixerThreadSafe(buf, buflen);

		break;
	}
	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

extern "C" int djican_main(int argc, char *argv[])
{
	PX4_INFO("%d", strcmp(argv[1], "send"));
	if (strcmp(argv[1], "send") == 0) {
		send_can(argv[2]);
		return 0;
	}
	return DjiCan::main(argc, argv);
}
