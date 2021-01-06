/**
 * @file pwm_main.c
 * Main function file for PWM module
 *
 * @author Chen Tong
 */

#include <px4_platform_common/log.h>

__EXPORT int pwm_main(int argc, char *argv[]);

int pwm_main(int argc, char *argv[])
{
	PX4_INFO("Hello pwm!");
	return OK;
}
