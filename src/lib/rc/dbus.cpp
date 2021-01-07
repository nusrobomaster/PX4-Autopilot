#include <px4_platform_common/px4_config.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "dbus.h"

#include <debug.h>

uint8_t inputBuffer[18] = {};
int prev = 0;

bool
dbus_parse(uint64_t now, uint8_t *frame, unsigned len, uint16_t *values,
	   uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, unsigned *frame_drops, uint16_t max_channels)
{

	if (len != 18 && prev == 0){
		for(unsigned i = 0; i < len; i++){
			inputBuffer[i] = frame[i];
			prev = len;
		}
		return false;
	}
	else if(len + prev == 18 && prev != 0){
		for(int i = prev; i < 18; i ++){
			inputBuffer[i] = frame[i - prev];
		}
		prev = 0;
	}
	else if(len + prev > 18){
		prev = 0;
		return false;
	}
	else if(len == 18){
		for(int i = 0; i < 18; i++){
			inputBuffer[i] = frame[i];
		}
		prev = 0;
	}

	// convert raw data into meaningful data
	values[0] = ((int16_t)inputBuffer[0] | ((int16_t)inputBuffer[1] << 8)) & 0x07FF;
	values[0] = uint16_t(0.758 * double(values[0]) + 624.242);
	values[1] = (((int16_t)inputBuffer[1] >> 3) | ((int16_t)inputBuffer[2] << 5)) & 0x07FF;
	values[1] = uint16_t(0.758 * double(values[1]) + 624.242);
	values[2] = (((int16_t)inputBuffer[2] >> 6) | ((int16_t)inputBuffer[3] << 2) |
		((int16_t)inputBuffer[4] << 10)) & 0x07FF;
	values[2] = uint16_t(0.758 * double(values[2]) + 624.242);
	values[3] = (((int16_t)inputBuffer[4] >> 1) | ((int16_t)inputBuffer[5] << 7)) & 0x07FF;
	values[3] = uint16_t(0.758 * double(values[3]) + 624.242);



	values[4] = ((inputBuffer[5] >> 4) & 0x000C) >> 2;
	values[4] =  500 * values[4] + 400;
	values[5] = ((inputBuffer[5] >> 4) & 0x0003);
	values[5] =  500 * values[5] + 400;

	//syslog(LOG_INFO, "%u %u %u %u %u %u\n", values[0], values[1], values[2], values[3], values[4] , values[5]);

	// RC_CtrlData.mouse.x = ((int16_t)inputBuffer[6]) | ((int16_t)inputBuffer[7] << 8);
	// RC_CtrlData.mouse.y = ((int16_t)inputBuffer[8]) | ((int16_t)inputBuffer[9] << 8);
	// RC_CtrlData.mouse.z = ((int16_t)inputBuffer[10]) | ((int16_t)inputBuffer[11] << 8);

	// RC_CtrlData.mouse.press_l = inputBuffer[12];
	// RC_CtrlData.mouse.press_r = inputBuffer[13];

	values[6] = ((int16_t)inputBuffer[16]) | ((int16_t)inputBuffer[17] << 8);
	values[6] = uint16_t(0.758 * double(values[6]) + 624.242);

	*num_values = 7;


	*sbus_failsafe = false;
	*sbus_frame_drop = 0;
	return true;
}
