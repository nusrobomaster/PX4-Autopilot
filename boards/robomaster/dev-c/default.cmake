# RoboMaster Dev C Development Board has chip F407XXX

px4_add_board(
	PLATFORM nuttx
	VENDOR robomaster
	MODEL dev-c
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT robomaster_common
	# IO px4_io-v2_default
	TESTING
	# UAVCAN_INTERFACES 2
	SERIAL_PORTS
		URT6:/dev/ttyS0
	DRIVERS
		# adc
		# barometer # all available barometer drivers
		# batt_smbus
		# camera_capture
		# camera_trigger
		# differential_pressure # all available differential pressure drivers
		# distance_sensor # all available distance sensor drivers
		# dshot
		# gps
		# #heater
		imu/bosch/bmi088
		# irlock
		# lights/blinkm
		# lights/rgbled
		# lights/rgbled_ncp5623c
		magnetometer/ist8310 # for robomaster dev board c
		# mkblctrl
		# #optical_flow # all available optical flow drivers
		# optical_flow/px4flow
		#osd
		# pca9685
		# #power_monitor/ina226
		# #protocol_splitter
		# pwm_input
		# pwm_out_sim
		pwm_out
		# px4io
		# roboclaw
		# tap_esc
		# telemetry # all available telemetry drivers
		# test_ppm
		tone_alarm
		# uavcan
	MODULES
		commander
		attitude_estimator_q
		dataman
		ekf2
		events
		# fw_att_control
		# fw_pos_control_l1
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		# logger
		mavlink
		# mc_att_control
		# mc_hover_thrust_estimator
		# mc_pos_control
		# mc_rate_control
		# #micrortps_bridge
		navigator
		rc_update
		rover_pos_control
		sensors
		# sih
		# temperature_compensation
		# vmount
		# vtol_att_control
	SYSTEMCMDS
		# bl_update
		#dmesg
		# dumpfile
		# esc_calib
		# gpio
		# hardfault_log
		i2cdetect
		# led_control
		# mixer
		# motor_ramp
		# motor_test
		# mtd
		# nshterm
		param
		# perf
		# pwm
		reboot
		# reflect
		# sd_bench
		# tests # tests and test runner
		top
		topic_listener
		tune_control
		# usb_connected
		# ver
		work_queue
	EXAMPLES
		# fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		hello
		# hwtest # Hardware test
		# #matlab_csv_serial
		px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		# px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		# # rover_steering_control # Rover example app
		# uuv_example_app
		work_item
	)
