/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file px4fmu2_init.c
 *
 * PX4FMUv2-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>

#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

#include <nuttx/leds/userled.h>

// #include "stm32_ccm.h"

# if defined(FLASH_BASED_PARAMS)
#  include <parameters/flashparams/flashfs.h>
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	// /* set the peripheral rails off */
	// stm32_configgpio(GPIO_VDD_5V_PERIPH_EN);
	// stm32_gpiowrite(GPIO_VDD_5V_PERIPH_EN, 1);

	/* wait for the peripheral rail to reach GND */
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);

	/* re-enable power */

	// /* switch the peripheral rail back on */
	// stm32_gpiowrite(GPIO_VDD_5V_PERIPH_EN, 0);
}

/************************************************************************************
  * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 *
 * status - 1 if resetting to boot loader
 *          0 if just resetting
 *
 ************************************************************************************/

__EXPORT void board_on_reset(int status)
{
	UNUSED(status);

	/* configure the GPIO pins to outputs and keep them low */
	// for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
	// 	px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
	// }

	/* On resets invoked from system (not boot) insure we establish a low
	 * output state (discharge the pins) on PWM pins before they become inputs.
	 *
	 * We also delay the onset of the that 3.1 Ms pulse as boot. This has
	 * triggered some ESC to spin. By adding this delay here the reset
	 * is pushed out > 400 ms. So the ESC PWM input can not mistake
	 * the 3.1 Ms pulse as a valid PWM command.
	 *
	 * fixme:Establish in upstream NuttX an CONFIG_IO_INIT_STATE to
	 * the initialize the IO lines in the clock config.
	 *
	 */

	if (status >= 0) {
		up_mdelay(400);

		/* on reboot (status >= 0) reset sensors and peripherals */
		board_spi_reset(10, 0xffff);
	}
}

/************************************************************************************
  * Name: determin_hw_version
 *
 * Description:
 *
 * This function looks at HW deltas to determine what the
 * build is running on using the following criteria:
 *
 * MSN  PB12   FMUv2                Cube             MINI
 * CAN2_RX       CONECTOR             MX30521          NC
 * PU.PD         1,0                   1,1             1,0
 *
 * LSN  PB4    FMUv2                Cube             MINI
 *               ACCEL_DRDY LSM303D    NC              NC
 * PU.PD         0,0                   1,0             1,0

 *  PB12:PB4
 *  ud   ud
 *  10   00 - 0x8 FMUv2
 *  11   10 - 0xE Cube AKA V2.0
 *  10   10 - 0xA PixhawkMini
 *  10   11 - 0xB FMUv2 questionable hardware (should be treated like regular FMUv2)
 *
 *  This will return OK on success and -1 on not supported
 *
 *  hw_type Initial state is {'V','2',0, 0}
 *   V 2    - FMUv2
 *   V 3 0  - FMUv3 2.0
 *   V 3 1  - FMUv3 2.1 - not differentiateable,
 *   V 2 M  - FMUv2 Mini
 *
 ************************************************************************************/

// static int determin_hw_version(int *version, int *revision)
// {
// 	*revision = 0; /* default revision */
// 	int rv = 0;
// 	int pos = 0;
// 	stm32_configgpio(GPIO_PULLDOWN | (HW_VER_PB4 & ~GPIO_PUPD_MASK));
// 	up_udelay(10);
// 	rv |= stm32_gpioread(HW_VER_PB4) << pos++;
// 	stm32_configgpio(HW_VER_PB4);
// 	up_udelay(10);
// 	rv |= stm32_gpioread(HW_VER_PB4) << pos++;

// 	int votes = 16;
// 	int ones[2] = {0, 0};
// 	int zeros[2] = {0, 0};

// 	while (votes--) {
// 		stm32_configgpio(GPIO_PULLDOWN | (HW_VER_PB12 & ~GPIO_PUPD_MASK));
// 		up_udelay(10);
// 		stm32_gpioread(HW_VER_PB12) ? ones[0]++ : zeros[0]++;
// 		stm32_configgpio(HW_VER_PB12);
// 		up_udelay(10);
// 		stm32_gpioread(HW_VER_PB12) ? ones[1]++ : zeros[1]++;
// 	}

// 	if (ones[0] > zeros[0]) {
// 		rv |= 1 << pos;
// 	}

// 	pos++;

// 	if (ones[1] > zeros[1]) {
// 		rv |= 1 << pos;
// 	}

// 	stm32_configgpio(HW_VER_PB4_INIT);
// 	stm32_configgpio(HW_VER_PB12_INIT);
// 	*version = rv;
// 	return OK;
// }

/************************************************************************************
 * Name: board_get_hw_type_name
 *
 * Description:
 *   Optional returns a string defining the HW type
 *
 *
 ************************************************************************************/

// __EXPORT const char *board_get_hw_type_name()
// {
// 	return (const char *) hw_type;
// }

/************************************************************************************
 * Name: board_get_hw_version
 *
 * Description:
 *   Optional returns a integer HW version
 *
 *
 ************************************************************************************/

// __EXPORT int board_get_hw_version()
// {
// 	return  HW_VER_SIMPLE(hw_version);
// }

/************************************************************************************
 * Name: board_get_hw_revision
 *
 * Description:
 *   Optional returns a integer HW revision
 *
 *
 ************************************************************************************/

// __EXPORT int board_get_hw_revision()
// {
// 	return  hw_revision;
// }

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
	board_on_reset(-1);

	/* configure LEDs */
	board_autoled_initialize();

	/* configure ADC pins */

	// stm32_configgpio(GPIO_ADC1_IN2);	/* BATT_VOLTAGE_SENS */
	// stm32_configgpio(GPIO_ADC1_IN3);	/* BATT_CURRENT_SENS */
	// stm32_configgpio(GPIO_ADC1_IN4);	/* VDD_5V_SENS */
	// stm32_configgpio(GPIO_ADC1_IN13);	/* FMU_AUX_ADC_1 */
	// stm32_configgpio(GPIO_ADC1_IN14);	/* FMU_AUX_ADC_2 */
	// stm32_configgpio(GPIO_ADC1_IN15);	/* PRESSURE_SENS */

	// /* configure power supply control/sense pins */
	// stm32_configgpio(GPIO_VDD_5V_PERIPH_EN);
	// board_control_spi_sensors_power_configgpio();
	// board_control_spi_sensors_power(true, 0xffff);
	// stm32_configgpio(GPIO_VDD_BRICK_VALID);
	// stm32_configgpio(GPIO_VDD_SERVO_VALID);
	// stm32_configgpio(GPIO_VDD_USB_VALID);
	// stm32_configgpio(GPIO_VDD_5V_HIPOWER_OC);
	// stm32_configgpio(GPIO_VDD_5V_PERIPH_OC);

	/*
	 * CAN GPIO config.
	 * Forced pull up on CAN2 is required for FMUv2  where the second interface lacks a transceiver.
	 * If no transceiver is connected, the RX pin will float, occasionally causing CAN controller to
	 * fail during initialization.
	 */
	// stm32_configgpio(GPIO_CAN1_RX);
	// stm32_configgpio(GPIO_CAN1_TX);
	// stm32_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
	// stm32_configgpio(GPIO_CAN2_TX);

	// ccm_initialize();

}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

// static struct spi_dev_s *spi1;
// static struct spi_dev_s *spi2;
static struct spi_dev_s *spi4;
static struct spi_dev_s *spi5;
static struct sdio_dev_s *sdio;

__EXPORT int board_app_initialize(uintptr_t arg)
{
	syslog(LOG_INFO, "board_app_initialize\n");
	/* Ensure the power is on 1 ms before we drive the GPIO pins */
	usleep(1000);



	px4_platform_init();

	/* configure SPI interfaces (after the hw is determined) */
	syslog(LOG_INFO, "stm32_spiinitialize\n");
	stm32_spiinitialize();


	/* initial LED state */

	// This will create a character device for LED, under /dev/ledx
	// it will call led_init() in the constructor
	drv_led_start();

	// int ret = userled_lower_initialize("/dev/userleds");
	// if (ret < 0)
	// {
	// 	syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
	// }




	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "DMA alloc FAILED\n");
	}

	/* set up the serial DMA polling */
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;

	hrt_call_every(&serial_dma_call,
		       ts_to_abstime(&ts),
		       ts_to_abstime(&ts),
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);



	// if (board_hardfault_init(2, true) != 0) {
	// 	led_on(LED_AMBER);
	// }

	/* Configure SPI-based devices */

	// spi1 = stm32_spibus_initialize(1);

	// if (!spi1) {
	// 	syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", 1);
	// 	led_on(LED_AMBER);
	// 	return -ENODEV;
	// }

	// /* Default SPI1 to 1MHz and de-assert the known chip selects. */
	// SPI_SETFREQUENCY(spi1, 10000000);
	// SPI_SETBITS(spi1, 8);
	// SPI_SETMODE(spi1, SPIDEV_MODE3);
	// up_udelay(20);

	// /* Get the SPI port for the FRAM */

	// spi2 = stm32_spibus_initialize(2);

	// if (!spi2) {
	// 	syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", 2);
	// 	led_on(LED_AMBER);
	// 	return -ENODEV;
	// }

	// /* Default SPI2 to 37.5 MHz (40 MHz rounded to nearest valid divider, F4 max)
	//  * and de-assert the known chip selects. */

	// // XXX start with 10.4 MHz in FRAM usage and go up to 37.5 once validated
	// SPI_SETFREQUENCY(spi2, 12 * 1000 * 1000);
	// SPI_SETBITS(spi2, 8);
	// SPI_SETMODE(spi2, SPIDEV_MODE3);

	spi4 = stm32_spibus_initialize(4);

	if (!spi4) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", 4);
		led_on(LED_AMBER);
		return -ENODEV;
	}

	// /* Default SPI4 to 1MHz and de-assert the known chip selects. */
	// SPI_SETFREQUENCY(spi4, 10000000);
	// SPI_SETBITS(spi4, 8);
	// SPI_SETMODE(spi4, SPIDEV_MODE3);

	spi5 = stm32_spibus_initialize(5);

	if (!spi5) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", 5);
		led_on(LED_AMBER);
		return -ENODEV;
	}

#ifdef CONFIG_MMCSD
	/* First, get an instance of the SDIO interface */

	sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);

	if (!sdio) {
		led_on(LED_AMBER);
		syslog(LOG_ERR, "[boot] Failed to initialize SDIO slot %d\n", CONFIG_NSH_MMCSDSLOTNO);
		return -ENODEV;
	}

	/* Now bind the SDIO interface to the MMC/SD driver */
	int ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);

	if (ret != OK) {
		led_on(LED_AMBER);
		syslog(LOG_ERR, "[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	/* Then let's guess and say that there is a card in the slot. There is no card detect GPIO. */
	sdio_mediachange(sdio, true);
#else
	(void)sdio;

#endif

	/*
	 * Bootloader(sector 0):
	 * start: 0x08000000, len: 16K, end: 0x08003E80
	 *
	 * FlashFS(sector 1 and 2):
	 * start: 0x08004000, len: 32K, end: 0x0800BFFF
	 *
	 * Firmware(sector 3 to 11):
	 * start: 0x0800C000, len: 976K, end: 0x080FFFFF
	 *
	 * Dataman(sector 23):
	 * start: 0x081E0000, len: 128K, end: 0x08200000
	 *
	 * First 1MB memory bank complete assigned.
	 * Second 1MB memory bank is mostly empty.
	 */

#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t params_sector_map[] = {
		{1, 16 * 1024, 0x08004000},
		{2, 16 * 1024, 0x08008000},
		{0, 0, 0},
	};

	/* Initialize the flashfs layer to use heap allocated memory */
	int result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		syslog(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
		led_on(LED_AMBER);
		return -ENODEV;
	}

#endif

#if defined(FLASH_BASED_DATAMAN)
	static dm_sector_descriptor_t dm_sector_map = {23, 128 * 1024, 0x081E0000};
	dm_flash_sector_description_set(&dm_sector_map);
#endif

	return OK;
}
