/*
 * Board support file for OMAP4430 based NotleBoard.
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: David Anders <x0132446@ti.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "board-notle.h"
#include "notle-usb-mux.h"

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/hwspinlock.h>
#include <linux/usb/otg.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/memblock.h>
#include <linux/omapfb.h>
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/reboot.h>
#include <linux/spi/spi.h>
#include <plat/mcspi.h>
#include <linux/mfd/twl6040.h>
#include <linux/gps_elton.h>
#include <linux/platform_data/omap-abe-twl6040.h>

#ifdef CONFIG_INPUT_LTR506ALS
#include <linux/i2c/ltr506als.h>
#endif
#include <linux/mpu.h>

#ifdef CONFIG_INPUT_GLASSHUB
#include <linux/i2c/glasshub.h>
#endif

#include <linux/mpu.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

/* Synaptics touchpad driver. */
#if defined(CONFIG_RMI4_BUS)
#include <linux/rmi.h>
#endif

#include <mach/hardware.h>
#include "common.h"

#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <plat/android-display.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/drm.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/remoteproc.h>
#include <plat/vram.h>
#include <plat/omap-pm.h>
#include <plat/omap-serial.h>
#include <plat/omap_apps_brd_id.h>

#include <linux/power/bq27x00_battery.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <video/omap-panel-notle.h>

#include "omap_ram_console.h"
#include "hsmmc.h"
#include "control.h"
#include "pm.h"
#include "common-board-devices.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "cm1_44xx.h"
#include "iomap.h"
#include "omap4_ion.h"

#define RX_TIMEOUT			(3 * HZ) /* RX DMA timeout (jiffies) */

#define DEFAULT_RXDMA_TIMEOUT	(3 * HZ)	/* RX DMA timeout (jiffies) */
#define DEFAULT_RXDMA_POLLRATE	1		/* RX DMA polling rate (us) */
#define DEFAULT_RXDMA_BUFSIZE	4096		/* RX DMA buffer size */
#define DEFAULT_AUTOSUSPEND_DELAY	3000	/* Runtime autosuspend (msecs)*/

static notle_version NOTLE_VERSION = UNVERSIONED;

static int notle_gpio_board_evt1[GPIO_MAX_INDEX] = {
    [GPIO_MPU9000_INT_TIMER_INDEX] = GPIO_MPU9000_INT_TIMER_EVT1,
    [GPIO_MPU9000_INT_INDEX] = GPIO_MPU9000_INT_EVT1,
    [GPIO_USB_MUX_CB0_INDEX] = GPIO_USB_MUX_CB0_EVT1,
    [GPIO_USB_MUX_CB1_INDEX] = GPIO_USB_MUX_CB1,
    [GPIO_GPS_ON_OFF_INDEX] = GPIO_GPS_ON_OFF_EVT1,
    [GPIO_GPS_RESET_N_INDEX] = GPIO_GPS_RESET_N_EVT1,
    [GPIO_LCD_RST_N_INDEX] = GPIO_LCD_RST_N_EVT1,
    [GPIO_DISP_ENB_INDEX] = GPIO_DISP_ENB,
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT1,
    [GPIO_CAM_PWDN_INDEX] = GPIO_CAM_PWDN_EVT1,
    [GPIO_TOUCHPAD_INT_N_INDEX] = GPIO_TOUCHPAD_INT_N_EVT1,
    [GPIO_PROX_INT_INDEX] = GPIO_PROX_INT_EVT1,
    [GPIO_BLINK_INT_INDEX] = GPIO_PROX_INT_EVT1,    // shared in evt1
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT1,
    [GPIO_BCM_BT_HOST_WAKE_INDEX] = GPIO_BCM_BT_HOST_WAKE_EVT1,
    [GPIO_BCM_WLAN_HOST_WAKE_INDEX] = GPIO_BCM_WLAN_HOST_WAKE_EVT,
};
static int notle_gpio_board_evt2[GPIO_MAX_INDEX] = {
    [GPIO_MPU9000_INT_INDEX] = GPIO_MPU9000_INT_EVT2,
    [GPIO_USB_MUX_CB0_INDEX] = GPIO_USB_MUX_CB0_EVT2,
    [GPIO_USB_MUX_CB1_INDEX] = GPIO_USB_MUX_CB1,
    [GPIO_GPS_ON_OFF_INDEX] = GPIO_GPS_ON_OFF_EVT2,
    [GPIO_GPS_RESET_N_INDEX] = GPIO_GPS_RESET_N_EVT2,
    [GPIO_GPS_AWAKE_INDEX] = GPIO_GPS_AWAKE_EVT2,
    [GPIO_LCD_RST_N_INDEX] = GPIO_LCD_RST_N_EVT2,
    [GPIO_FPGA_CDONE_INDEX] = GPIO_FPGA_CDONE,
    [GPIO_FPGA_CRESET_B_INDEX] = GPIO_FPGA_CRESET_B,
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT2,
    [GPIO_CAM_PWDN_INDEX] = GPIO_CAM_PWDN_EVT2,
    [GPIO_TOUCHPAD_INT_N_INDEX] = GPIO_TOUCHPAD_INT_N_EVT2,
    [GPIO_PROX_INT_INDEX] = GPIO_PROX_INT_EVT2,
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT2,
    [GPIO_BCM_BT_HOST_WAKE_INDEX] = GPIO_BCM_BT_HOST_WAKE_EVT2,
    [GPIO_BCM_WLAN_HOST_WAKE_INDEX] = GPIO_BCM_WLAN_HOST_WAKE_EVT,
    [GPIO_BLINK_INT_INDEX] = GPIO_BLINK_INT_EVT2,
    [GPIO_SOC_INT_INDEX] = GPIO_SOC_INT_EVT2,
    [GPIO_BAT_LOW_INDEX] = GPIO_BAT_LOW_EVT2,
};

/* Read board version from GPIO.  Result in NOTLE_VERSION. */
static void notle_version_init(void)
{
        int r;

        // mux board version gpio's
        // use low level interface since omap_mux_init has not been called yet
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID2);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID1);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID0);

        r = gpio_request_one(GPIO_BOARD_ID2, GPIOF_IN, "id2");
        if (r) {
                pr_err("Failed to get gpio %d for id2 pin of board version\n", GPIO_BOARD_ID2);
        }

        r = gpio_request_one(GPIO_BOARD_ID1, GPIOF_IN, "id1");
        if (r) {
                pr_err("Failed to get gpio %d for id1 pin of board version\n", GPIO_BOARD_ID1);
        }

        r = gpio_request_one(GPIO_BOARD_ID0, GPIOF_IN, "id0");
        if (r) {
                pr_err("Failed to get gpio %d for id0 pin of board version\n", GPIO_BOARD_ID0);
        }
        NOTLE_VERSION = gpio_get_value(GPIO_BOARD_ID0) | (gpio_get_value(GPIO_BOARD_ID1) << 1)
            | (gpio_get_value(GPIO_BOARD_ID2) << 2);

        gpio_free(GPIO_BOARD_ID0);
        gpio_free(GPIO_BOARD_ID1);
        gpio_free(GPIO_BOARD_ID2);
}


static char * notle_version_str(notle_version board_ver)
{
        switch (board_ver)
        {
        case V1_EVT1:
                return "V1 EVT1";
        case V1_EVT2:
                return "V1 EVT2";
        case V1_EVT3:
                return "V1 EVT3";
        case V1_DVT1:
                return "V1 DVT1";
        case V1_5_PROTO:
                return "V1.5 PROTO";
        default:
                return "UNVERSIONED";
        }
        return "UNVERSIONED";
}

int notle_version_supported() {
        return ( (NOTLE_VERSION>=SUPPORTED_FROM) && (NOTLE_VERSION<=SUPPORTED_TO) );
}

int notle_version_before( notle_version then ) {
        return ( then > NOTLE_VERSION );
}

int notle_version_after( notle_version then ) {
        return ( then < NOTLE_VERSION );
}

int notle_version_support_battery_temperature(void) {
        return (NOTLE_VERSION >= V1_EVT2);
}

#ifdef CONFIG_OMAP4_DUTY_CYCLE_GOVERNOR

static struct pcb_section omap4_duty_governor_pcb_sections[] = {
	{
		.pcb_temp_level			= 25000,
		.max_opp			= 1008000,
		.duty_cycle_enabled		= false,
		.tduty_params = {
			.nitro_rate		= 1008000,
			.cooling_rate		= 800000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 80,
		},
	},
	{
		.pcb_temp_level			= 30000,
		.max_opp			= 1008000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1008000,
			.cooling_rate		= 800000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 37,
		},
	},
	{
		.pcb_temp_level			= 35000,
		.max_opp			= 1008000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1008000,
			.cooling_rate		= 800000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 24,
		},
	},
	{
		.pcb_temp_level			= 40000,
		.max_opp			= 800000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 800000,
			.cooling_rate		= 600000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 19,
		},
	},
	{
		.pcb_temp_level			= 55000,
		.max_opp			= 800000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 800000,
			.cooling_rate		= 600000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 14,
		},
	},
	{
		.pcb_temp_level			= 70000,
		.max_opp			= 600000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 600000,
			.cooling_rate		= 300000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 6,
		},
	},
	{
		.pcb_temp_level			= 75000,
		.max_opp			= 600000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 600000,
			.cooling_rate		= 300000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 3,
		},
	},
	{
		.pcb_temp_level			= 80000,
		.max_opp			= 600000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 600000,
			.cooling_rate		= 300000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 2,
		},
	},

	{
		.pcb_temp_level			= 85000,
		.max_opp			= 600000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 600000,
			.cooling_rate		= 300000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 1,
		},
	},
};

static struct pcb_section omap4_duty_governor_turbo_sprint_pcb_sections[] = {
    {
        .pcb_temp_level         = 65000,
        .max_opp                = 1008000,
        .duty_cycle_enabled     = false,
        .tduty_params = {
            .nitro_rate         = 1008000,
            .cooling_rate       = 800000,
            .nitro_interval     = 20000,
            .nitro_percentage   = 80,
        },
    },
    {
        .pcb_temp_level         = 70000,
        .max_opp                = 600000,
        .duty_cycle_enabled     = true,
        .tduty_params = {
            .nitro_rate         = 800000,
            .cooling_rate       = 600000,
            .nitro_interval     = 20000,
            .nitro_percentage   = 80,
        },
    },
    {
        .pcb_temp_level         = 75000,
        .max_opp                = 600000,
        .duty_cycle_enabled     = true,
        .tduty_params = {
            .nitro_rate         = 600000,
            .cooling_rate       = 300000,
            .nitro_interval     = 20000,
            .nitro_percentage   = 3,
        },
    },
    {
        .pcb_temp_level         = 80000,
        .max_opp                = 600000,
        .duty_cycle_enabled     = true,
        .tduty_params = {
            .nitro_rate         = 600000,
            .cooling_rate       = 300000,
            .nitro_interval     = 20000,
            .nitro_percentage   = 2,
        },
    },

    {
        .pcb_temp_level         = 85000,
        .max_opp                = 600000,
        .duty_cycle_enabled     = true,
        .tduty_params = {
            .nitro_rate         = 600000,
            .cooling_rate       = 300000,
            .nitro_interval     = 20000,
            .nitro_percentage   = 1,
        },
    },
};

void init_duty_governor(void)
{
	omap4_duty_pcb_section_reg(omap4_duty_governor_pcb_sections,
		ARRAY_SIZE(omap4_duty_governor_pcb_sections));
	omap4_duty_turbo_sprint_pcb_section_reg(omap4_duty_governor_turbo_sprint_pcb_sections,
	    ARRAY_SIZE(omap4_duty_governor_turbo_sprint_pcb_sections));
}
#else
void init_duty_governor(void){}
#endif /*CONFIG_OMAP4_DUTY_CYCLE*/

int
notle_get_gpio(int gpio_index)
{
    int ret = -1;

    if (!notle_version_supported()) {
        return ret;
    }

    switch (NOTLE_VERSION) {
    case V1_EVT1:
        ret = notle_gpio_board_evt1[gpio_index];
        break;
    default:
        ret = notle_gpio_board_evt2[gpio_index];
        break;
    }

    // Special case gpio_wk0
    if (ret == 0 && gpio_index != GPIO_BCM_WLAN_HOST_WAKE_INDEX) {
        pr_err("%s:Uninitialized index %d\n", __FUNCTION__, gpio_index);
        ret = -1;
    }
    return(ret);
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "notleboard::status1",
		.default_trigger	= "heartbeat",
		.gpio			= 7,
	},
	{
		.name			= "notleboard::status2",
		.default_trigger	= "mmc0",
		.gpio			= 8,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static void __init notle_init_early(void)
{
  omap4430_init_early();
  init_duty_governor();
}

static int notle_enable_dpi(struct omap_dss_device *dssdev) {
        gpio_set_value(dssdev->reset_gpio, 1);
        return 0;
}

static void notle_disable_dpi(struct omap_dss_device *dssdev) {
        gpio_set_value(dssdev->reset_gpio, 0);
}

static int notle_enable_panel(void) {
        if (NOTLE_VERSION == V1_EVT1) {
            gpio_set_value(GPIO_DISP_ENB, 1);
        }
        return 0;
};

static void notle_disable_panel(void) {
        if (NOTLE_VERSION == V1_EVT1) {
            gpio_set_value(GPIO_DISP_ENB, 0);
        }
};

/*
 * Gamma table where output = input ^ gamma
 * Format of entries is 0xiirrggbb (ii is the index)
 * Hardware and image experiments determined that our default
 * gamma should be set at 1.1
 */
static u32 gamma_11[OMAP_DSS_GAMMA_TABLE_SIZE] = {
    0x00000000, 0x01010101, 0x02010101, 0x03020202,
    0x04030303, 0x05030303, 0x06040404, 0x07050505,
    0x08060606, 0x09060606, 0x0a070707, 0x0b080808,
    0x0c090909, 0x0d0a0a0a, 0x0e0a0a0a, 0x0f0b0b0b,
    0x100c0c0c, 0x110d0d0d, 0x120e0e0e, 0x130f0f0f,
    0x14101010, 0x15101010, 0x16111111, 0x17121212,
    0x18131313, 0x19141414, 0x1a151515, 0x1b161616,
    0x1c161616, 0x1d171717, 0x1e181818, 0x1f191919,
    0x201a1a1a, 0x211b1b1b, 0x221c1c1c, 0x231d1d1d,
    0x241e1e1e, 0x251f1f1f, 0x261f1f1f, 0x27202020,
    0x28212121, 0x29222222, 0x2a232323, 0x2b242424,
    0x2c252525, 0x2d262626, 0x2e272727, 0x2f282828,
    0x30292929, 0x312a2a2a, 0x322a2a2a, 0x332b2b2b,
    0x342c2c2c, 0x352d2d2d, 0x362e2e2e, 0x372f2f2f,
    0x38303030, 0x39313131, 0x3a323232, 0x3b333333,
    0x3c343434, 0x3d353535, 0x3e363636, 0x3f373737,
    0x40383838, 0x41393939, 0x423a3a3a, 0x433b3b3b,
    0x443c3c3c, 0x453d3d3d, 0x463e3e3e, 0x473e3e3e,
    0x483f3f3f, 0x49404040, 0x4a414141, 0x4b424242,
    0x4c434343, 0x4d444444, 0x4e454545, 0x4f464646,
    0x50474747, 0x51484848, 0x52494949, 0x534a4a4a,
    0x544b4b4b, 0x554c4c4c, 0x564d4d4d, 0x574e4e4e,
    0x584f4f4f, 0x59505050, 0x5a515151, 0x5b525252,
    0x5c535353, 0x5d545454, 0x5e555555, 0x5f565656,
    0x60575757, 0x61585858, 0x62595959, 0x635a5a5a,
    0x645b5b5b, 0x655c5c5c, 0x665d5d5d, 0x675e5e5e,
    0x685f5f5f, 0x69606060, 0x6a616161, 0x6b626262,
    0x6c636363, 0x6d646464, 0x6e656565, 0x6f666666,
    0x70676767, 0x71686868, 0x72696969, 0x736a6a6a,
    0x746b6b6b, 0x756c6c6c, 0x766d6d6d, 0x776e6e6e,
    0x786f6f6f, 0x79707070, 0x7a717171, 0x7b727272,
    0x7c737373, 0x7d747474, 0x7e757575, 0x7f767676,
    0x80777777, 0x81797979, 0x827a7a7a, 0x837b7b7b,
    0x847c7c7c, 0x857d7d7d, 0x867e7e7e, 0x877f7f7f,
    0x88808080, 0x89818181, 0x8a828282, 0x8b838383,
    0x8c848484, 0x8d858585, 0x8e868686, 0x8f878787,
    0x90888888, 0x91898989, 0x928a8a8a, 0x938b8b8b,
    0x948c8c8c, 0x958d8d8d, 0x968e8e8e, 0x978f8f8f,
    0x98909090, 0x99919191, 0x9a929292, 0x9b939393,
    0x9c959595, 0x9d969696, 0x9e979797, 0x9f989898,
    0xa0999999, 0xa19a9a9a, 0xa29b9b9b, 0xa39c9c9c,
    0xa49d9d9d, 0xa59e9e9e, 0xa69f9f9f, 0xa7a0a0a0,
    0xa8a1a1a1, 0xa9a2a2a2, 0xaaa3a3a3, 0xaba4a4a4,
    0xaca5a5a5, 0xada6a6a6, 0xaea7a7a7, 0xafa9a9a9,
    0xb0aaaaaa, 0xb1ababab, 0xb2acacac, 0xb3adadad,
    0xb4aeaeae, 0xb5afafaf, 0xb6b0b0b0, 0xb7b1b1b1,
    0xb8b2b2b2, 0xb9b3b3b3, 0xbab4b4b4, 0xbbb5b5b5,
    0xbcb6b6b6, 0xbdb7b7b7, 0xbeb8b8b8, 0xbfbababa,
    0xc0bbbbbb, 0xc1bcbcbc, 0xc2bdbdbd, 0xc3bebebe,
    0xc4bfbfbf, 0xc5c0c0c0, 0xc6c1c1c1, 0xc7c2c2c2,
    0xc8c3c3c3, 0xc9c4c4c4, 0xcac5c5c5, 0xcbc6c6c6,
    0xccc7c7c7, 0xcdc9c9c9, 0xcecacaca, 0xcfcbcbcb,
    0xd0cccccc, 0xd1cdcdcd, 0xd2cecece, 0xd3cfcfcf,
    0xd4d0d0d0, 0xd5d1d1d1, 0xd6d2d2d2, 0xd7d3d3d3,
    0xd8d4d4d4, 0xd9d6d6d6, 0xdad7d7d7, 0xdbd8d8d8,
    0xdcd9d9d9, 0xdddadada, 0xdedbdbdb, 0xdfdcdcdc,
    0xe0dddddd, 0xe1dedede, 0xe2dfdfdf, 0xe3e0e0e0,
    0xe4e1e1e1, 0xe5e3e3e3, 0xe6e4e4e4, 0xe7e5e5e5,
    0xe8e6e6e6, 0xe9e7e7e7, 0xeae8e8e8, 0xebe9e9e9,
    0xeceaeaea, 0xedebebeb, 0xeeececec, 0xefededed,
    0xf0efefef, 0xf1f0f0f0, 0xf2f1f1f1, 0xf3f2f2f2,
    0xf4f3f3f3, 0xf5f4f4f4, 0xf6f5f5f5, 0xf7f6f6f6,
    0xf8f7f7f7, 0xf9f8f8f8, 0xfafafafa, 0xfbfbfbfb,
    0xfcfcfcfc, 0xfdfdfdfd, 0xfefefefe, 0xffffffff
};

/* Using the panel-notle-dpi driver, we only specify enable/disable. */
static struct panel_notle_data panel_notle_preevt2 = {
        .platform_enable          = notle_enable_dpi,
        .platform_disable         = notle_disable_dpi,
        .panel_enable             = notle_enable_panel,
        .panel_disable            = notle_disable_panel,
        .red_max_mw               = 63,
        .green_max_mw             = 192,
        .blue_max_mw              = 96,
        .limit_mw                 = 80,
        .red_percent              = 1667,
        .green_percent            = 5416,
        .blue_percent             = 2917,
        .cpr_enable               = 0,
        .gamma_table              = NULL,
        .gamma_enable             = 0,
};

static struct panel_notle_data panel_notle = {
        .platform_enable          = notle_enable_dpi,
        .platform_disable         = notle_disable_dpi,
        .panel_enable             = notle_enable_panel,
        .panel_disable            = notle_disable_panel,
        .red_max_mw               = 63,
        .green_max_mw             = 96,
        .blue_max_mw              = 96,
        .limit_mw                 = 76,
        .red_percent              = 2444,
        .green_percent            = 4444,
        .blue_percent             = 3111,
        .cpr_enable               = 1,
        .cpr_coefs                = {
            205, 9, 17, 25, 209, 11, 9, -1, 247
        },
        .gamma_table              = gamma_11,
        .gamma_enable             = 1,
};

// gpio line set in board specific code
struct omap_dss_device panel_notle_device = {
        .type                     = OMAP_DISPLAY_TYPE_DPI,
        .name                     = "notle_nhd_panel",
        .driver_name              = "panel_notle",
        .data                     = &panel_notle,
        .phy.dpi.data_lines       = 24,
        .channel                  = OMAP_DSS_CHANNEL_LCD2,
        .vsync_gpio               = -1,
        .panel = {
                .timings = {
                        .x_res = 640,
                        .y_res = 360,
                },
        },
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
        .skip_init              = true,
#else
        .skip_init              = false,
#endif
};

static struct omap2_mcspi_device_config ice40_mcspi_config = {
        .turbo_mode                = 0,
        .single_channel            = 1,  /* 0: slave, 1: master */
};

static struct spi_board_info ice40_spi_board_info[] __initdata = {
        [0] = {
                .modalias                = "ice40-spi",
                .bus_num                 = 1,
                .chip_select             = 0,
                .max_speed_hz            = 48000000,
                .controller_data         = &ice40_mcspi_config,
        },
};

int __init notle_dpi_init(void)
{
        int r;

        if (notle_version_before(V1_EVT2)) {
            r = gpio_request_one(GPIO_DISP_ENB, GPIOF_OUT_INIT_LOW, "disp_enable");
            if (r) {
                    pr_err("Failed to get display enable gpio\n");
                    return r;
            }
        }

        panel_notle_device.reset_gpio = notle_get_gpio(GPIO_LCD_RST_N_INDEX);
        r = gpio_request_one(panel_notle_device.reset_gpio,
                             GPIOF_OUT_INIT_HIGH, "panel_reset");
        if (r) {
                pr_err("Failed to get panel reset powerdown GPIO\n");
                if (notle_version_before(V1_EVT2)) {
                    gpio_free(GPIO_DISP_ENB);
                }
                return r;
        }

        if (notle_version_after(V1_EVT1)) {
                struct panel_notle_data *panel_data = &panel_notle;

                panel_data->gpio_fpga_cdone = notle_get_gpio(GPIO_FPGA_CDONE_INDEX);
                r = gpio_request_one(panel_data->gpio_fpga_cdone,
                                     GPIOF_IN, "fpga_cdone");
                if (r) {
                        pr_err("Failed to get fpga_cdone GPIO\n");
                        gpio_free(panel_notle_device.reset_gpio);
                        if (notle_version_before(V1_EVT2)) {
                            gpio_free(GPIO_DISP_ENB);
                        }
                        return r;
                }

                panel_data->gpio_fpga_creset_b = notle_get_gpio(GPIO_FPGA_CRESET_B_INDEX);
                r = gpio_request_one(panel_data->gpio_fpga_creset_b,
                                     GPIOF_OUT_INIT_HIGH, "fpga_creset_b");
                if (r) {
                        pr_err("Failed to get fpga_creset_b GPIO\n");
                        gpio_free(panel_data->gpio_fpga_cdone);
                        gpio_free(panel_notle_device.reset_gpio);
                        if (notle_version_before(V1_EVT2)) {
                            gpio_free(GPIO_DISP_ENB);
                        }
                        return r;
                }
        }

        if (notle_version_supported()) {
          spi_register_board_info(ice40_spi_board_info,
                                  ARRAY_SIZE(ice40_spi_board_info));
        }

        return 0;
}

static struct omap_dss_device *panel_notle_dss_devices[] = {
        &panel_notle_device,
};

static struct omap_dss_board_info panel_notle_dss_data = {
        .num_devices    = ARRAY_SIZE(panel_notle_dss_devices),
        .devices        = panel_notle_dss_devices,
        .default_device = &panel_notle_device,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode		= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode		= MUSB_PERIPHERAL,
#endif
	.power			= 100,
};

// Need an empty struct or USB won't initialize
// Investigate using the pmic default getter if we can
static struct twl4030_usb_data omap4_usbphy_data = {
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA |  MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
                .no_off_init    = true,
                // TODO(rocky): Experiment with turning this off to see if
                // it improves hsmmc suspend/resume problem.  Measured
                // effect on current draw is zero.
                .power_saving   = false,
	},
        {
                .name           = "bcm4329",
                .mmc            = 5,
                .caps           = MMC_CAP_4_BIT_DATA,
                    // TODO(abliss): | MMC_CAP_POWER_OFF_CARD,
                .gpio_wp        = -EINVAL,
                .gpio_cd        = -EINVAL,
                .ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21,
                .nonremovable   = true,
		.mmc_data	= &tuna_wifi_data,
        },
        /* This device is only present on Dog devices.  It will be blanked out
         * below (by setting .mmc to 0) for other versions.
         */
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_29_30,
                .no_off_init    = true,
                .power_saving   = true,
	},
	{}	/* Terminator */
};

#ifdef CONFIG_MMC_NOTLE
static struct regulator_consumer_supply notle_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
};
#endif
static struct regulator_consumer_supply notle_vaux_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.1",
	},
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	if (!pdata) {
		dev_err(dev, "%s: NULL platform data\n", __func__);
		return -EINVAL;
	}
	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		 if (ret)
			dev_err(dev, "%s: Error card detect config(%d)\n",
				__func__, ret);
		 else
			pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed omap4_twl6030_hsmmc_set_late_init\n");
		return;
	}
	pdata = dev->platform_data;

	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;
	omap_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++) {
		omap4_twl6030_hsmmc_set_late_init(&c->pdev->dev);
        }

	return 0;
}

// Voltage for eMMC flash
static struct regulator_init_data notle_vaux1 = {
	.constraints = {
		.min_uV			= 2900000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,

	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = notle_vaux_supply,
};


static struct regulator_consumer_supply notle_cam2_supply[] = {
  {
    .supply = "cam2pwr",
  },
  REGULATOR_SUPPLY("av-switch", "omap-abe-twl6040"),
};

// Voltage for sensors (mag, acc, gyro, light sensor, camera)
static struct regulator_init_data notle_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
        .num_consumer_supplies = 2,
        .consumer_supplies = notle_cam2_supply,
};

static struct regulator_init_data notle_vaux3 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 1200000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

#ifdef CONFIG_MMC_NOTLE
/* Voltage for SD card */
static struct regulator_init_data notle_vmmc = {
	.constraints = {
		.min_uV			= 2900000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = notle_vmmc_supply,
};
#else
/* gpio_100 camera power-down */
static struct regulator_init_data notle_vmmc = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};
#endif

/* unused */
static struct regulator_init_data notle_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

/* Voltage for the touchpad */
static struct regulator_init_data notle_vusim = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

/* unused */
static struct regulator_init_data notle_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
                /* Fixed voltage regulators do not have a set_voltage() hook
                 * therefore cannot have the voltage set. */
		.apply_uV		= false,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

/* 1.8V for "lots of important stuff" - clocks, plls, etc. */
static struct regulator_init_data notle_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
                /* Fixed voltage regulators do not have a set_voltage() hook
                 * therefore cannot have the voltage set. */
		.apply_uV		= false,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

/* A/D convertor? */
static struct regulator_init_data notle_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
                /* Fixed voltage regulators do not have a set_voltage() hook
                 * therefore cannot have the voltage set. */
		.apply_uV		= false,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply notle_vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "twl6030_usb"),
};

/* Powers OMAP's USB controller */
/* TODO(rocky): This still doesn't seem to get hooked up properly with the
 * twl6030 driver.  i.e. I still see these in the kernel log:
 * [   77.002502] suspend_set_state: VUSB: No configuration
 */
static struct regulator_init_data notle_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(notle_vusb_supply),
	.consumer_supplies      = notle_vusb_supply,
};

static struct regulator_consumer_supply notle_clk32kg_supply[] = {
	REGULATOR_SUPPLY("clk32kg", NULL),
};

static struct regulator_init_data omap4_notle_clk32kg = {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.always_on              = true,
	},
	.num_consumer_supplies	= ARRAY_SIZE(notle_clk32kg_supply),
	.consumer_supplies	= notle_clk32kg_supply,
};

// TODO(eieio): revisit these when we optimize sleep current
// TODO(atv): Investigate new pad configuration
/* ttyO0 unused */
//static struct omap_device_pad notle_uart1_pads[] __initdata = {
//	{
//        // fails on first with repeat
//		//.name	= "mcspi1_cs2.uart1_cts",
//		.name	= ".uart1_cts",
//		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart1_rts.uart1_rts",
//		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart1_tx.uart1_tx",
//		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart1_rx.uart1_rx",
//		.flags	= OMAP_DEVICE_PAD_REMUX,
//		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
//		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
//	},
//};
//
///* ttyO1 bluetooth */
//static struct omap_device_pad notle_uart2_pads[] __initdata = {
//	{
//		.name	= "uart2_cts.uart2_cts",
//		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart2_rts.uart2_rts",
//		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart2_tx.uart2_tx",
//		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart2_rx.uart2_rx",
//		.flags	= OMAP_DEVICE_PAD_REMUX,
//		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
//		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
//	},
//};

/* ttyO2 console port */
//static struct omap_device_pad notle_uart3_pads[] __initdata = {
//	{
//		.name	= "uart3_cts_rctx.uart3_cts_rctx",
//		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart3_rts_sd.uart3_rts_sd",
//		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart3_tx_irtx.uart3_tx_irtx",
//		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart3_rx_irrx.uart3_rx_irrx",
//		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
//		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
//		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
//	},
//};

/* ttyO3 GPS */
//static struct omap_device_pad notle_uart4_pads[] __initdata = {
//	{
//		.name	= "uart4_tx.uart4_tx",
//		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
//	},
//	{
//		.name	= "uart4_rx.uart4_rx",
//		.flags	= OMAP_DEVICE_PAD_REMUX,
//		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
//		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
//	},
//};

//static struct omap_uart_port_info omap_serial_port_info[] __initdata = {
//        { /* ttyO0 unused */
//                .dma_enabled        = 0,
//                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
//                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
//                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
//                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
//                .wer = 0,
//        },
//        { /* ttyO1 bluetooth */
//                .dma_enabled        = 0,
//                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
//                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
//                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
//                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
//                .wake_peer = bcm_bt_lpm_exit_lpm_locked,
//                .rts_mux_driver_control = 1,
//                .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
//        },
//        { /* ttyO2 console port */
//                .dma_enabled        = 0,
//                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
//                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
//                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
//                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
//                .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
//        },
//        {  /* ttyO3 GPS */
//                .dma_enabled        = 0,
//                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
//                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
//                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
//                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
//                .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
//        },
//};

void __init notle_serial_init(void)
{
	omap_serial_init();
//	omap_serial_board_init(omap_serial_port_info);
	/*omap_serial_init_port_pads(0, notle_uart1_pads,
		ARRAY_SIZE(notle_uart1_pads), &omap_serial_port_info[0]);
	omap_serial_init_port_pads(1, notle_uart2_pads,
		ARRAY_SIZE(notle_uart2_pads), &omap_serial_port_info[1]);
	omap_serial_init_port_pads(3, notle_uart4_pads,
		ARRAY_SIZE(notle_uart4_pads), &omap_serial_port_info[3]);*/
}

/* Initialize FIQ Debugger */
// TODO(atv): FIQ debugger
static int __init board_serial_debug_init(void)
{
	/* 3rd UART is now owned by fiq debugger */
	/*return omap_serial_debug_init(2, false, true,
		notle_uart3_pads, ARRAY_SIZE(notle_uart3_pads));*/
  return 0;
}
device_initcall(board_serial_debug_init);

static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};


// Translate hardware buttons to keys -- we have only one.  Note that
static struct gpio_keys_button notle_button_table[] = {
    [0] = {
                .code   = KEY_CAMERA,           \
                .gpio   = GPIO_CAMERA,          \
                .desc   = "Camera",             \
                .type   = EV_KEY,               \
                .wakeup = 1,                    \
                .debounce_interval = 5,         \
                .active_low = 1,                \
    },
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons  = notle_button_table,
	.nbuttons = ARRAY_SIZE(notle_button_table),
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
	.id   = -1,
};

/* These gpios get set dynamically when we determine the platform */
static struct gps_elton_platform_data_s gps_elton_platform_data  = {
	.gpio_reset = -1,
	.gpio_on_off = -1,
	.gpio_awake = -1,
};

static struct platform_device gps_elton_platform_device = {
	.name	= "gps_elton",
	.id	= -1,
	.dev	= {
		.platform_data	= &gps_elton_platform_data,
	},
};
static struct platform_device notle_spdif_dit_codec = {
	.name           = "spdif-dit",
	.id             = -1,
};
static struct platform_device notle_dmic_codec = {
	.name	= "dmic-codec",
	.id	= -1,
};

static struct omap_abe_twl6040_data notle_abe_audio_data = {
	.has_hs		= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	.has_hf		= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	.has_aux	= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	.has_afm	= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	.jack_detection	= 1,
	.mclk_freq	= 38400000,
	.card_name = "Notle",
	.has_hsmic = 0,
	.has_dmic = 1,
	.has_abe = 1,
	.has_ep = 1,

};

static struct platform_device notle_abe_audio = {
	.name		= "omap-abe-twl6040",
	.id		= -1,
	.dev = {
		.platform_data = &notle_abe_audio_data,
	},
};

static struct platform_device *notle_devices[] __initdata = {
        &leds_gpio,
        &gpio_keys,
        &bcm4330_bluetooth_device,
        &gps_elton_platform_device,
        &notle_dmic_codec,
        &notle_spdif_dit_codec,
        &notle_abe_audio,
};

static struct platform_device notle_pcb_temp_sensor = {
	.name = "notle_pcb_sensor",
};

static char *notle_charger_supplicants_evt1[] = {
	"twl6030_battery",
};

static char *notle_charger_supplicants_evt2[] = {
	"bq27520-0",
};

static struct twl4030_charger_platform_data notle_charger_data = {
	.monitor_interval_seconds = 15,
	.max_charger_current_mA   = 1500,
	.max_charger_voltage_mV   = 4560,
	.max_bat_voltage_mV       = 4200,
	.low_bat_voltage_mV       = 3300,
	/* Fill in supplied_to/num_supplicants based on board revisions */
};

static int notle_batt_table[] = {
        /* adc code for temperature in degree C */
        929, 925, /* -2 ,-1 */
        920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
        875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
        816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
        748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
        671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
        591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
        511, 504, 496 /* 60 - 62 */
};

static struct twl4030_battery_platform_data notle_battery_data = {
	.monitoring_interval_seconds = 15,
	.battery_tmp_tbl             = notle_batt_table,
	.tblsize                     = ARRAY_SIZE(notle_batt_table),
};


static struct twl4030_codec_data twl6040_audio = {
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
};

static struct twl4030_vibra_data twl6040_vibra = {
};

static struct twl4030_audio_data twl6040_codec = {
	.codec		= &twl6040_audio,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

static struct twl6040_platform_data twl6040_data = {
	.codec		= &twl6040_codec,
	.audpwron_gpio	= 127,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

static struct twl4030_madc_platform_data notle_gpadc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data notle_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &notle_vmmc,
	.vpp		= &notle_vpp,
	.vusim		= &notle_vusim,
	.vana		= &notle_vana,
	.vcxio		= &notle_vcxio,
	.vdac		= &notle_vdac,
	.vusb		= &notle_vusb,
	.vaux1		= &notle_vaux1,
	.vaux2		= &notle_vaux2,
	.vaux3		= &notle_vaux3,
	.clk32kg	= &omap4_notle_clk32kg,
	.usb		= &omap4_usbphy_data,

	/* children */
	.charger        = &notle_charger_data,
	.battery        = NULL, // fill this in conditionally for EVT1
	.audio          = &twl6040_codec,
	.madc           = &notle_gpadc_data,
};

#ifdef CONFIG_RMI4_BUS
struct notle_gpio_data_s {
	int gpio_num;
	const char *name;
};

// Need to dynamically set gpio_num based on board type
static struct notle_gpio_data_s notle_touchpad_gpio_data = {
	.name = "touchpad",
};

static int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	int retval = 0;
	struct notle_gpio_data_s *data = gpio_data;

	if (configure) {
		/* Enable the interrupt */
		enable_irq(gpio_to_irq(data->gpio_num));
		printk(KERN_INFO "%s Callback to setup touchpad gpio %d %s\n",
		       __func__, data->gpio_num, data->name);
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		        __func__, data->gpio_num);
	}
	return retval;
}

static struct rmi_f11_sensor_data rmi_device_platform_data_f11 = {
	.axis_align = {
		.swap_axes = false,
		.flip_x = false,
		.flip_y = true,

		.clip_X_low = 0,
		.clip_Y_low = 0,
		.clip_X_high = 0,
		.clip_Y_high = 0,

		.offset_X = 0,
		.offset_Y = 0,
		.rel_report_enabled = 0,
		.delta_x_threshold = 2,
		.delta_y_threshold = 2,
	},

	.virtual_buttons = {
		.buttons = 0,
		.map = NULL,
	},
	/* We are using touchpad type A protocol. */
	.type_a = 1,
};

static struct rmi_device_platform_data synaptics_platformdata = {
	.driver_name = "rmi",
	.sensor_name = "tm2240",

	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.level_triggered = true,
	.gpio_data = &notle_touchpad_gpio_data,
	.gpio_config = synaptics_touchpad_gpio_setup,

	.reset_delay_ms = 100,

	.f11_sensor_data = &rmi_device_platform_data_f11,
	.f11_sensor_count = 1,

	/* function handler pdata */
	.power_management = {
		.nosleep = RMI_F01_NOSLEEP_OFF,
		.wakeup_threshold = 0,
		.doze_holdoff = 0,
		.doze_interval = 0,
	},

#ifdef CONFIG_RMI4_FWLIB
	.firmware_name = "firmware_name",
#endif
#ifdef  CONFIG_PM
	.pm_data = NULL,
	.pre_suspend = NULL,
	.post_suspend = NULL,
	.pre_resume = NULL,
	.post_resume = NULL,
#endif
};

#endif  /* CONFIG_RMI4_BUS */

// cb0/1 LOW/HIGH is tty; LOW/LOW is float (initial default)
static struct usb_mux_platform_data usb_mux_platformdata = {
	// .gpio_cb0 configured in omap_usb_mux_init
	.gpio_cb0_flags = GPIOF_OUT_INIT_LOW,
	.gpio_cb0_label = "gpio_usb_mux_cb0",

	// .gpio_cb1 configured in omap_usb_mux_init
	.gpio_cb1_flags = GPIOF_OUT_INIT_LOW,
	.gpio_cb1_label = "gpio_usb_mux_cb1",
};

static struct platform_device notle_usb_mux = {
	.name = "usb_mux",
	.dev	= {
		.platform_data	= &usb_mux_platformdata,
	},
};

/*
 * Translate temperatures to compensate for thermistor circuit problem (EVT2).
 * Precision is limited but should provide accurate values.
 */
static int notle_translate_temp(int temperature)
{
	if (temperature <= 980)
		/*
		 * This indicates that the thermistor is disconnected on EVT2. Return the same
		 * value as would be measured on EVT3 in case of missing thermistor so that the
		 * gas gauge driver can recognize the disconnected state.
		 */
		temperature = -408;
	else if (temperature <= 986)
		temperature = 0;
	else if (temperature <= 989)
		temperature = 50;
	else if (temperature <= 993)
		temperature = 100;
	else if (temperature <= 998)
		temperature = 150;
	else if (temperature <= 1003)
		temperature = 200;
	else if (temperature <= 1009)
		temperature = 250;
	else if (temperature <= 1016)
		temperature = 300;
	else if (temperature <= 1024)
		temperature = 350;
	else if (temperature <= 1044)
		temperature = 400;
	else if (temperature <= 1055)
		temperature = 450;
	else if (temperature <= 1067)
		temperature = 500;
	else if (temperature <= 1080)
		temperature = 550;
	else if (temperature <= 1095)
		temperature = 600;
	else if (temperature <= 1116)
		temperature = 650;
	else
		temperature = 700;

	return temperature;
}

/* Gas gauge board specific configuration filled in at board init */
static struct bq27x00_platform_data notle_gasgauge_platform_data = {
	.soc_int_irq = -1,
	.bat_low_irq = -1,
};

static struct i2c_board_info __initdata notle_i2c_1_boardinfo[] = {
#ifdef CONFIG_BATTERY_BQ27x00
	{
		I2C_BOARD_INFO("bq27520", 0x55),
		.platform_data = &notle_gasgauge_platform_data,
	},
#endif
};

static struct i2c_board_info __initdata notle_i2c_3_boardinfo[] = {
#if defined(CONFIG_RMI4_BUS)
        {
                I2C_BOARD_INFO("rmi_i2c", 0x20),
                .platform_data = &synaptics_platformdata,
        },
#endif  /* CONFIG_RMI4_BUS */
};

/*
 * i2c-4
 */
/* MPU */
static struct mpu_platform_data mpu9150_notle_data = {
        .int_config     = 0x10,
        .orientation    = { 0, 1, 0,
                            0, 0, 1,
                            1, 0, 0 },
        .level_shifter  = 1,
        .sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
        .sec_slave_id   = COMPASS_ID_AK8975,
        .secondary_i2c_addr = 0x0C,
        .key = {221, 22, 205, 7, 217, 186, 151, 55,
            206, 254, 35, 144, 225, 102, 47, 50},
        .secondary_orientation = { 1, 0, 0,
                                   0, 0, -1,
                                   0, 1, 0 },
};

// gpio_int_no is board specific
#ifdef CONFIG_INPUT_LTR506ALS
static struct ltr506_platform_data notle_ltr506als_data = {
	/* Boolean to allow interrupt to wake device or not */
	.pfd_gpio_int_wake_dev = 0,

	/* Disable als on suspend flag */
	.pfd_disable_als_on_suspend = 1,

	/* ALS enable filtering interrupts
	 * by suppressing interrupts when measured value
	 * falls within some driver calculated threshold.  This
	 * can be manually set by setting a '0' here and writing
	 * to the sysfs threshold file.
	 * e.g.
	 *  echo "1000 1020" > /sys/class/input/input7/als_threshold
	 */
	.pfd_als_filter_interrupts = 0,

	/* ALS resolution / bit width
	 * '000: 20 bit
	 * '001: 19 bit
	 * '010: 18 bit
	 * '011: 17 bit
	 * '100: 16 bit (default)
	 * '101: 12 bit
	 * '110: 8 bit
	 * '111: 4 bit
	 */
	.pfd_als_resolution = 0,

	/* ALS measurement repeat rate
	 * '000:  100ms
	 * '001:  200ms
	 * '010:  500ms (default)
	 * '011: 1000ms
	 * '1xx: 2000ms */
	.pfd_als_meas_rate = 2,

	/* ALS gain.
	 * '00: 1 lux/count (1-64k lux)
	 * '01: 0.5 lux/count (0.5-32k lux)
	 * '10: 0.01 lux/count (0.02-640 lux)
	 * '11: 0.005 lux/count (0.01-320 lux) */
	.pfd_als_gain = 3,

	/* Disable ps on suspend flag */
	.pfd_disable_ps_on_suspend = 0,

	/* PS enable filtering interrupts
	 * by suppressing interrupts when measured value
	 * falls within some driver calculated threshold.  This
	 * can be manually set by setting a '0' here and writing
	 * to the sysfs threshold file.
	 * e.g.
	 *  echo "1000 1020" > /sys/class/input/input7/als_threshold
	 */
	.pfd_ps_filter_interrupts = 0,

	/* PS measurement repeate rate
	 * '000:   12.5ms (ALS auto-disabled)
	 * '001:   50ms
	 * '010:   70ms
	 * '011:  100ms
	 * '100:  200ms
	 * '101:  500ms
	 * '110: 1000ms
	 * '111: 2000ms */
	.pfd_ps_meas_rate = 1,

	/* PS gain. NOTE(CMM) This is must write to '3'
	 * '00:  x8 gain
	 * '01: x16 gain
	 * '10: x32 gain
	 * '11: x64 gain ## */
	.pfd_ps_gain = 3,

	/* LED pulse frequency.
	 * '000: 30kHz
	 * '001: 40kHz
	 * '010: 50kHz
	 * '011: 60kHz
	 * '100: 70kHz
	 * '101: 80kHz
	 * '110: 90kHz
	 * '111: 100kHz */
	.pfd_led_pulse_freq = 3,

	/* LED Duty cycle. NOTE(CMM) This is must write to '1'
	 * '00:  25%
	 * '01:  50% ##
	 * '10:  75%
	 * '11: 100% */
	.pfd_led_duty_cyc = 1,

	/* LED peak current.
	 * '000:   5mA
	 * '001:  10mA
	 * '010:  20mA
	 * '011:  50mA
	 * '1xx: 100mA */
	.pfd_led_peak_curr = 0,

	/* LED Pulse count. Number of LED pulses to be
	 * emitted for a measurement. */
	.pfd_led_pulse_count = 127,
};
#endif

#ifdef CONFIG_INPUT_GLASSHUB
static struct glasshub_platform_data notle_glasshub_data;
#endif  /* CONFIG_INPUT_GLASSHUB */

static struct i2c_board_info __initdata notle_i2c_4_boardinfo[] = {
        {
                I2C_BOARD_INFO("panel-notle-panel", 0x49),
        },
        {
                I2C_BOARD_INFO("mpu9150", 0x68),
                .platform_data = &mpu9150_notle_data,
        },
#ifdef CONFIG_INPUT_LTR506ALS
        {
                I2C_BOARD_INFO("ltr506als", 0x3a),
                .flags = I2C_CLIENT_WAKE,
                .platform_data = &notle_ltr506als_data,
        },
#endif

/* dls: slave address of 0x35 is chosen not to conflict and
 * easy to see on i2c bus. Can be changed by modifying the
 * code in the Glass hub MCU.
 */
#ifdef CONFIG_INPUT_GLASSHUB
        {
                I2C_BOARD_INFO("glasshub", 0x35),
                .platform_data = &notle_glasshub_data,
        },
#endif
};

static void __init notle_pmic_mux_init(void)
{

        omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
                                          OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
                                struct omap_i2c_bus_board_data *pdata)
{
       /* spinlock_id should be -1 for a generic lock request */
       if (spinlock_id < 0)
               pdata->handle = hwspin_lock_request(USE_MUTEX_LOCK);
       else
               pdata->handle = hwspin_lock_request_specific(spinlock_id, USE_MUTEX_LOCK);

       if (pdata->handle != NULL) {
               pdata->hwspin_lock_timeout = hwspin_lock_timeout;
               pdata->hwspin_unlock = hwspin_unlock;
       } else {
               pr_err("I2C hwspinlock request failed for bus %d\n", \
                                                               bus_id);
       }
}

static struct omap_i2c_bus_board_data __initdata notle_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata notle_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata notle_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata notle_i2c_4_bus_pdata;

static void __init notle_i2c_irq_fixup(void)
{
    int i;
    int gpio_mpu, gpio_prox, gpio_touchpad, gpio_blink;
    struct i2c_board_info *pinfo;

    gpio_prox = notle_get_gpio(GPIO_PROX_INT_INDEX);
    gpio_mpu = notle_get_gpio(GPIO_MPU9000_INT_INDEX);
    gpio_touchpad = notle_get_gpio(GPIO_TOUCHPAD_INT_N_INDEX);
    gpio_blink = notle_get_gpio(GPIO_BLINK_INT_INDEX);

    // Fix up the global device data structures

#ifdef CONFIG_RMI4_BUS
    synaptics_platformdata.attn_gpio = gpio_touchpad;
#endif

#ifdef CONFIG_INPUT_LTR506ALS
    notle_ltr506als_data.pfd_gpio_int_no = gpio_prox;
#endif

#ifdef CONFIG_INPUT_GLASSHUB
    notle_glasshub_data.gpio_int_no = gpio_blink;
#endif

#ifdef CONFIG_RMI4_BUS
    notle_touchpad_gpio_data.gpio_num = gpio_touchpad;
#endif

    // Now fixup the irqs set in the various 2c boardinfo structs
    pinfo = notle_i2c_4_boardinfo;
    for (i = 0; i < ARRAY_SIZE(notle_i2c_4_boardinfo); i++) {
        if (!strcmp("mpu9150", pinfo->type)  || !strcmp("ak8975", pinfo->type)) {
            pinfo->irq = gpio_to_irq(gpio_mpu);
        }
        if (!strcmp("ltr506als", pinfo->type)) {
            pinfo->irq = gpio_to_irq(gpio_prox);
        }
        if (!strcmp("glasshub", pinfo->type)) {
            pinfo->irq = gpio_to_irq(gpio_blink);
        }
        pinfo++;
    }
}

/* BQ27520 is on I2C1 with the PMIC; this init needs to happen before that bus is initialized. */
static int __init notle_bq27520_init(void)
{
	int soc_int_gpio, soc_int_irq;
	int bat_low_gpio, bat_low_irq;
	int res;

	soc_int_gpio = notle_get_gpio(GPIO_SOC_INT_INDEX);
	bat_low_gpio = notle_get_gpio(GPIO_BAT_LOW_INDEX);

	res = gpio_request_one(soc_int_gpio, GPIOF_IN, "soc_int_n");
	if (res) {
		pr_err("%s: Failed to get soc_int gpio: %d\n", __func__, res);
		goto error;
	}

	soc_int_irq = gpio_to_irq(soc_int_gpio);

	res = irq_set_irq_wake(soc_int_irq, 1);
	if (res) {
		pr_err("%s: Failed to set irq wake for soc_int: %d\n", __func__, res);
		goto error;
	}

	res = gpio_request_one(bat_low_gpio, GPIOF_IN, "bat_low_n");
	if (res) {
		pr_err("%s: Failed to get bat_low gpio: %d\n", __func__, res);
		goto error;
	}

	bat_low_irq = gpio_to_irq(bat_low_gpio);

	res = irq_set_irq_wake(bat_low_irq, 1);
	if (res) {
		pr_err("%s: Failed to set irq wake for bat_low: %d\n", __func__, res);
		goto error;
	}

	pr_warn("%s: soc_int_gpio=%d soc_int_irq=%d bat_low_gpio=%d bat_low_irq=%d\n",
			__func__, soc_int_gpio, soc_int_irq, bat_low_gpio, bat_low_irq);

	notle_gasgauge_platform_data.soc_int_irq = soc_int_irq;
	notle_gasgauge_platform_data.bat_low_irq = bat_low_irq;

	return 0;

error:
	return res;
}

static int __init notle_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &notle_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &notle_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &notle_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &notle_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &notle_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &notle_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &notle_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &notle_i2c_4_bus_pdata);

	/* setup the charger/battery platform data based on board revision */
	switch (NOTLE_VERSION) {
		case V1_EVT1:
			notle_charger_data.supplied_to = notle_charger_supplicants_evt1;
			notle_charger_data.num_supplicants =
				ARRAY_SIZE(notle_charger_supplicants_evt1);

			/* EVT1 uses the PMIC gas gauge to poorly monitory the state of charge */
			notle_twldata.battery = &notle_battery_data;
			break;

		case V1_EVT2:
			notle_bq27520_init();

			notle_gasgauge_platform_data.translate_temp = notle_translate_temp;

				notle_charger_data.supplied_to = notle_charger_supplicants_evt2;
			notle_charger_data.num_supplicants =
				ARRAY_SIZE(notle_charger_supplicants_evt2);

			/* gas gauge is on i2c1, which is registered in the pmic init */
			i2c_register_board_info(1, notle_i2c_1_boardinfo,
					ARRAY_SIZE(notle_i2c_1_boardinfo));
			break;

		case V1_EVT3:
		case V1_DVT1:
		case V1_5_PROTO:
		default:
			notle_bq27520_init();

			notle_gasgauge_platform_data.translate_temp = NULL;

			notle_charger_data.supplied_to = notle_charger_supplicants_evt2;
			notle_charger_data.num_supplicants =
				ARRAY_SIZE(notle_charger_supplicants_evt2);

			/* gas gauge is on i2c1, which is registered in the pmic init */
			i2c_register_board_info(1, notle_i2c_1_boardinfo,
					ARRAY_SIZE(notle_i2c_1_boardinfo));
			break;

	}

	if (!notle_version_supported()) {
		pr_err("Unrecognized Notle version: %i\n", NOTLE_VERSION);
		return -1;
	}

	omap4_pmic_get_config(&notle_twldata, 0,
		TWL_COMMON_REGULATOR_V1V8 |
		TWL_COMMON_REGULATOR_V2V1);

	omap4_pmic_init("twl6030", &notle_twldata, &twl6040_data, OMAP44XX_IRQ_SYS_2N);
	notle_i2c_irq_fixup();
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, notle_i2c_3_boardinfo,
			ARRAY_SIZE(notle_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, notle_i2c_4_boardinfo,
			ARRAY_SIZE(notle_i2c_4_boardinfo));
	return 0;
}

#ifdef CONFIG_OMAP_MUX
// Board specific MUX settings
// EVT1 Core:
static struct omap_board_mux evt1_board_mux[] __initdata = {
    OMAP4_MUX(GPMC_AD12,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_BT_WAKE
    OMAP4_MUX(GPMC_A19,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_RST_N
    OMAP4_MUX(GPMC_A20,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB0
    OMAP4_MUX(GPMC_A21,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB1
    OMAP4_MUX(GPMC_A24,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_BT_REG_ON
    OMAP4_MUX(GPMC_A25,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_ON_OFF
    OMAP4_MUX(GPMC_NCS2,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_RESET_N
    OMAP4_MUX(GPMC_NCS3,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // LCD_RST_N
    OMAP4_MUX(USBB1_ULPITLL_CLK,    OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // DISP_ENB
    OMAP4_MUX(USBB2_HSIC_STROBE,    OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_WLAN_WAKE
    OMAP4_MUX(MCSPI4_CLK,           OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BT_RST_N
    OMAP4_MUX(MCSPI4_CS0,           OMAP_MUX_MODE3 | OMAP_PIN_INPUT),   // BCM_BT_HOST_WAKE
    OMAP4_MUX(USBB1_ULPITLL_DAT3,   OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // CAM_PWDN
    OMAP4_MUX(USBB1_ULPITLL_DAT5,   OMAP_MUX_MODE7 | OMAP_PULL_ENA),    // BACKLIGHT XXX Remove? NC on EVT1
    OMAP4_MUX(ABE_DMIC_DIN2,        OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // CAMERA, TOP_SW
    OMAP4_MUX(GPMC_AD8,             OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // TOUCHPAD_INT_N
    OMAP4_MUX(USBB1_ULPITLL_DAT2,   OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),                     // PROX_INT
    OMAP4_MUX(USBB1_ULPITLL_DAT7,   OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),           // MPU9000_INT
    OMAP4_MUX(USBB1_ULPITLL_DAT4,   OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),           // MPU9000_INT_TIMER
    OMAP4_MUX(USBB1_ULPITLL_STP,    OMAP_MUX_MODE7 ),                   // FPGA_CDONE
    OMAP4_MUX(USBB1_ULPITLL_DIR,    OMAP_MUX_MODE7 ),                   // FPGA_CRESET_B
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
// EVT1 WakeUp:
static struct omap_board_mux evt1_board_wkup_mux[] __initdata = {
    OMAP4_MUX(SIM_IO,               OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),  // BCM_WLAN_HOST_WAKE
    OMAP4_MUX(FREF_CLK4_REQ,        OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GREEN_LED
    OMAP4_MUX(FREF_CLK4_OUT,        OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // YELLOW_LED
    OMAP4_MUX(SIM_CLK,              OMAP_MUX_MODE7 ),                   // FPGA_CBSEL0
    OMAP4_MUX(SIM_CD,               OMAP_MUX_MODE7 ),                   // FPGA_CBSEL1
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

// EVT2 Core:
static struct omap_board_mux evt2_board_mux[] __initdata = {
    OMAP4_MUX(GPMC_AD12,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_BT_WAKE
    OMAP4_MUX(GPMC_A19,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_RST_N
    OMAP4_MUX(GPMC_A22,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB0
    OMAP4_MUX(GPMC_A21,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB1
    OMAP4_MUX(GPMC_A24,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_BT_REG_ON
    OMAP4_MUX(MCSPI1_CS1,           OMAP_MUX_MODE3 | OMAP_PIN_INPUT),   // GPS_AWAKE
    OMAP4_MUX(MCSPI1_CS2,           OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_ON_OFF
    OMAP4_MUX(MCSPI1_CS3,           OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_RESET_N
    OMAP4_MUX(USBB1_ULPITLL_DAT6,   OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // LCD_RST_N
    OMAP4_MUX(USBB2_HSIC_STROBE,    OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_WLAN_WAKE
    OMAP4_MUX(ABE_MCBSP2_FSX,       OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BT_RST_N
    OMAP4_MUX(SDMMC1_CLK,           OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // CAM_PWDN
    OMAP4_MUX(SDMMC1_DAT1,          OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),                     // ALS_PROX_INT
    OMAP4_MUX(ABE_DMIC_DIN2,        OMAP_MUX_MODE7),                    // OBSOLETE CAMERA, TOP_SW
    OMAP4_MUX(DPM_EMU2,             OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),                     // SOC_INT
    OMAP4_MUX(USBB1_ULPITLL_STP,    OMAP_MUX_MODE3 | OMAP_PIN_INPUT),   // FPGA_CDONE
    OMAP4_MUX(USBB1_ULPITLL_NXT,    OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // FPGA_CRESET_B
    OMAP4_MUX(USBB1_ULPITLL_DAT7,   OMAP_MUX_MODE7 ),                   // FPGA_CBSEL1
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
// EVT2 WakeUp:
static struct omap_board_mux evt2_board_wkup_mux[] __initdata = {
    OMAP4_MUX(SIM_IO,               OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),  // BCM_WLAN_HOST_WAKE
    OMAP4_MUX(FREF_CLK3_REQ,        OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // CAMERA, TOP_SW
    OMAP4_MUX(SIM_CD,               OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // TOUCHPAD_INT_N
    OMAP4_MUX(SIM_CLK,              OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),                     // BLINK_INT
    OMAP4_MUX(SIM_PWRCTRL,          OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),           // MPU9000_INT_TIMER
    OMAP4_MUX(SIM_RESET,            OMAP_MUX_MODE3 | OMAP_PIN_INPUT),   // BCM_BT_HOST_WAKE
    OMAP4_MUX(FREF_CLK3_OUT,        OMAP_MUX_MODE3 | OMAP_PIN_INPUT),   // BAT_LOW
    OMAP4_MUX(SYS_PWRON_RESET_OUT,  OMAP_MUX_MODE7 ),                   // FPGA_CBSEL0
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define emtpy_board_mux	NULL
#endif

static int omap_audio_init(void)
{
	int audio_power_on_gpio = GPIO_AUDIO_POWERON;
	twl6040_codec.audpwron_gpio = audio_power_on_gpio;

	omap_mux_init_signal("sys_nirq2.sys_nirq2", OMAP_PIN_INPUT_PULLUP);
	return 0;
}

static int omap_usb_mux_init(void)
{
	int err;

	usb_mux_platformdata.gpio_cb0 = notle_get_gpio(GPIO_USB_MUX_CB0_INDEX);
	usb_mux_platformdata.gpio_cb1 = GPIO_USB_MUX_CB1;

	err = platform_device_register(&notle_usb_mux);
	if (err)
		pr_err("notle_usb_mux registration failed: %d\n", err);

	return err;
}

static int notle_gps_init(void) {
	int r;
	int gpio_gps_reset, gpio_gps_on_off;

	/* Tell the driver the gpio pins to access the GPS chip */
	gps_elton_platform_data.gpio_reset = notle_get_gpio(GPIO_GPS_RESET_N_INDEX);
	gps_elton_platform_data.gpio_on_off = notle_get_gpio(GPIO_GPS_ON_OFF_INDEX);
	/* An uninialized index returns -1 indicating non existant gpio */
	gps_elton_platform_data.gpio_awake = notle_get_gpio(GPIO_GPS_AWAKE_INDEX);

	/* Configuration of requested GPIO lines */
	gpio_gps_reset = notle_get_gpio(GPIO_GPS_RESET_N_INDEX);
	gpio_gps_on_off = notle_get_gpio(GPIO_GPS_ON_OFF_INDEX);

        r = gpio_request_one(gpio_gps_reset, GPIOF_OUT_INIT_HIGH,
                "gps_reset_n");
        if (r) {
                pr_err("Failed to get gps_reset_n gpio\n");
                goto error;
        }

        r = gpio_export(gpio_gps_reset, false);
        if (r) {
                pr_err("Unable to export gps_reset_n gpio\n");
        }

        r = gpio_sysfs_set_active_low(gpio_gps_reset, 0);
        if (r) {
                pr_err("Unable to set sysfs gps_reset_n active low\n");
        }

        /* Need a rising edge to turn device on. */
        r = gpio_request_one(gpio_gps_on_off, GPIOF_OUT_INIT_LOW,
                "gps_on_off");
        if (r) {
                pr_err("Failed to get gps_on_off gpio\n");
                goto error;
        }

        r = gpio_export(gpio_gps_on_off, false);
        if (r) {
                pr_err("Unable to export gps_on_off gpio\n");
        }

        /* EVT2 added the GPS_AWAKE connection, was NC in EVT1 */
        if ( notle_version_after(V1_EVT1) ) {
            r = gpio_request_one(GPIO_GPS_AWAKE_EVT2, GPIOF_IN, "gps_awake");
            if (r) {
                    pr_err("Failed to get gps_awake gpio\n");
                    goto error;
            }

            r = gpio_export(GPIO_GPS_AWAKE_EVT2, false);
            if (r) {
                    pr_err("Unable to export gps_awake gpio\n");
            }
        }
        return 0;

error:
        return r;
}

/* XXX Turning on GPS currently costs us ~50mA of current draw.
static int __init notle_gps_start(void) {
	gpio_set_value(GPIO_GPS_ON_OFF, 1);
        pr_info("Turning on GPS chip\n");
        return 0;
}
late_initcall(notle_gps_start);
*/

static int __init notle_touchpad_init(void) {
        int r;
        int gpio_touchpad_int;

        pr_info("%s()+\n", __func__);

        /* Configuration of requested GPIO line */
        gpio_touchpad_int = notle_get_gpio(GPIO_TOUCHPAD_INT_N_INDEX);

        r = gpio_request_one(gpio_touchpad_int, GPIOF_IN, "touchpad_int_n");
        if (r) {
                pr_err("Failed to get touchpad_int_n gpio\n");
        }
        /* Allow this interrupt to wake the system */
        r = irq_set_irq_wake(gpio_to_irq(gpio_touchpad_int), 1);
        if (r) {
                pr_err("%s Unable to set irq to wake device\n", __FUNCTION__);
        }
        return r;
}

static int __init notle_imu_init(void) {
        int r;
        int gpio_mpu9000_int_timer, gpio_mpu9000_int;

        pr_info("%s()+\n", __func__);
        gpio_mpu9000_int = notle_get_gpio(GPIO_MPU9000_INT_INDEX);

        /* Configuration of requested GPIO line */

        if (NOTLE_VERSION == V1_EVT1) {
            gpio_mpu9000_int_timer = notle_get_gpio(GPIO_MPU9000_INT_TIMER_INDEX);
            r = gpio_request_one(gpio_mpu9000_int_timer, GPIOF_IN, "mpuirq_timer");
            if (r) {
                    pr_err("Failed to get mpu9000_int_timer gpio\n");
            } else {
                    pr_err("got the mpu9000 timer gpio!!!\n");
            }
        }

        r = gpio_request(gpio_mpu9000_int, "mpuirq");
        if (r) {
                pr_err("Failed to get mpu9000_int gpio\n");
        }
        r = gpio_direction_input(gpio_mpu9000_int);
        if (r) {
                pr_err("Failed to get mpu9000_int gpio\n");
        }
        /* Allow this interrupt to wake the system */
        r = irq_set_irq_wake(gpio_to_irq(gpio_mpu9000_int), 1);
        if (r) {
                pr_err("%s Unable to set irq to wake device\n", __FUNCTION__);
        }
        return r;
}

#ifdef CONFIG_INPUT_GLASSHUB
static int __init notle_glasshub_init(void) {
        int r;

        pr_info("%s()+\n", __func__);

		notle_glasshub_data.irq = gpio_to_irq(notle_glasshub_data.gpio_int_no);

        /* Configuration of requested GPIO line */
        r = gpio_request_one(notle_glasshub_data.gpio_int_no, GPIOF_IN, "glasshub_int");
        if (r) {
                pr_err("Failed to get glasshub gpio\n");
        }

        return r;
}
#endif

#define NOTLE_FB_RAM_SIZE 640*360*4*3
static struct omapfb_platform_data notle_fb_pdata = {
        .mem_desc = {
                .region_cnt = 1,
                .region = {
                        [0] = {
                                .size = NOTLE_FB_RAM_SIZE,
                        },
                },
        },
};

static u32 __get_notle_memsize(void)
{
    // Check the DMM Register to figure out if this is 1 GB or 2 GB device
    // bit 22:20 specs the sys size.
    u32 sys_size = (__raw_readl(OMAP44XX_DMM_VIRT + DMM_LISA_MAP__0) >> 20) & 0x7;
    return 2048 >> (7-sys_size);
}

static void __init notle_init(void)
{
        int package = OMAP_PACKAGE_CBS;
        int err;
        u32 omap_reg;
        printk(KERN_DEBUG "notle_init\n");
        if (omap_rev() == OMAP4430_REV_ES1_0)
                package = OMAP_PACKAGE_CBL;
        notle_version_init();
        omap_create_board_props();
        if (!notle_version_supported()) {
              omap_emif_set_device_details(1, &lpddr2_elpida_2G_S4_x2_info, lpddr2_elpida_2G_S4_timings, ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
                                           &lpddr2_elpida_S4_min_tck, NULL);
              omap_emif_set_device_details(2, &lpddr2_elpida_2G_S4_x2_info, lpddr2_elpida_2G_S4_timings, ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
                                           &lpddr2_elpida_S4_min_tck, NULL);
              pr_err("No mux init for Notle version: %s\n",
                     notle_version_str(NOTLE_VERSION));
        }

        if ( notle_version_before(V1_EVT2) ) {
            omap_emif_set_device_details(1, &lpddr2_elpida_2G_S4_x2_info, lpddr2_elpida_2G_S4_timings, ARRAY_SIZE(lpddr2_elpida_2G_S4_timings), 
                                         &lpddr2_elpida_S4_min_tck, NULL);
            omap_emif_set_device_details(2, &lpddr2_elpida_2G_S4_x2_info, lpddr2_elpida_2G_S4_timings, ARRAY_SIZE(lpddr2_elpida_2G_S4_timings), 
                                         &lpddr2_elpida_S4_min_tck, NULL);

            omap4_mux_init(evt1_board_mux, evt1_board_wkup_mux, package);

            // Additional mux/pad settings

            // The GPIOWK_IO_PWRDNZ bit needs to be set after muxing
            //and before you set it to input
            omap4_ctrl_wk_pad_writel(OMAP4_USIM_PWRDNZ_MASK,
                    OMAP4_CTRL_MODULE_PAD_WKUP_CONTROL_USIMIO);
        } else {

            u32 mem_size_mb = __get_notle_memsize();
            printk("Notle Board Ram Size = %d MB\n", mem_size_mb);

            if (notle_version_after(V1_EVT2) && (mem_size_mb==2048))
            {
                omap_emif_set_device_details(1, &lpddr2_elpida_4G_S4_x2_info, lpddr2_elpida_4G_S4_timings, ARRAY_SIZE(lpddr2_elpida_4G_S4_timings),
                                             &lpddr2_elpida_S4_min_tck, NULL);
                omap_emif_set_device_details(2, &lpddr2_elpida_4G_S4_x2_info, lpddr2_elpida_4G_S4_timings, ARRAY_SIZE(lpddr2_elpida_4G_S4_timings),
                                             &lpddr2_elpida_S4_min_tck, NULL);
            }
            else {
                omap_emif_set_device_details(1, &lpddr2_elpida_4G_S4_info, lpddr2_elpida_4G_S4_timings, ARRAY_SIZE(lpddr2_elpida_4G_S4_timings),
                                             &lpddr2_elpida_S4_min_tck, NULL);
            }

            omap4_mux_init(evt2_board_mux, evt2_board_wkup_mux, package);

            // Additional mux/pad settings

            // The GPIOWK_IO_PWRDNZ bit needs to be set after muxing
            //and before you set it to input
            omap4_ctrl_wk_pad_writel(OMAP4_USIM_PWRDNZ_MASK,
                    OMAP4_CTRL_MODULE_PAD_WKUP_CONTROL_USIMIO);

            // Camera power down on gpio_100 needs the correct voltage of 1.8V
            // wk1 needs correct magic bias settings
            omap_reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);
            omap_reg |= OMAP4_MMC1_PBIASLITE_PWRDNZ_MASK | OMAP4_MMC1_PWRDNZ_MASK;
            omap_reg &= ~OMAP4_MMC1_PBIASLITE_VMODE_MASK;
            omap_reg |= OMAP4_USIM_PBIASLITE_PWRDNZ_MASK;
            omap_reg &= ~OMAP4_USIM_PBIASLITE_VMODE_MASK;
            omap4_ctrl_pad_writel(omap_reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);
        }
        notle_pmic_mux_init();

        printk("Notle board revision: %s(%d)", notle_version_str(NOTLE_VERSION), NOTLE_VERSION);

        err = omap_usb_mux_init();
        if (err) {
                pr_err("USB MUX intitialization failed: %d\n", err);
        }

        err = omap_audio_init();
        if (err) {
                pr_err("Audio initialization failed: %d\n", err);
        }

#ifndef CONFIG_MMC_NOTLE
        // Disable support for sd card:
        mmc[2].mmc = 0;
#endif

        err = notle_imu_init();
        if (err) {
                pr_err("IMU initialization failed: %d\n", err);
        }
        notle_i2c_init();
        omap4_register_ion();


        err = notle_wlan_init();
        if (err) {
                pr_err("Wifi initialization failed: %d\n", err);
        }

        notle_serial_init();
        omap4_twl6030_hsmmc_init(mmc);
        usb_musb_init(&musb_board_data);
        omap_init_dmm_tiler();

        omapfb_set_platform_data(&notle_fb_pdata);

        err = notle_gps_init();
        if (err) {
                pr_err("GPS initialization failed: %d\n", err);
        }

        // Do this after the wlan_init, which inits the regulator shared
        // with the bluetooth device and muxes the bt signals.
        platform_add_devices(notle_devices, ARRAY_SIZE(notle_devices));

        err = platform_device_register(&notle_pcb_temp_sensor);
        if (err) {
            pr_err("notle_pcb_temp_sensor registration failed: %d\n", err);
        }

        err = notle_touchpad_init();
        if (err) {
                pr_err("Touchpad initialization failed: %d\n", err);
        }

#ifdef CONFIG_INPUT_GLASSHUB
        err = notle_glasshub_init();
        if (err) {
                pr_err("Glass hub initialization failed: %d\n", err);
        }
#endif

        if ( notle_version_before(V1_EVT2) ) {
            panel_notle_device.data = &panel_notle_preevt2;
        }
        err = notle_dpi_init();
        if (!err) {
            ((struct panel_notle_data *)panel_notle_device.data)->notle_version = NOTLE_VERSION;
            omap_display_init(&panel_notle_dss_data);
        } else {
            pr_err("DPI initialization failed: %d\n", err);
        }

        omap_enable_smartreflex_on_init();
        omap_pm_enable_off_mode();
        printk(KERN_DEBUG "notle_init done\n");
}

static void __init notle_map_io(void)
{
        omap2_set_globals_443x();
        omap44xx_map_common_io();
}

static struct sgx_omaplfb_config notle_omaplfb_config[] = {
  {
    .tiler2d_buffers = 2,
    .swap_chain_length = 2,
  }
};

static struct sgx_omaplfb_platform_data notle_omaplfb_plat_data = {
  .num_configs = 1,
  .configs = notle_omaplfb_config,
};

static void __init notle_reserve(void)
{
	omap_rproc_reserve_cma(RPROC_CMA_OMAP4);
#ifdef CONFIG_ION_OMAP
        omap_android_display_setup(&panel_notle_dss_data,
                                   NULL,
                                   &notle_omaplfb_plat_data,
                                   &notle_fb_pdata
                                   );
	omap4_ion_init();
#else
        omap_android_display_setup(&panel_notle_dss_data,
                                   NULL,
                                   NULL,
                                   &notle_fb_pdata
                                   );
#endif

	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);

	omap_reserve();
}

MACHINE_START(NOTLE, "OMAP4430")
	/* Maintainer: David Anders - Texas Instruments Inc */
	.atag_offset    = 0x100,
	.reserve	= notle_reserve,
	.map_io		= notle_map_io,
	.init_early	= notle_init_early,
	.init_irq	= gic_init_irq,
	.handle_irq = gic_handle_irq,
	.init_machine	= notle_init,
	.timer		= &omap4_timer,
	.restart = omap_prcm_restart,
MACHINE_END
