/* Board support file for Samsung Tuna Board.
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2010 Texas Instruments
 *
 * Based on mach-omap2/board-omap4panda.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>
#include <linux/reboot.h>
#include <linux/memblock.h>
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/omap_die_governor.h>

#include <mach/hardware.h>
#include <asm/hardware/gic.h>
#include "common-board-devices.h"
#include "common.h"
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/drm.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap-pm.h>
#include <plat/omap_apps_brd_id.h>

#include "omap4-sar-layout.h"
#include "hsmmc.h"
#include "control.h"
#include "pm.h"
#include "mux.h"
#include "board-tuna.h"
#include "omap4_ion.h"

#define DEFAULT_PLAT_PHYS_OFFSET 0x80000000 /* Default RAM start address */

#define TUNA_RAMCONSOLE_START	(DEFAULT_PLAT_PHYS_OFFSET + SZ_512M)
#define TUNA_RAMCONSOLE_SIZE	SZ_2M

#define GPIO_AUD_PWRON		127
#define GPIO_AUD_PWRON_TORO_V1	20

#define REBOOT_FLAG_RECOVERY	0x52564352
#define REBOOT_FLAG_FASTBOOT	0x54534146
#define REBOOT_FLAG_NORMAL	0x4D524F4E

static int tuna_hw_rev;

static struct gpio tuna_hw_rev_gpios[] = {
	{76, GPIOF_IN, "hw_rev0"},
	{75, GPIOF_IN, "hw_rev1"},
	{74, GPIOF_IN, "hw_rev2"},
	{73, GPIOF_IN, "hw_rev3"},
	{170, GPIOF_IN, "hw_rev4"},
};

static void omap4_tuna_init_hw_rev(void)
{
	int ret;
	int i;
	u32 r;

	/* Disable weak driver pulldown on usbb2_hsic_strobe */
	r = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_USBB_HSIC);
	r &= ~OMAP4_USBB2_HSIC_STROBE_WD_MASK;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_USBB_HSIC);

	ret = gpio_request_array(tuna_hw_rev_gpios,
		ARRAY_SIZE(tuna_hw_rev_gpios));

	BUG_ON(ret);

	for (i = 0; i < ARRAY_SIZE(tuna_hw_rev_gpios); i++)
		tuna_hw_rev |= gpio_get_value(tuna_hw_rev_gpios[i].gpio) << i;

	pr_info("Tuna HW revision: %02x\n", tuna_hw_rev);
}

int omap4_tuna_get_revision(void)
{
	return tuna_hw_rev & TUNA_REV_MASK;
}

int omap4_tuna_get_type(void)
{
	return tuna_hw_rev & TUNA_TYPE_MASK;
}

bool omap4_tuna_final_gpios(void)
{
	int type = omap4_tuna_get_type();
	int rev = omap4_tuna_get_revision();

	if (type == TUNA_TYPE_TORO ||
	    (rev != TUNA_REV_PRE_LUNCHBOX && rev != TUNA_REV_LUNCHBOX))
		return true;

	return false;
}

/* wl127x BT, FM, GPS connectivity chip */
static int wl1271_gpios[] = {46, -1, -1};
static struct platform_device wl1271_device = {
	.name	= "kim",
	.id	= -1,
	.dev	= {
		.platform_data	= &wl1271_gpios,
	},
};

static struct resource ramconsole_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start	= TUNA_RAMCONSOLE_START,
		.end	= TUNA_RAMCONSOLE_START + TUNA_RAMCONSOLE_SIZE - 1,
	},
};

static struct platform_device ramconsole_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ramconsole_resources),
	.resource       = ramconsole_resources,
};

static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};

static void __init tuna_bt_init(void)
{
	/* BT_EN - GPIO 104 */
	omap_mux_init_signal("gpmc_ncs6.gpio_103", OMAP_PIN_OUTPUT);
	/*BT_nRST - GPIO 42 */
	omap_mux_init_signal("gpmc_a18.gpio_42", OMAP_PIN_OUTPUT);
	/* BT_WAKE - GPIO 27 */
	omap_mux_init_signal("dpm_emu16.gpio_27", OMAP_PIN_OUTPUT);
	/* BT_HOST_WAKE  - GPIO 177 */
	omap_mux_init_signal("kpd_row5.gpio_177", OMAP_PIN_INPUT);
}

static struct platform_device *tuna_devices[] __initdata = {
	&ramconsole_device,
	&wl1271_device,
	&bcm4330_bluetooth_device,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	.mode			= MUSB_PERIPHERAL,
#else
	.mode			= MUSB_OTG,
#endif
	.power			= 100,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.nonremovable	= true,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.ocr_mask       = MMC_VDD_165_195,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.name		= "omap_wlan",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21,
		.nonremovable	= false,
		//.mmc_data	= &tuna_wifi_data,
	},
	{}	/* Terminator */
};

#ifdef CONFIG_OMAP4_DUTY_CYCLE_GOVERNOR

static struct pcb_section omap4_duty_governor_pcb_sections[] = {
	{
		.pcb_temp_level			= DUTY_GOVERNOR_DEFAULT_TEMP,
		.max_opp			= 1200000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1200000,
			.cooling_rate		= 1008000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 24,
		},
	},
};

static void init_duty_governor(void)
{
	omap4_duty_pcb_section_reg(omap4_duty_governor_pcb_sections,
				   ARRAY_SIZE
				   (omap4_duty_governor_pcb_sections));
}
#else
static void init_duty_governor(void){}
#endif /*CONFIG_OMAP4_DUTY_CYCLE*/

/* Initial set of thresholds for different thermal zones */
static struct omap_thermal_zone thermal_zones[] = {
	OMAP_THERMAL_ZONE("safe", 0, 25000, 65000, 250, 1000, 400),
	OMAP_THERMAL_ZONE("monitor", 0, 60000, 80000, 250, 250,	250),
	OMAP_THERMAL_ZONE("alert", 0, 75000, 90000, 250, 250, 150),
	OMAP_THERMAL_ZONE("critical", 1, 85000,	115000,	250, 250, 50),
};

static struct omap_die_governor_pdata omap_gov_pdata = {
	.zones = thermal_zones,
	.zones_num = ARRAY_SIZE(thermal_zones),
};

static struct regulator_consumer_supply tuna_vaux3_supplies[] = {
	{
		.supply = "vlcd",
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

static struct regulator_init_data tuna_vaux3 = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(tuna_vaux3_supplies),
	.consumer_supplies = tuna_vaux3_supplies,
};

#if 0 
3.4-Disable audio for now.
static struct twl4030_codec_audio_data twl6040_audio = {
	/* Add audio only data */
};

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};
#endif

static struct twl4030_platform_data tuna_twldata = {
	.vaux3		= &tuna_vaux3,
};

static void tuna_audio_init(void)
{
	unsigned int aud_pwron;

	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);

	/* aud_pwron */
	if (omap4_tuna_get_type() == TUNA_TYPE_TORO &&
	    omap4_tuna_get_revision() >= 1)
		aud_pwron = GPIO_AUD_PWRON_TORO_V1;
	else
		aud_pwron = GPIO_AUD_PWRON;
	omap_mux_init_gpio(aud_pwron, OMAP_PIN_OUTPUT);
	//twl6040_codec.audpwron_gpio = aud_pwron;
}

static struct i2c_board_info __initdata tuna_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &tuna_twldata,
	},
};

static int __init tuna_i2c_init(void)
{

	omap4_pmic_get_config(&tuna_twldata, TWL_COMMON_PDATA_USB,
			TWL_COMMON_REGULATOR_VDAC |
			TWL_COMMON_REGULATOR_VAUX2 |
			TWL_COMMON_REGULATOR_VMMC |
			TWL_COMMON_REGULATOR_VPP |
			TWL_COMMON_REGULATOR_VANA |
			TWL_COMMON_REGULATOR_VCXIO |
			TWL_COMMON_REGULATOR_VUSB |
			TWL_COMMON_REGULATOR_CLK32KG);

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("i2c1_scl.i2c1_scl", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("i2c1_sda.i2c1_sda", OMAP_PIN_INPUT_PULLUP);

	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, tuna_i2c1_boardinfo,
			      ARRAY_SIZE(tuna_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	omap_register_i2c_bus(4, 400, NULL, 0);
	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* hwrev */
	OMAP4_MUX(CSI21_DY4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DX4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DY3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DX3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(USBB2_HSIC_STROBE, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_board_mux board_wkup_mux[] __initdata = {
	/* power button */
	OMAP4_MUX(SIM_CD, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#else
#define board_mux	NULL
#define board_wkup_mux	NULL

#endif

static int tuna_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	void __iomem *sar_base;
	unsigned int flag = REBOOT_FLAG_NORMAL;

	sar_base = omap4_get_sar_ram_base();

	if (!sar_base)
		return notifier_from_errno(-ENOMEM);

	if (code == SYS_RESTART) {
		if (_cmd) {
			if (!strcmp(_cmd, "recovery"))
				flag = REBOOT_FLAG_RECOVERY;
			else if (!strcmp(_cmd, "bootloader"))
				flag = REBOOT_FLAG_FASTBOOT;
		}
	}

	/* The Samsung LOKE bootloader will look for the boot flag at a fixed
	 * offset from the end of the 1st SAR bank.
	 */
	writel(flag, sar_base + SAR_BANK2_OFFSET - 0xC);

	return NOTIFY_DONE;
}

static struct notifier_block tuna_reboot_notifier = {
	.notifier_call = tuna_notifier_call,
};

#define HSMMC2_MUX	(OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP)
#define HSMMC1_MUX	OMAP_PIN_INPUT_PULLUP

static void __init tuna_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, board_wkup_mux, package);

	omap4_tuna_init_hw_rev();

	omap_create_board_props();

	register_reboot_notifier(&tuna_reboot_notifier);

	if (omap4_tuna_final_gpios()) {
		/* hsmmc d0-d7 */
		omap_mux_init_signal("sdmmc1_dat0.sdmmc1_dat0", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat1.sdmmc1_dat1", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat2.sdmmc1_dat2", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat3.sdmmc1_dat3", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat4.sdmmc1_dat4", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat5.sdmmc1_dat5", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat6.sdmmc1_dat6", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat7.sdmmc1_dat7", HSMMC1_MUX);
		/* hsmmc cmd */
		omap_mux_init_signal("sdmmc1_cmd.sdmmc1_cmd", HSMMC1_MUX);
		/* hsmmc clk */
		omap_mux_init_signal("sdmmc1_clk.sdmmc1_clk", HSMMC1_MUX);
	} else {
		/* hsmmc d0-d7 */
		omap_mux_init_signal("gpmc_ad0", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad1", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad2", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad3", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad4", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad5", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad6", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad7", HSMMC2_MUX);
		/* hsmmc cmd */
		omap_mux_init_signal("gpmc_nwe", HSMMC2_MUX);
		/* hsmmc clk */
		omap_mux_init_signal("gpmc_noe", HSMMC2_MUX);

		mmc[0].mmc = 2;
	}

	if (omap4_tuna_get_revision() != TUNA_REV_PRE_LUNCHBOX) {
		gpio_request(158, "emmc_en");
		gpio_direction_output(158, 1);
		omap_mux_init_gpio(158, OMAP_PIN_INPUT_PULLUP);
	}

	//tuna_wlan_init();
	tuna_audio_init();
	tuna_i2c_init();
	tuna_bt_init();
	platform_add_devices(tuna_devices, ARRAY_SIZE(tuna_devices));
	omap_serial_init();
	omap4_twl6030_hsmmc_init(mmc);
	usb_musb_init(&musb_board_data);
	omap_init_dmm_tiler();
	omap4_register_ion();
	omap_die_governor_register_pdata(&omap_gov_pdata);
	omap4_tuna_display_init();
	omap4_tuna_input_init();
	//omap4_tuna_nfc_init();
	omap4_tuna_power_init();
	omap4_tuna_sensors_init();
	omap_enable_smartreflex_on_init();
	omap_pm_enable_off_mode();
}

static void __init tuna_reserve(void)
{
	omap_reserve();
	memblock_remove(TUNA_RAMCONSOLE_START, TUNA_RAMCONSOLE_SIZE);
}

static void __init tuna_init_early(void)
{
	omap4430_init_early();
	init_duty_governor();
}

MACHINE_START(TUNA, "Tuna")
	/* Maintainer: Google, Inc */
	.atag_offset	= 0x100,
	.reserve	= tuna_reserve,
	.map_io		= omap4_map_io,
	.init_early	= tuna_init_early,
	.init_irq	= gic_init_irq,
	.handle_irq	= gic_handle_irq,
	.init_machine	= tuna_init,
	.timer		= &omap4_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
