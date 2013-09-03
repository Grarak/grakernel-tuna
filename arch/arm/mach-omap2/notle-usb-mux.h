/*
 * arch/arm/mach-omap2/notle-usb-mux.h
 *
 * Notle board USB MUX control platform data.
 *
 * Copyright (c) 2013 Google, Inc.
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

#ifndef __ARCH_ARM_MACH_OMAP2_NOTLE_USB_MUX_H
#define __ARCH_ARM_MACH_OMAP2_NOTLE_USB_MUX_H

/* Defines for PMIC that should be in TI's headers but sadly are not */
/* PMIC Misc registers	& OTG Backup ID1/0x48 TWL6030_MODULE_ID0 */
#define REG_PMC_MISC_MISC2					 0xe5
#define REG_OTG_BACKUP						 0xfa
#define TWL6030_OTG_BACKUP_OTG_REV			(1<<1)
#define TWL6030_OTG_BACKUP_ID_WKUP			(0<<1)

/* USB OG registers ID2/0x49 TWL_MODULE_USB or TWL6030_MODULE_ID1 */
#define REG_USB_ID_CTRL_SET					0x06
#define REG_USB_ID_CTRL_CLR					0x07
#define TWL6030_USB_ID_CTRL_PU_220K			(1<<7)
#define TWL6030_USB_ID_CTRL_PU_100K			(1<<6)
#define TWL6030_USB_ID_CTRL_GND_DRV			(1<<5)
#define TWL6030_USB_ID_CTRL_SRC_16U			(1<<4)
#define TWL6030_USB_ID_CTRL_SRC_5U			(1<<3)
#define TWL6030_USB_ID_CTRL_ACT_COMP		(1<<2)
#define TWL6030_USB_ID_CTRL_MEAS			(1<<0)
#define REG_USB_ID_INT_SRC					0x0f
#define TWL6030_USB_ID_INT_FLOAT			(1<<4)
#define TWL6030_USB_ID_INT_A				(1<<3)
#define TWL6030_USB_ID_INT_B				(1<<2)
#define TWL6030_USB_ID_INT_C				(1<<1)
#define TWL6030_USB_ID_INT_GND				(1<<0)

/* GPADC Trimming registers ID3/0x4A TWL6030_MODULE_ID2 */
#define REG_GPADC_TRIM15					0xdb
#define REG_GPADC_TRIM16					0xdc

#define TWL6030_GPADC_ID_CHANNEL			14

struct usb_mux_platform_data {
	int gpio_cb0;
	int gpio_cb0_flags;
	const char *gpio_cb0_label;

	int gpio_cb1;
	int gpio_cb1_flags;
	const char *gpio_cb1_label;
};

#endif

