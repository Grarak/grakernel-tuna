/*
 * linux/drivers/power/twl6030_charger.c
 *
 * Based on: OMAP4:TWL6030 charger driver for Linux
 *
 * Copyright (C) 2012 Google, Inc.
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/i2c/bq2415x.h>
#include <linux/wakelock.h>
#include <linux/usb/otg.h>
#include <asm/div64.h>
#include <linux/reboot.h>

#define CONTROLLER_INT_MASK               0x00
#define CONTROLLER_CTRL1                  0x01
#define CONTROLLER_WDG                    0x02
#define CONTROLLER_STAT1                  0x03
#define CHARGERUSB_INT_STATUS             0x04
#define CHARGERUSB_INT_MASK               0x05
#define CHARGERUSB_STATUS_INT1            0x06
#define CHARGERUSB_STATUS_INT2            0x07
#define CHARGERUSB_CTRL1                  0x08
#define CHARGERUSB_CTRL2                  0x09
#define CHARGERUSB_CTRL3                  0x0A
#define CHARGERUSB_STAT1                  0x0B
#define CHARGERUSB_VOREG                  0x0C
#define CHARGERUSB_VICHRG                 0x0D
#define CHARGERUSB_CINLIMIT               0x0E
#define CHARGERUSB_CTRLLIMIT1             0x0F
#define CHARGERUSB_CTRLLIMIT2             0x10
#define ANTICOLLAPSE_CTRL1                0x11
#define ANTICOLLAPSE_CTRL2                0x12
#define LED_PWM_CTRL1                     0x14
#define LED_PWM_CTRL2                     0x15

#define FG_REG_00                         0x00
#define FG_REG_01                         0x01
#define FG_REG_02                         0x02
#define FG_REG_03                         0x03
#define FG_REG_04                         0x04
#define FG_REG_05                         0x05
#define FG_REG_06                         0x06
#define FG_REG_07                         0x07
#define FG_REG_08                         0x08
#define FG_REG_09                         0x09
#define FG_REG_10                         0x0A
#define FG_REG_11                         0x0B

/* CONTROLLER_INT_MASK */
#define MVAC_FAULT                        (1 << 7)
#define MAC_EOC                           (1 << 6)
#define LINCH_GATED                       (1 << 5)
#define MBAT_REMOVED                      (1 << 4)
#define MFAULT_WDG                        (1 << 3)
#define MBAT_TEMP                         (1 << 2)
#define MVBUS_DET                         (1 << 1)
#define MVAC_DET                          (1 << 0)

/* CONTROLLER_CTRL1 */
#define CONTROLLER_CTRL1_EN_LINCH         (1 << 5)
#define CONTROLLER_CTRL1_EN_CHARGER       (1 << 4)
#define CONTROLLER_CTRL1_SEL_CHARGER      (1 << 3)

/* CONTROLLER_STAT1 */
#define CONTROLLER_STAT1_EXTCHRG_STATZ    (1 << 7)
#define CONTROLLER_STAT1_LINCH_GATED      (1 << 6)
#define CONTROLLER_STAT1_CHRG_DET_N       (1 << 5)
#define CONTROLLER_STAT1_FAULT_WDG        (1 << 4)
#define CONTROLLER_STAT1_VAC_DET          (1 << 3)
#define VAC_DET                           (1 << 3)
#define CONTROLLER_STAT1_VBUS_DET         (1 << 2)
#define VBUS_DET                          (1 << 2)
#define CONTROLLER_STAT1_BAT_REMOVED      (1 << 1)
#define CONTROLLER_STAT1_BAT_TEMP_OVRANGE (1 << 0)

/* CHARGERUSB_INT_STATUS */
#define EN_LINCH                          (1 << 4)
#define CURRENT_TERM_INT                  (1 << 3)
#define CHARGERUSB_STAT                   (1 << 2)
#define CHARGERUSB_THMREG                 (1 << 1)
#define CHARGERUSB_FAULT                  (1 << 0)

/* CHARGERUSB_INT_MASK */
#define MASK_MCURRENT_TERM                (1 << 3)
#define MASK_MCHARGERUSB_STAT             (1 << 2)
#define MASK_MCHARGERUSB_THMREG           (1 << 1)
#define MASK_MCHARGERUSB_FAULT            (1 << 0)

/* CHARGERUSB_STATUS_INT1 */
#define CHARGERUSB_STATUS_INT1_TMREG      (1 << 7)
#define CHARGERUSB_STATUS_INT1_NO_BAT     (1 << 6)
#define CHARGERUSB_STATUS_INT1_BST_OCP    (1 << 5)
#define CHARGERUSB_STATUS_INT1_TH_SHUTD   (1 << 4)
#define CHARGERUSB_STATUS_INT1_BAT_OVP    (1 << 3)
#define CHARGERUSB_STATUS_INT1_POOR_SRC   (1 << 2)
#define CHARGERUSB_STATUS_INT1_SLP_MODE   (1 << 1)
#define CHARGERUSB_STATUS_INT1_VBUS_OVP   (1 << 0)

/* CHARGERUSB_STATUS_INT2 */
#define ICCLOOP                           (1 << 3)
#define CURRENT_TERM                      (1 << 2)
#define CHARGE_DONE                       (1 << 1)
#define ANTICOLLAPSE                      (1 << 0)

/* CHARGERUSB_CTRL1 */
#define SUSPEND_BOOT                      (1 << 7)
#define OPA_MODE                          (1 << 6)
#define HZ_MODE                           (1 << 5)
#define TERM                              (1 << 4)

/* CHARGERUSB_CTRL2 */
#define CHARGERUSB_CTRL2_VITERM_50        (0 << 5)
#define CHARGERUSB_CTRL2_VITERM_100       (1 << 5)
#define CHARGERUSB_CTRL2_VITERM_150       (2 << 5)
#define CHARGERUSB_CTRL2_VITERM_400       (7 << 5)

/* CHARGERUSB_CTRL3 */
#define VBUSCHRG_LDO_OVRD                 (1 << 7)
#define CHARGE_ONCE                       (1 << 6)
#define BST_HW_PR_DIS                     (1 << 5)
#define AUTOSUPPLY                        (1 << 3)
#define BUCK_HSILIM                       (1 << 0)

/* CHARGERUSB_VOREG */
#define CHARGERUSB_VOREG_3P52             0x01
#define CHARGERUSB_VOREG_4P0              0x19
#define CHARGERUSB_VOREG_4P2              0x23
#define CHARGERUSB_VOREG_4P76             0x3F

/* CHARGERUSB_VICHRG */
#define CHARGERUSB_VICHRG_300             0x0
#define CHARGERUSB_VICHRG_500             0x4
#define CHARGERUSB_VICHRG_1500            0xE

/* CHARGERUSB_CINLIMIT */
#define CHARGERUSB_CIN_LIMIT_100          0x1
#define CHARGERUSB_CIN_LIMIT_300          0x5
#define CHARGERUSB_CIN_LIMIT_500          0x9
#define CHARGERUSB_CIN_LIMIT_NONE         0xF

/* CHARGERUSB_CTRLLIMIT1 */
#define VOREGL_4P16                       0x21
#define VOREGL_4P56                       0x35

/* CHARGERUSB_CTRLLIMIT2 */
#define CHARGERUSB_CTRLLIMIT2_1500        0x0E
#define LOCK_LIMIT                        (1 << 4)

/* ANTICOLLAPSE_CTRL2 */
#define BUCK_VTH_SHIFT                    5

/* FG_REG_00 */
#define CC_ACTIVE_MODE_SHIFT              6
#define CC_AUTOCLEAR                      (1 << 2)
#define CC_CAL_EN                         (1 << 1)
#define CC_PAUSE                          (1 << 0)

#define REG_TOGGLE1                       0x90
#define FGDITHS                           (1 << 7)
#define FGDITHR                           (1 << 6)
#define FGS                               (1 << 5)
#define FGR                               (1 << 4)

/* TWL6030_GPADC_CTRL */
#define GPADC_CTRL_TEMP1_EN               (1 << 0)    /* input ch 1 */
#define GPADC_CTRL_TEMP2_EN               (1 << 1)    /* input ch 4 */
#define GPADC_CTRL_SCALER_EN              (1 << 2)    /* input ch 2 */
#define GPADC_CTRL_SCALER_DIV4            (1 << 3
#define GPADC_CTRL_SCALER_EN_CH11         (1 << 4)    /* input ch 11 */
#define GPADC_CTRL_TEMP1_EN_MONITOR       (1 << 5)
#define GPADC_CTRL_TEMP2_EN_MONITOR       (1 << 6)
#define GPADC_CTRL_ISOURCE_EN             (1 << 7)

#define GPADC_ISOURCE_22uA                22
#define GPADC_ISOURCE_7uA                 7

/* TWL6030/6032 BATTERY VOLTAGE GPADC CHANNELS */
#define TWL6030_GPADC_VBAT_CHNL           0x07
#define TWL6032_GPADC_VBAT_CHNL           0x12

/* TWL6030_GPADC_CTRL2 */
#define GPADC_CTRL2_CH18_SCALER_EN        BIT(2)

#define REG_MISC1                         0xE4
#define VAC_MEAS                          0x04
#define VBAT_MEAS                         0x02
#define BB_MEAS                           0x01

#define REG_USB_VBUS_CTRL_SET             0x04
#define VBUS_MEAS                         0x01
#define REG_USB_ID_CTRL_SET               0x06
#define ID_MEAS                           0x01

#define BBSPOR_CFG                        0xE6
#define BB_CHG_EN                         (1 << 3)

#define STS_HW_CONDITIONS                 0x21
#define STS_USB_ID                        (1 << 2)    /* Level status of USB ID */

enum led_mode {
	LED_PWM_AUTO = 0x0,
	LED_PWM_ON = 0x1,
	LED_PWM_OFF= 0x2,
};

#if defined(CONFIG_MACH_NOTLE)
extern int notle_version_support_battery_temperature(void);
#endif

/* sign extension needs a little care */
static __inline int sign_extend(int n, int num_bits)
{
	int shift = (int)(sizeof(int) * 8 - num_bits);
	return (n << shift) >> shift;
}

/* Ptr to thermistor table */
static struct wake_lock usb_wake_lock;

#define STATE_BATTERY                     0    /* no wall power, charging disabled */
#define STATE_FAULT                       1    /* charging off due to fault condition */
#define STATE_FULL                        2    /* wall power but battery is charged */
#define STATE_USB                         3    /* 500mA wall power, charging enabled */
#define STATE_AC                          4    /* 1000mA wall power, charging enabled */

static const char *twl6030_state[] = {
	"BATTERY", "FAULT", "FULL", "USB", "AC"
};

#define is_powered(di) (di->state > STATE_FAULT)
#define is_charging(di) (di->state > STATE_FULL)

#define LED_STATE_NONE                   -1
#define LED_STATE_OFF                     0
#define LED_STATE_ON                      1
#define LED_STATE_RAMP                    2
#define LED_STATE_OSCILLATE               3
#define LED_UPDATE_RATE_MS               66
#define LED_RAMPUP_TIME_MS             1280

struct led_state {
	int state;          // current state
	int next;           // state after completing current state

	enum led_mode mode; // cached led mode
	int cur;            // current value
	int target;         // target value
	int base;           // bottom/top of oscillation

	long rate;          // rate to change value

	int led_low;        // configured low led level
	int led_high;       // configured high led level
	int led_step;       // step to increment or decrement per timer tick
};

struct twl6030_charger_device_info {
	struct device *dev;

	int voltage_uV;
	int current_uA;
	int current_avg_uA;
	int temperature_cC;
	int bat_health;
	int state;
	int vbus_online;

	int current_limit_mA;
	int temperature_critical;
	int charge_disabled;

	struct led_state led;
	struct timer_list led_timer;
	struct work_struct led_work;

	unsigned int recharge_capacity;

	unsigned long monitor_interval_jiffies;

	u8 usb_online;

	u8 watchdog_duration;
	unsigned int min_vbus;

	struct twl4030_charger_platform_data *platform_data;

	unsigned int charger_incurrent_mA;
	unsigned int charger_outcurrent_mA;
	unsigned int charger_voltage_mV;
	unsigned int max_bat_voltage_mV;
	unsigned int max_charger_current_mA;
	unsigned long usb_max_power;
	unsigned long usb_event;

	struct power_supply usb;
	struct power_supply *battery;

	struct usb_phy *phy;
	struct notifier_block nb;

	struct work_struct charge_control_work;
	struct work_struct charge_fault_work;
	struct delayed_work monitor_work;

	struct workqueue_struct *wq;
};

/* Intensity curve: 255*((x/255)**2) over [0, 255] */
static const int led_curve[] = {
	0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,
	0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x01,  0x01,  0x01,
	0x01,  0x01,  0x01,  0x02,  0x02,  0x02,  0x02,  0x02,  0x03,  0x03,
	0x03,  0x03,  0x04,  0x04,  0x04,  0x04,  0x05,  0x05,  0x05,  0x05,
	0x06,  0x06,  0x06,  0x07,  0x07,  0x07,  0x08,  0x08,  0x09,  0x09,
	0x09,  0x0a,  0x0a,  0x0b,  0x0b,  0x0b,  0x0c,  0x0c,  0x0d,  0x0d,
	0x0e,  0x0e,  0x0f,  0x0f,  0x10,  0x10,  0x11,  0x11,  0x12,  0x12,
	0x13,  0x13,  0x14,  0x14,  0x15,  0x16,  0x16,  0x17,  0x17,  0x18,
	0x19,  0x19,  0x1a,  0x1b,  0x1b,  0x1c,  0x1d,  0x1d,  0x1e,  0x1f,
	0x1f,  0x20,  0x21,  0x21,  0x22,  0x23,  0x24,  0x24,  0x25,  0x26,
	0x27,  0x28,  0x28,  0x29,  0x2a,  0x2b,  0x2c,  0x2c,  0x2d,  0x2e,
	0x2f,  0x30,  0x31,  0x32,  0x32,  0x33,  0x34,  0x35,  0x36,  0x37,
	0x38,  0x39,  0x3a,  0x3b,  0x3c,  0x3d,  0x3e,  0x3f,  0x40,  0x41,
	0x42,  0x43,  0x44,  0x45,  0x46,  0x47,  0x48,  0x49,  0x4a,  0x4b,
	0x4c,  0x4d,  0x4f,  0x50,  0x51,  0x52,  0x53,  0x54,  0x55,  0x57,
	0x58,  0x59,  0x5a,  0x5b,  0x5d,  0x5e,  0x5f,  0x60,  0x61,  0x63,
	0x64,  0x65,  0x66,  0x68,  0x69,  0x6a,  0x6c,  0x6d,  0x6e,  0x70,
	0x71,  0x72,  0x74,  0x75,  0x76,  0x78,  0x79,  0x7a,  0x7c,  0x7d,
	0x7f,  0x80,  0x81,  0x83,  0x84,  0x86,  0x87,  0x89,  0x8a,  0x8c,
	0x8d,  0x8f,  0x90,  0x92,  0x93,  0x95,  0x96,  0x98,  0x99,  0x9b,
	0x9c,  0x9e,  0xa0,  0xa1,  0xa3,  0xa4,  0xa6,  0xa8,  0xa9,  0xab,
	0xac,  0xae,  0xb0,  0xb1,  0xb3,  0xb5,  0xb6,  0xb8,  0xba,  0xbc,
	0xbd,  0xbf,  0xc1,  0xc3,  0xc4,  0xc6,  0xc8,  0xca,  0xcb,  0xcd,
	0xcf,  0xd1,  0xd3,  0xd4,  0xd6,  0xd8,  0xda,  0xdc,  0xde,  0xe0,
	0xe1,  0xe3,  0xe5,  0xe7,  0xe9,  0xeb,  0xed,  0xef,  0xf1,  0xf3,
	0xf5,  0xf7,  0xf9,  0xfb,  0xfd,  0xff,
};

/* Convert from output curve to linear intensity. Used to convert the
 * initial boot value of the led into the linear state machine space.
 * This inverse table selects the mid-points within ranges of equivalent
 * mappings */
static const int led_curve_inverse[] = {
	0x07,  0x13,  0x19,  0x1d,  0x21,  0x25,  0x29,  0x2c,  0x2e,  0x31,
	0x33,  0x36,  0x38,  0x3a,  0x3c,  0x3e,  0x40,  0x42,  0x44,  0x46,
	0x48,  0x4a,  0x4b,  0x4d,  0x4f,  0x50,  0x52,  0x53,  0x55,  0x56,
	0x58,  0x59,  0x5b,  0x5c,  0x5e,  0x5f,  0x60,  0x62,  0x63,  0x64,
	0x65,  0x67,  0x68,  0x69,  0x6a,  0x6c,  0x6d,  0x6e,  0x6f,  0x70,
	0x71,  0x73,  0x74,  0x75,  0x76,  0x77,  0x78,  0x79,  0x7a,  0x7b,
	0x7c,  0x7d,  0x7e,  0x7f,  0x80,  0x81,  0x82,  0x83,  0x84,  0x85,
	0x86,  0x87,  0x88,  0x89,  0x8a,  0x8b,  0x8c,  0x8d,  0x8d,  0x8e,
	0x8f,  0x90,  0x91,  0x92,  0x93,  0x94,  0x94,  0x95,  0x96,  0x97,
	0x98,  0x99,  0x99,  0x9a,  0x9b,  0x9c,  0x9d,  0x9e,  0x9e,  0x9f,
	0xa0,  0xa1,  0xa2,  0xa2,  0xa3,  0xa4,  0xa5,  0xa5,  0xa6,  0xa7,
	0xa8,  0xa8,  0xa9,  0xaa,  0xab,  0xab,  0xac,  0xad,  0xae,  0xae,
	0xaf,  0xb0,  0xb1,  0xb1,  0xb2,  0xb3,  0xb3,  0xb4,  0xb5,  0xb6,
	0xb6,  0xb7,  0xb8,  0xb8,  0xb9,  0xba,  0xba,  0xbb,  0xbc,  0xbc,
	0xbd,  0xbe,  0xbe,  0xbf,  0xc0,  0xc0,  0xc1,  0xc2,  0xc2,  0xc3,
	0xc4,  0xc4,  0xc5,  0xc6,  0xc6,  0xc7,  0xc8,  0xc8,  0xc9,  0xc9,
	0xca,  0xcb,  0xcb,  0xcc,  0xcd,  0xcd,  0xce,  0xce,  0xcf,  0xd0,
	0xd0,  0xd1,  0xd2,  0xd2,  0xd3,  0xd3,  0xd4,  0xd5,  0xd5,  0xd6,
	0xd6,  0xd7,  0xd8,  0xd8,  0xd9,  0xd9,  0xda,  0xda,  0xdb,  0xdc,
	0xdc,  0xdd,  0xdd,  0xde,  0xde,  0xdf,  0xe0,  0xe0,  0xe1,  0xe1,
	0xe2,  0xe2,  0xe3,  0xe4,  0xe4,  0xe5,  0xe5,  0xe6,  0xe6,  0xe7,
	0xe7,  0xe8,  0xe9,  0xe9,  0xea,  0xea,  0xeb,  0xeb,  0xec,  0xec,
	0xed,  0xed,  0xee,  0xee,  0xef,  0xf0,  0xf0,  0xf1,  0xf1,  0xf2,
	0xf2,  0xf3,  0xf3,  0xf4,  0xf4,  0xf5,  0xf5,  0xf6,  0xf6,  0xf7,
	0xf7,  0xf8,  0xf8,  0xf9,  0xf9,  0xfa,  0xfa,  0xfb,  0xfb,  0xfc,
	0xfc,  0xfd,  0xfd,  0xfe,  0xfe,  0xff,
};

static bool otg_enabled(struct usb_phy *phy) {
	return (phy->last_event == USB_EVENT_ID);
}

static int twl6030_get_pwm_level(void)
{
	u8 pwm = 0;
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &pwm, LED_PWM_CTRL1);
	return led_curve_inverse[pwm];
}

static void twl6030_set_pwm_level(u8 pwm)
{
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, led_curve[pwm], LED_PWM_CTRL1);
}

static int twl6030_get_led_mode(void)
{
	u8 mode = 0;
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &mode, LED_PWM_CTRL2);

	return mode & 0x3;
}

static void twl6030_set_led_mode(enum led_mode mode)
{
	u8 oldval = 0;

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &oldval, LED_PWM_CTRL2);
	oldval = (oldval & 0xfc) | mode;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, oldval, LED_PWM_CTRL2);
}

static void twl6030_set_led(struct twl6030_charger_device_info *di,
		enum led_mode mode, int pwm)
{
	if (pwm >= 0) {
		di->led.cur = pwm;
		twl6030_set_pwm_level(pwm & 0xff);
	}

	di->led.mode = mode;
	twl6030_set_led_mode(mode);
}

static void twl6030_led_timer(unsigned long data)
{
	struct twl6030_charger_device_info *di = (struct twl6030_charger_device_info *) data;
	queue_work(di->wq, &di->led_work);
}

/*
 * Read the hardware state and setup the led state struct to match.
 */
static void twl6030_init_led_state(struct twl6030_charger_device_info *di)
{
	int pwm, mode;

	pwm = twl6030_get_pwm_level();
	mode = twl6030_get_led_mode();

	switch (di->led.mode) {
		case LED_PWM_OFF:
			di->led.state = LED_STATE_OFF;
			di->led.cur = pwm;
			di->led.mode = mode;
			break;

		case LED_PWM_ON:
			di->led.state = LED_STATE_ON;
			di->led.cur = pwm;
			di->led.mode = mode;
			break;

		default:
			// set the state to off, since we don't know if the bootloader left the charger on
			twl6030_set_led(di, LED_PWM_OFF, pwm);
			di->led.state = LED_STATE_OFF;
			break;
	}

	di->led.next = LED_STATE_NONE;
	di->led.rate = 0;
	di->led.target = di->led.cur;
	di->led.base = 0;
	di->led.led_low = 0x00;
	di->led.led_high = 0xff;
	di->led.led_step = (di->led.led_high - di->led.led_low) / (LED_RAMPUP_TIME_MS / LED_UPDATE_RATE_MS);

	setup_timer(&di->led_timer, twl6030_led_timer, (unsigned long) di);
}

static int twl6030_led_add(struct twl6030_charger_device_info *di, int inc)
{
	int value = di->led.cur + inc;

	/* if the increment crosses the target, clip to target */
	if ((value < di->led.target && di->led.cur > di->led.target) ||
			(value > di->led.target && di->led.cur < di->led.target)) {
		value = di->led.target;
	}

	return value;
}

static void twl6030_led_run(struct twl6030_charger_device_info *di)
{
	switch (di->led.state) {
		case LED_STATE_OFF:
			twl6030_set_led(di, LED_PWM_OFF, -1);

			if (di->led.next > LED_STATE_NONE) {
				di->led.state = di->led.next;
				di->led.next = LED_STATE_NONE;
				mod_timer(&di->led_timer, msecs_to_jiffies(di->led.rate) + jiffies);
			}
			break;

		case LED_STATE_ON:
			twl6030_set_led(di, LED_PWM_ON, di->led.cur);

			if (di->led.next > LED_STATE_NONE) {
				di->led.state = di->led.next;
				di->led.next = LED_STATE_NONE;
				mod_timer(&di->led_timer, msecs_to_jiffies(di->led.rate) + jiffies);
			}
			break;

		case LED_STATE_RAMP:
			if (di->led.mode != LED_PWM_ON)
				twl6030_set_led(di, LED_PWM_ON, 0);

			if (di->led.cur < di->led.target) {
				di->led.cur = twl6030_led_add(di, di->led.led_step);
				twl6030_set_pwm_level(di->led.cur);
			} else if (di->led.cur > di->led.target) {
				di->led.cur = twl6030_led_add(di, -di->led.led_step);
				twl6030_set_pwm_level(di->led.cur);
			}

			if (di->led.cur == di->led.target) {
				di->led.state = di->led.next;
				di->led.next = LED_STATE_NONE;
			}

			mod_timer(&di->led_timer, msecs_to_jiffies(di->led.rate) + jiffies);
			break;

		case LED_STATE_OSCILLATE:
			if (di->led.mode != LED_PWM_ON)
				twl6030_set_led(di, LED_PWM_ON, 0);

			if (di->led.cur < di->led.target) {
				di->led.cur = twl6030_led_add(di, di->led.led_step);
				twl6030_set_pwm_level(di->led.cur);
			} else if (di->led.cur > di->led.target) {
				di->led.cur = twl6030_led_add(di, -di->led.led_step);
				twl6030_set_pwm_level(di->led.cur);
			}

			if (di->led.cur == di->led.target) {
				int temp = di->led.target;
				di->led.target = di->led.base;
				di->led.base = temp;
			}

			mod_timer(&di->led_timer, msecs_to_jiffies(di->led.rate) + jiffies);
			break;
	}
}

static void twl6030_led_work(struct work_struct *work)
{
	struct twl6030_charger_device_info *di = container_of(work,
			struct twl6030_charger_device_info, led_work);
	twl6030_led_run(di);
}

static void twl6030_eval_led_state(struct twl6030_charger_device_info *di)
{
	if (is_powered(di)) {
		if (is_charging(di)) {
			if (di->led.state != LED_STATE_OSCILLATE) {
				del_timer_sync(&di->led_timer);

				di->led.state = LED_STATE_OSCILLATE;
				di->led.target = di->led.led_high;
				di->led.base = di->led.led_low;
				di->led.rate = LED_UPDATE_RATE_MS;

				twl6030_led_run(di);
			}
		} else {
			if (di->led.state != LED_STATE_ON &&
					!(di->led.state == LED_STATE_RAMP && di->led.next == LED_STATE_ON
						&& di->led.target == (di->led.led_high >> 2))) {
				del_timer_sync(&di->led_timer);

				di->led.state = LED_STATE_RAMP;
				di->led.target = di->led.led_high >> 2;
				di->led.rate = LED_UPDATE_RATE_MS;
				di->led.next = LED_STATE_ON;

				twl6030_led_run(di);
			}
		}
	} else {
		if (di->led.state != LED_STATE_OFF && !(di->led.state == LED_STATE_RAMP
					&& di->led.next == LED_STATE_OFF && di->led.target == 0x00)) {
			del_timer_sync(&di->led_timer);

			di->led.state = LED_STATE_RAMP;
			di->led.target = 0x00;
			di->led.rate = LED_UPDATE_RATE_MS;
			di->led.next = LED_STATE_OFF;

			twl6030_led_run(di);
		}
	}
}

static struct power_supply *get_battery_power_supply(
		struct twl6030_charger_device_info *di)
{
	if (!di->battery)
		di->battery = power_supply_get_by_name(di->usb.supplied_to[0]);
	
	return di->battery;
}

/* charger configuration */
static void twl6030_config_min_vbus_reg(struct twl6030_charger_device_info *di,
		unsigned int value)
{
	u8 rd_reg = 0;
	int ret;

	if (value > 4760 || value < 4200) {
		dev_dbg(di->dev, "invalid min vbus\n");
		return;
	}

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg,
			ANTICOLLAPSE_CTRL2);
	if (ret)
		goto err;
	rd_reg = rd_reg & 0x1F;
	rd_reg = rd_reg | (((value - 4200)/80) << BUCK_VTH_SHIFT);
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, rd_reg,
			ANTICOLLAPSE_CTRL2);

	if (!ret)
		return;
err:
	pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_iterm_reg(struct twl6030_charger_device_info *di,
		unsigned int term_currentmA)
{
	int ret;

	if ((term_currentmA > 400) || (term_currentmA < 50)) {
		dev_dbg(di->dev, "invalid termination current\n");
		return;
	}

	term_currentmA = ((term_currentmA - 50)/50) << 5;
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, term_currentmA,
			CHARGERUSB_CTRL2);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static int twl6030_get_volimit_config(void)
{
	u8 reg;
	int ret;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &reg, CHARGERUSB_CTRLLIMIT1);
	if (ret) {
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
		return -1;
	}

	return (((int) reg) * 20) + 3500;
}

static int twl6030_get_vilimit_config(void)
{
	u8 reg;
	int ret;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &reg, CHARGERUSB_CTRLLIMIT2);
	if (ret) {
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
		return -1;
	}

	reg &= 0xf; // mask only the value field

	switch (reg) {
		case 0x0 ... 0xe: return (reg + 1) * 100;
		default: return 1500;
	}
}

static int twl6030_is_volimit_locked(void)
{
	u8 reg;
	int ret;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &reg, CHARGERUSB_CTRLLIMIT2);
	if (ret) {
		pr_err("%s: Error access to TLW6030 (%d)\n", __func__, ret);
		return -1;
	}

	return !!(reg & LOCK_LIMIT);
}

static void twl6030_config_voreg_reg(struct twl6030_charger_device_info *di,
		unsigned int voltagemV)
{
	int ret;

	if ((voltagemV < 3500) || (voltagemV > 4760)) {
		dev_dbg(di->dev, "invalid charger_voltagemV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
			CHARGERUSB_VOREG);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_vichrg_reg(struct twl6030_charger_device_info *di,
		unsigned int currentmA)
{
	int ret;

	if ((currentmA >= 300) && (currentmA <= 450))
		currentmA = (currentmA - 300) / 50;
	else if ((currentmA >= 500) && (currentmA <= 1500))
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid charger_currentmA\n");
		return;
	}

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
			CHARGERUSB_VICHRG);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_cinlimit_reg(struct twl6030_charger_device_info *di,
		unsigned int currentmA)
{
	int ret;

	if ((currentmA >= 50) && (currentmA <= 750)) {
		currentmA = (currentmA - 50) / 50;
	} else if (currentmA < 50) {
		dev_dbg(di->dev, "invalid input current limit\n");
		return;
	} else {
		/* This is no current limit */
		currentmA = 0x0F;
	}

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
			CHARGERUSB_CINLIMIT);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_limit1_reg(struct twl6030_charger_device_info *di,
		unsigned int voltagemV)
{
	int ret;

	if ((voltagemV < 3500) || (voltagemV > 4760)) {
		dev_dbg(di->dev, "invalid max_charger_voltage_mV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
			CHARGERUSB_CTRLLIMIT1);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_limit2_reg(struct twl6030_charger_device_info *di,
		unsigned int currentmA)
{
	int ret;

	if ((currentmA >= 300) && (currentmA <= 450))
		currentmA = (currentmA - 300) / 50;
	else if ((currentmA >= 500) && (currentmA <= 1500))
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid max_charger_current_mA\n");
		return;
	}

	currentmA |= LOCK_LIMIT;
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
			CHARGERUSB_CTRLLIMIT2);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static int twl6030_set_watchdog(struct twl6030_charger_device_info *di, int val)
{
	di->watchdog_duration = val;
	return twl_i2c_write_u8(TWL6030_MODULE_CHARGER, val, CONTROLLER_WDG);
}


static const int vichrg[] = {
	300, 350, 400, 450, 500, 600, 700, 800,
	900, 1000, 1100, 1200, 1300, 1400, 1500, 300
};

/*
 * Return channel value
 * Or < 0 on failure.
 */
static int twl6030_get_gpadc_conversion(struct twl6030_charger_device_info *di,
		int channel_no)
{
	struct twl6030_gpadc_request req;
	int temp = 0;
	int ret;

	req.channels = (1 << channel_no);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);
	if (ret < 0)
		return ret;

	if (req.rbuf[channel_no] > 0)
		temp = req.rbuf[channel_no];

	return temp;
}

static int is_battery_present(struct twl6030_charger_device_info *di)
{
	/* TODO */
	return 1;
}

/* TODO: remove this hack once notle-usb-mux.c takes over the MUX config completely */
extern void usb_mux_force(int use_usb);

static void twl6030_stop_usb_charger(struct twl6030_charger_device_info *di)
{
	int ret;

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
	dev_warn(di->dev, "battery: CHARGER OFF\n");

	usb_mux_force(false);
}

static void twl6030_start_usb_charger(struct twl6030_charger_device_info *di, int mA)
{
	int ret;

	if (!is_battery_present(di)) {
		dev_err(di->dev, "BATTERY NOT DETECTED!\n");
		return;
	}

	if (di->charge_disabled)
		return;

	if (mA < 50) {
		twl6030_stop_usb_charger(di);
		return;
	}

	usb_mux_force(true);

	twl6030_config_vichrg_reg(di, di->charger_outcurrent_mA);
	twl6030_config_cinlimit_reg(di, mA);
	twl6030_config_voreg_reg(di, di->charger_voltage_mV);
	twl6030_config_iterm_reg(di, di->platform_data->termination_current_mA);

	if (mA >= 50) {
		twl6030_set_watchdog(di, di->watchdog_duration);
		/* disable current termination, suspend mode, boost mode, etc */
		twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CHARGERUSB_CTRL1);
		ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, CONTROLLER_CTRL1_EN_CHARGER, CONTROLLER_CTRL1);
		if (ret)
			goto err;
	}

	dev_warn(di->dev, "battery: CHARGER ON\n");
	return;

err:
	pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

/*
 * Interrupt service routine
 *
 * Attends to TWL 6030 power module interruptions events, specifically
 * USB_PRES (USB charger presence) CHG_PRES (AC charger presence) events
 *
 */
static irqreturn_t twl6030_charger_ctrl_interrupt(int irq, void *_di)
{
	struct twl6030_charger_device_info *di = _di;
	queue_work(di->wq, &di->charge_control_work);
	dev_warn(di->dev, "battery: CHARGE CTRL IRQ\n");
	return IRQ_HANDLED;
}

static irqreturn_t twl6030_charger_fault_interrupt(int irq, void *_di)
{
	struct twl6030_charger_device_info *di = _di;
	int ret;

	u8 usb_charge_sts, usb_charge_sts1, usb_charge_sts2;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts,
						CHARGERUSB_INT_STATUS);
	if (ret)
		goto err;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts1,
						CHARGERUSB_STATUS_INT1);
	if (ret)
		goto err;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts2,
						CHARGERUSB_STATUS_INT2);
	if (ret)
		goto err;

	dev_warn(di->dev, "battery: CHARGE FAULT IRQ: STS %02x INT1 %02x INT2 %02x\n",
			usb_charge_sts, usb_charge_sts1, usb_charge_sts2);
err:
	queue_work(di->wq, &di->charge_fault_work);
	return IRQ_HANDLED;
}

static enum power_supply_property twl6030_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

#define to_twl6030_usb_device_info(x) container_of((x), \
		struct twl6030_charger_device_info, usb);

static int twl6030_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct twl6030_charger_device_info *di = to_twl6030_usb_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->vbus_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = twl6030_get_gpadc_conversion(di, 10) * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int twl6030_usb_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct twl6030_charger_device_info *di =
		container_of(nb, struct twl6030_charger_device_info, nb);

	di->usb_event = event;
	switch (event) {
		case USB_EVENT_VBUS:
			di->usb_online = *((unsigned int *) data);
			break;
		case USB_EVENT_ENUMERATED:
			di->usb_max_power = *((unsigned int *) data);
			break;
		case USB_EVENT_CHARGER:
		case USB_EVENT_NONE:
		case USB_EVENT_ID:
			break;
		default:
			return NOTIFY_OK;
	}

	if (di->usb_event != event) {
		di->usb_event = event;
		queue_work(di->wq, &di->charge_control_work);
	}

	return NOTIFY_OK;
}

static void twl6030_determine_charge_state(struct twl6030_charger_device_info *di)
{
	u8 stat1;
	int newstate = STATE_BATTERY;

	/* TODO: i2c error -> fault? */
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &stat1, CONTROLLER_STAT1);

	/* printk("battery: determine_charge_state() stat1=%02x int1=%02x\n", stat1, int1); */

	if ((stat1 & VBUS_DET) && !otg_enabled(di->phy)) {
		/* dedicated charger detected by PHY? */
		if (di->usb_event == USB_EVENT_CHARGER)
			newstate = STATE_AC;
		else
			newstate = STATE_USB;

		if (!di->vbus_online) {
			di->vbus_online = 1;
			wake_lock(&usb_wake_lock);
			power_supply_changed(&di->usb);
		}
	} else {
		/* ensure we don't have a stale USB_EVENT_CHARGER should detect bounce */
		di->usb_event = USB_EVENT_NONE;

		if (di->vbus_online) {
			di->vbus_online = 0;
			/* give USB and userspace some time to react before suspending */
			wake_lock_timeout(&usb_wake_lock, HZ / 2);
			power_supply_changed(&di->usb);
		}
	}

	if (di->state == newstate)
		return;

	switch (newstate) {
		case STATE_FAULT:
		case STATE_BATTERY:
			if (is_charging(di))
				twl6030_stop_usb_charger(di);
			break;
		case STATE_USB:
		case STATE_AC:
			/* moving out of STATE_FULL should only happen on unplug
			 * or if we actually run down the battery capacity
			 */
			if (di->state == STATE_FULL) {
				newstate = STATE_FULL;
				break;
			}

			/* TODO: high current? */
			if (!is_charging(di))
				twl6030_start_usb_charger(di, di->current_limit_mA);
			break;
	}

	if (di->state != newstate) {
		dev_warn(di->dev, "battery: state %s -> %s\n",
				twl6030_state[di->state], twl6030_state[newstate]);
		di->state = newstate;
		twl6030_eval_led_state(di);
	}
}

static void twl6030_charge_control_work(struct work_struct *work)
{
	struct twl6030_charger_device_info *di =
		container_of(work, struct twl6030_charger_device_info, charge_control_work);

	twl6030_determine_charge_state(di);
}

static void twl6030_charge_fault_work(struct work_struct *work)
{
	struct twl6030_charger_device_info *di =
		container_of(work, struct twl6030_charger_device_info, charge_fault_work);

	u8 usb_charge_sts1;

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts1,
						CHARGERUSB_STATUS_INT1);

	if (usb_charge_sts1 & (1<<2))
		twl6030_stop_usb_charger(di);
	else if (!(usb_charge_sts1 & (1<<2)) && is_charging(di))
		twl6030_start_usb_charger(di, di->current_limit_mA);

	msleep(10);

	twl6030_determine_charge_state(di);
}

static void twl6030_monitor_work(struct work_struct *work)
{
	struct twl6030_charger_device_info *di = container_of(work,
			struct twl6030_charger_device_info, monitor_work.work);

	struct power_supply *battery;
	union power_supply_propval val;
	struct timespec ts;

	int capacity;
	int raw_capacity;
	int voltage_uV;
	int current_uA;
	int temperature_cC;
	int current_limit_mA;
	int qpassed_mAh;
	int charging;
	int ret;
	int status;

	/* pet the charger watchdog */
	if (is_charging(di))
		twl6030_set_watchdog(di, di->watchdog_duration);

	queue_delayed_work(di->wq, &di->monitor_work, di->monitor_interval_jiffies);

	battery = get_battery_power_supply(di);
	if (!battery) {
		dev_err(di->dev, "Failed to get battery power supply supplicant for charger\n");
		goto error;
	}

	if (!battery->get_property) {
		dev_err(di->dev, "bettery power supply has null get_property method\n");
		goto error;
	}

	ret = battery->get_property(battery, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (ret) {
		dev_err(di->dev, "Failed to read battery capacity: %d\n", ret);
		goto error;
	}
	capacity = val.intval;

	ret = battery->get_property(battery, POWER_SUPPLY_PROP_CAPACITY_LEVEL, &val);
	if (ret) {
		dev_err(di->dev, "Failed to read battery raw capacity: %d\n", ret);
		goto error;
	}
	raw_capacity = val.intval;

	ret = battery->get_property(battery, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret) {
		dev_err(di->dev, "Failed to read battery voltage: %d\n", ret);
		voltage_uV = 0;
	} else {
		voltage_uV = val.intval;
	}

	ret = battery->get_property(battery, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (ret) {
		dev_err(di->dev, "Failed to read battery current: %d\n", ret);
		current_uA = 0;
	} else {
		current_uA = val.intval;
	}

	ret = battery->get_property(battery, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret) {
		dev_warn(di->dev, "Failed to read battery temperature: %d\n", ret);
		temperature_cC = 0;
	} else {
		temperature_cC = val.intval;
	}

	ret = battery->get_property(battery, POWER_SUPPLY_PROP_STATUS, &val);
	if (ret) {
		dev_err(di->dev, "Failed to read battery status: %d\n", ret);
		goto error;
	}
	status = val.intval;

	ret = battery->get_property(battery,
		POWER_SUPPLY_PROP_CHARGE_COUNTER, &val);
	if (ret) {
		dev_warn(di->dev, "Failed to read charge counter: %d\n", ret);
		qpassed_mAh = 0;
	} else {
		qpassed_mAh = val.intval;
	}

#if defined(CONFIG_MACH_NOTLE)
        if (!notle_version_support_battery_temperature()) {
                // Use a default temp of 275 for 500mA charging on EVT1
		temperature_cC = 275;
        }
#endif

	/* store values in device info */
	di->voltage_uV = voltage_uV;
	di->current_uA = current_uA;
	di->temperature_cC = temperature_cC;

	/* manage temperature constraints */
	current_limit_mA = di->current_limit_mA;
	charging = is_charging(di);

	if (temperature_cC < 100 || temperature_cC > 450 || di->charge_disabled) {
		di->state = STATE_FAULT;
		di->current_limit_mA = 0;
#if 0
	} else if (temperature_cC < 150) {
		di->current_limit_mA = 169;
	} else if (temperature_cC < 250) {
		di->current_limit_mA = 282;
#endif
	} else {
		di->current_limit_mA = 500;
	}

	if (charging && current_limit_mA != di->current_limit_mA) {
		// update current setting, 0 will stop charging
		twl6030_start_usb_charger(di, di->current_limit_mA);
		twl6030_eval_led_state(di);
	}

	/* with charging state updated, handle hard poweroff points */
	if (temperature_cC < -100 || temperature_cC > 600) {
		dev_err(di->dev, "Battery temperature critical! temp=%d\n", temperature_cC);
		di->temperature_critical++;

		// shut down the device if we get two successive critical readings
		// this give the device time to log the critical temperature message
		if (di->temperature_critical >= 2) {
			dev_err(di->dev, "Powering down!\n");
			kernel_power_off();
		}
	} else {
		// reset counter if we get a non-critical reading
		di->temperature_critical = 0;
	}

	/*
	 * We will charge the device to 100% and the gas gauge reports the battery is full
	 * As a safety, if the gas gauge did not report that, charging will stop when
	 * we dip below 20mA.
	 */
	if (is_charging(di) && capacity == 100) {

		if (status == POWER_SUPPLY_STATUS_FULL) {
			dev_warn(di->dev,
				"bq27x00 reports full capacity, charging stop\n");
			di->state = STATE_FULL;
			twl6030_stop_usb_charger(di);
			twl6030_eval_led_state(di);
		} else if (current_uA < 20000) {
			dev_warn(di->dev,
				"bq27x00 did not report full capacity, charging stop\n");
			di->state = STATE_FULL;
			twl6030_stop_usb_charger(di);
			twl6030_eval_led_state(di);
		}

	}

	if (di->state == STATE_FULL && capacity <= di->recharge_capacity) {
		dev_warn(di->dev, "battery: drained from full to %d%%, charging again\n", capacity);
		di->state = STATE_USB;
		twl6030_start_usb_charger(di, di->current_limit_mA);
		twl6030_eval_led_state(di);
	}

	getnstimeofday(&ts);
	dev_warn(di->dev,
		"capacity=%d%% raw_capacity=%d%% "
		"voltage_uV=%d uV current_uA=%d uA "
		"temperature_cC=%d current_limit_mA=%d "
		"qpassed=%d mAh %s%s [%016lu]\n",
		capacity, raw_capacity,
		voltage_uV, current_uA,
		temperature_cC, di->current_limit_mA,
		qpassed_mAh, twl6030_state[di->state],
		is_charging(di) ? " CHG" : "", (unsigned long) ts.tv_sec);
error:
	twl6030_determine_charge_state(di);
}

static ssize_t set_led_high(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0)
			|| (val > 0xff))
		return -EINVAL;
	di->led.led_high = val;

	return status;
}

static ssize_t show_led_high(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", di->led.led_high);
}

static ssize_t set_led_low(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0)
			|| (val > 0xff))
		return -EINVAL;
	di->led.led_low = val;

	return status;
}

static ssize_t show_led_low(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", di->led.led_low);
}

static ssize_t show_vbus_voltage(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	val = twl6030_get_gpadc_conversion(di, 10);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_id_level(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	int val;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	val = twl6030_get_gpadc_conversion(di, 14);

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_regulation_voltage(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 3500)
			|| (val > di->max_bat_voltage_mV))
		return -EINVAL;
	di->charger_voltage_mV = val;
	twl6030_config_voreg_reg(di, val);

	return status;
}

static ssize_t show_regulation_voltage(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	val = di->max_bat_voltage_mV;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_termination_current(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 400))
		return -EINVAL;
	di->platform_data->termination_current_mA = val;
	twl6030_config_iterm_reg(di, val);

	return status;
}

static ssize_t show_termination_current(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	val = di->platform_data->termination_current_mA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_cin_limit(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 1500))
		return -EINVAL;
	di->charger_incurrent_mA = val;
	twl6030_config_cinlimit_reg(di, val);

	return status;
}

static ssize_t show_cin_limit(struct device *dev, struct device_attribute *attr,
								  char *buf)
{
	unsigned int val;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	val = di->charger_incurrent_mA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_charge_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 300)
			|| (val > di->max_charger_current_mA))
		return -EINVAL;
	di->charger_outcurrent_mA = val;
	twl6030_config_vichrg_reg(di, val);

	return status;
}

static ssize_t show_charge_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	val = di->charger_outcurrent_mA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_min_vbus(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 4200) || (val > 4760))
		return -EINVAL;
	di->min_vbus = val;
	twl6030_config_min_vbus_reg(di, val);

	return status;
}

static ssize_t show_min_vbus(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	unsigned int val;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	val = di->min_vbus;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_recharge_capacity(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 100))
		return -EINVAL;
	di->recharge_capacity = val;

	return status;
}

static ssize_t show_recharge_capacity(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	unsigned int val;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	val = di->recharge_capacity;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_charge_disabled(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int charge_disabled;
	int status = count;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	charge_disabled = di->charge_disabled;

	if (strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;

	di->charge_disabled = !!val;

	if (di->charge_disabled != charge_disabled) {
		if (di->charge_disabled) {
			di->state = STATE_FAULT;
			twl6030_stop_usb_charger(di);
			twl6030_eval_led_state(di);
		} else {
			queue_delayed_work(di->wq, &di->monitor_work, 0);
		}
	}

	return status;
}

static ssize_t show_charge_disabled(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	unsigned int val;
	struct twl6030_charger_device_info *di = dev_get_drvdata(dev);

	val = di->charge_disabled;
	return sprintf(buf, "%u\n", !!val);
}

static DEVICE_ATTR(vbus_voltage, S_IRUGO, show_vbus_voltage, NULL);
static DEVICE_ATTR(id_level, S_IRUGO, show_id_level, NULL);
static DEVICE_ATTR(regulation_voltage, S_IWUSR | S_IRUGO,
		show_regulation_voltage, set_regulation_voltage);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
		show_termination_current, set_termination_current);
static DEVICE_ATTR(cin_limit, S_IWUSR | S_IRUGO, show_cin_limit,
		set_cin_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO, show_charge_current,
		set_charge_current);
static DEVICE_ATTR(min_vbus, S_IWUSR | S_IRUGO, show_min_vbus, set_min_vbus);
static DEVICE_ATTR(recharge_capacity, S_IWUSR | S_IRUGO, show_recharge_capacity,
		set_recharge_capacity);
static DEVICE_ATTR(led_low, S_IWUSR | S_IRUGO, show_led_low, set_led_low);
static DEVICE_ATTR(led_high, S_IWUSR | S_IRUGO, show_led_high, set_led_high);
static DEVICE_ATTR(charge_disabled, S_IWUSR | S_IRUGO, show_charge_disabled,
		set_charge_disabled);

static struct attribute *twl6030_charger_attributes[] = {
	&dev_attr_vbus_voltage.attr,
	&dev_attr_id_level.attr,
	&dev_attr_regulation_voltage.attr,
	&dev_attr_termination_current.attr,
	&dev_attr_cin_limit.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_min_vbus.attr,
	&dev_attr_recharge_capacity.attr,
	&dev_attr_led_low.attr,
	&dev_attr_led_high.attr,
	&dev_attr_charge_disabled.attr,
	NULL,
};

static const struct attribute_group twl6030_charger_attr_group = {
	.attrs = twl6030_charger_attributes,
};

static int __devinit twl6030_charger_probe(struct platform_device *pdev)
{
	struct twl4030_charger_platform_data *pdata = pdev->dev.platform_data;
	struct twl6030_charger_device_info *di;
	int irq = -1;
	int ret;

	dev_warn(&pdev->dev, "enter\n");

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	if (!pdata->supplied_to || pdata->num_supplicants < 1) {
		dev_err(&pdev->dev, "a supplicant must be sepcified; "
				"supplied_to=%p num=%zu\n", pdata->supplied_to,
				pdata->num_supplicants);
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->platform_data = kmemdup(pdata, sizeof(*pdata), GFP_KERNEL);
	if (!di->platform_data) {
		kfree(di);
		return -ENOMEM;
	}

	di->dev = &pdev->dev;
	di->state = STATE_BATTERY;
	di->recharge_capacity = 94;
	di->monitor_interval_jiffies =
		msecs_to_jiffies(pdata->monitor_interval_seconds * 1000);

	di->usb.name = "twl6030_usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = twl6030_usb_props;
	di->usb.num_properties = ARRAY_SIZE(twl6030_usb_props);
	di->usb.get_property = twl6030_usb_get_property;
	di->usb.supplied_to = pdata->supplied_to;
	di->usb.num_supplicants = pdata->num_supplicants;

	if (pdata->num_supplicants > 1) {
		dev_warn(&pdev->dev, "more than one supplicant specified (%zu), only the "
				"first will be used\n", pdata->num_supplicants);
	}

	/*
	 * try to cache the battery power supply now. if it hasn't registered
	 * yet we will cache it next opportunity
	 */
	get_battery_power_supply(di);

	platform_set_drvdata(pdev, di);

	wake_lock_init(&usb_wake_lock, WAKE_LOCK_SUSPEND, "usb_wake_lock");

	di->wq = create_freezable_workqueue(dev_name(&pdev->dev));

	ret = power_supply_register(&pdev->dev, &di->usb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register usb power supply\n");
		goto usb_failed;
	}

	INIT_WORK(&di->led_work, twl6030_led_work);
	INIT_WORK(&di->charge_control_work, twl6030_charge_control_work);
	INIT_WORK(&di->charge_fault_work, twl6030_charge_fault_work);
	INIT_DELAYED_WORK_DEFERRABLE(&di->monitor_work, twl6030_monitor_work);

	/* initialize for USB charging */

	/* see if the bootloader already set and locked the limits */
	if (twl6030_is_volimit_locked() == 1) {
		/* read the vo and vi reg limits set in the bootloader */
		di->max_bat_voltage_mV = twl6030_get_volimit_config();
		di->max_charger_current_mA = twl6030_get_vilimit_config();

		dev_warn(&pdev->dev, "Using limits set by bootloader: %d mV %d mA\n",
				di->max_bat_voltage_mV, di->max_charger_current_mA);
	} else {
		/* use the conservative platform data defaults for safety */
		di->max_bat_voltage_mV = pdata->max_bat_voltage_mV;
		di->max_charger_current_mA = pdata->max_charger_current_mA;

		twl6030_config_limit1_reg(di, pdata->max_charger_voltage_mV);
		twl6030_config_limit2_reg(di, pdata->max_charger_current_mA);

		dev_warn(&pdev->dev, "Using default limits: %d mV %d mA\n",
				di->max_bat_voltage_mV, di->max_charger_current_mA);
	}

	di->charger_voltage_mV = di->max_bat_voltage_mV;
	di->charger_outcurrent_mA = di->max_charger_current_mA;

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
			CONTROLLER_INT_MASK);
	if (ret)
		goto init_failed;

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
			MASK_MCHARGERUSB_THMREG | MASK_MCURRENT_TERM,
			CHARGERUSB_INT_MASK);
	if (ret)
		goto init_failed;


	twl6030_set_watchdog(di, 32);

	di->nb.notifier_call = twl6030_usb_notifier_call;
	di->phy = usb_get_phy(USB_PHY_TYPE_USB2);
	if (di->phy) {
		ret = usb_register_notifier(di->phy, &di->nb);
		if (ret)
			dev_err(&pdev->dev, "phy register notifier failed %d\n", ret);
	} else
		dev_err(&pdev->dev, "usb_register_notifier failed %d\n", ret);

	di->charger_incurrent_mA = 500;
	if (di->charger_incurrent_mA < 500)
		di->current_limit_mA = di->charger_incurrent_mA;
	else
		di->current_limit_mA = 500;

	/* request charger fault interrupt */
	irq = platform_get_irq(pdev, 1);
	ret = request_threaded_irq(irq, NULL, twl6030_charger_fault_interrupt,
			0, "twl_charger_fault", di);
	if (ret) {
		dev_err(&pdev->dev, "could not request irq %d, status %d\n", irq, ret);
		goto init_failed;
	}

	/* request charger ctrl interrupt */
	irq = platform_get_irq(pdev, 0);
	ret = request_threaded_irq(irq, NULL, twl6030_charger_ctrl_interrupt,
			0, "twl_charger_ctrl", di);
	if (ret) {
		dev_err(&pdev->dev, "could not request irq %d, status %d\n", irq, ret);
		goto chg_irq_fail;
	}

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK, REG_INT_MSK_STS_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK, REG_INT_MSK_STS_C);

	ret = sysfs_create_group(&pdev->dev.kobj, &twl6030_charger_attr_group);
	if (ret)
		dev_err(&pdev->dev, "could not create sysfs files\n");

	/* setup the led state */
	twl6030_init_led_state(di);

	queue_delayed_work(di->wq, &di->monitor_work, 0);

	dev_warn(&pdev->dev, "exit\n");
	return 0;

	/* TODO: fix fail exit mess */
chg_irq_fail:
	irq = platform_get_irq(pdev, 1);
	free_irq(irq, di);

init_failed:
	power_supply_unregister(&di->usb);

usb_failed:
	if (irq != -1)
		free_irq(irq, di);

	wake_lock_destroy(&usb_wake_lock);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	dev_warn(di->dev, "exit with error: %d\n", ret);
	return ret;
}

#define twl6030_charger_suspend NULL
#define twl6030_charger_resume NULL

static const struct dev_pm_ops pm_ops = {
	.suspend = twl6030_charger_suspend,
	.resume = twl6030_charger_resume,
};

static struct platform_driver twl6030_charger_driver = {
	.probe = twl6030_charger_probe,
	.driver = {
		.name = "twl6030_charger",
		.pm = &pm_ops,
	},
};

static int __init twl6030_charger_init(void)
{
	return platform_driver_register(&twl6030_charger_driver);
}
module_init(twl6030_charger_init);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6030_charger");
