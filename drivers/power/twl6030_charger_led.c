/*
 * linux/drivers/power/twl6030_charger_led.c
 *
 * Copyright (C) 2013 Google, Inc.
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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>
#include <asm/div64.h>

#define LED_PWM_CTRL1                     0x14
#define LED_PWM_CTRL2                     0x15

enum led_mode {
	LED_PWM_AUTO = 0x0,
	LED_PWM_ON = 0x1,
	LED_PWM_OFF= 0x2,
};

#define LED_STATE_NONE                   -1
#define LED_STATE_OFF                     0
#define LED_STATE_ON                      1
#define LED_STATE_RAMP                    2
#define LED_STATE_OSCILLATE               3
#define LED_UPDATE_RATE_MS               66
#define LED_RAMPUP_TIME_MS             1280

struct twl6030_led_state {
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

	struct timer_list timer;
	struct work_struct work;
	struct workqueue_struct *wq;
};

static void twl6030_led_work(struct work_struct *work);

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

static void twl6030_set_led(struct twl6030_led_state *di,
		enum led_mode mode, int pwm)
{
	if (pwm >= 0) {
		di->cur = pwm;
		twl6030_set_pwm_level(pwm & 0xff);
	}

	di->mode = mode;
	twl6030_set_led_mode(mode);
}

static void twl6030_led_timer(unsigned long data)
{
	struct twl6030_led_state *di = (struct twl6030_led_state *) data;
	queue_work(di->wq, &di->work);
}

/*
 * Read the hardware state and setup the led state struct to match.
 */
struct twl6030_led_state *twl6030_init_led_state(struct device *dev)
{
	struct twl6030_led_state *di;
	int pwm, mode;

	di = kzalloc(sizeof(struct twl6030_led_state), GFP_KERNEL);
	if (!di)
		goto done;

	pwm = twl6030_get_pwm_level();
	mode = twl6030_get_led_mode();

	switch (di->mode) {
		case LED_PWM_OFF:
			di->state = LED_STATE_OFF;
			di->cur = pwm;
			di->mode = mode;
			break;

		case LED_PWM_ON:
			di->state = LED_STATE_ON;
			di->cur = pwm;
			di->mode = mode;
			break;

		default:
			// set the state to off, since we don't know if the bootloader left the charger on
			twl6030_set_led(di, LED_PWM_OFF, pwm);
			di->state = LED_STATE_OFF;
			break;
	}

	di->next = LED_STATE_NONE;
	di->rate = 0;
	di->target = di->cur;
	di->base = 0;
	di->led_low = 0x00;
	di->led_high = 0xff;
	di->led_step = (di->led_high - di->led_low) / (LED_RAMPUP_TIME_MS / LED_UPDATE_RATE_MS);

	setup_timer(&di->timer, twl6030_led_timer, (unsigned long) di);

	di->wq = create_freezable_workqueue(dev_name(dev));
	INIT_WORK(&di->work, twl6030_led_work);

done:
	return di;
}

static int twl6030_led_add(struct twl6030_led_state *di, int inc)
{
	int value = di->cur + inc;

	/* if the increment crosses the target, clip to target */
	if ((value < di->target && di->cur > di->target) ||
			(value > di->target && di->cur < di->target)) {
		value = di->target;
	}

	return value;
}

static void twl6030_led_run(struct twl6030_led_state *di)
{
	switch (di->state) {
		case LED_STATE_OFF:
			twl6030_set_led(di, LED_PWM_OFF, -1);

			if (di->next > LED_STATE_NONE) {
				di->state = di->next;
				di->next = LED_STATE_NONE;
				mod_timer(&di->timer, msecs_to_jiffies(di->rate) + jiffies);
			}
			break;

		case LED_STATE_ON:
			twl6030_set_led(di, LED_PWM_ON, di->cur);

			if (di->next > LED_STATE_NONE) {
				di->state = di->next;
				di->next = LED_STATE_NONE;
				mod_timer(&di->timer, msecs_to_jiffies(di->rate) + jiffies);
			}
			break;

		case LED_STATE_RAMP:
			if (di->mode != LED_PWM_ON)
				twl6030_set_led(di, LED_PWM_ON, 0);

			if (di->cur < di->target) {
				di->cur = twl6030_led_add(di, di->led_step);
				twl6030_set_pwm_level(di->cur);
			} else if (di->cur > di->target) {
				di->cur = twl6030_led_add(di, -di->led_step);
				twl6030_set_pwm_level(di->cur);
			}

			if (di->cur == di->target) {
				di->state = di->next;
				di->next = LED_STATE_NONE;
			}

			mod_timer(&di->timer, msecs_to_jiffies(di->rate) + jiffies);
			break;

		case LED_STATE_OSCILLATE:
			if (di->mode != LED_PWM_ON)
				twl6030_set_led(di, LED_PWM_ON, 0);

			if (di->cur < di->target) {
				di->cur = twl6030_led_add(di, di->led_step);
				twl6030_set_pwm_level(di->cur);
			} else if (di->cur > di->target) {
				di->cur = twl6030_led_add(di, -di->led_step);
				twl6030_set_pwm_level(di->cur);
			}

			if (di->cur == di->target) {
				int temp = di->target;
				di->target = di->base;
				di->base = temp;
			}

			mod_timer(&di->timer, msecs_to_jiffies(di->rate) + jiffies);
			break;
	}
}

static void twl6030_led_work(struct work_struct *work)
{
	struct twl6030_led_state *di = container_of(work, struct twl6030_led_state, work);
	twl6030_led_run(di);
}

void twl6030_eval_led_state(struct twl6030_led_state *di, bool is_powered, bool is_charging)
{
	if (is_powered) {
		if (is_charging) {
			if (di->state != LED_STATE_OSCILLATE) {
				del_timer_sync(&di->timer);

				di->state = LED_STATE_OSCILLATE;
				di->target = di->led_high;
				di->base = di->led_low;
				di->rate = LED_UPDATE_RATE_MS;

				twl6030_led_run(di);
			}
		} else {
			if (di->state != LED_STATE_ON &&
					!(di->state == LED_STATE_RAMP && di->next == LED_STATE_ON
						&& di->target == (di->led_high >> 2))) {
				del_timer_sync(&di->timer);

				di->state = LED_STATE_RAMP;
				di->target = di->led_high >> 2;
				di->rate = LED_UPDATE_RATE_MS;
				di->next = LED_STATE_ON;

				twl6030_led_run(di);
			}
		}
	} else {
		if (di->state != LED_STATE_OFF && !(di->state == LED_STATE_RAMP
					&& di->next == LED_STATE_OFF && di->target == 0x00)) {
			del_timer_sync(&di->timer);

			di->state = LED_STATE_RAMP;
			di->target = 0x00;
			di->rate = LED_UPDATE_RATE_MS;
			di->next = LED_STATE_OFF;

			twl6030_led_run(di);
		}
	}
}

