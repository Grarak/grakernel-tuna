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
#include <linux/usb/omap_usb.h>
#include <asm/div64.h>
#include <linux/reboot.h>
#include <linux/switch.h>

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

/*
 * The system can be in one of three major states that can
 * tranisition to each other on certain events.
 *
 * Unplugged: The device is not connected to external power.
 *    In this state the battery will discharge due to load.
 *    This state is entered whenever external power is
 *    removed.
 *
 * Charging: The device is connected to exernal power and
 *    the battery is actively being charged. The battery
 *    will only discharge if the system load exceeds the
 *    charger current capability.
 *
 * Not Charging: The device is connected to external power
 *    but the battery is not actively being charged. There
 *    are several conditions that result in this state:
 *    battery full, temperature lockout, invalid/poor
 *    charge source, and software disable.
 */
enum charger_state {
	CHARGER_STATE_UNPLUGGED = 0,
	CHARGER_STATE_CHARGING,
	CHARGER_STATE_NOT_CHARGING,
};

static const char *state_strings[] = {
	"UNPLUGGED",
	"CHARGING",
	"NOT CHARGING",
};

#define state_to_str(state) (state_strings[(state)])

#define is_powered(di) ((di)->state != CHARGER_STATE_UNPLUGGED)
#define is_charging(di) ((di)->state == CHARGER_STATE_CHARGING)
#define is_otg_enabled(di) ((di)->otg_online)

/* thermal limits for battery in tenths of a degree centigrade */
#define CHARGER_LOW_TEMP_LIMIT (100)
#define CHARGER_HIGH_TEMP_LIMIT (450)
#define CHARGER_LOW_TEMP_CRITICAL (-100)
#define CHARGER_HIGH_TEMP_CRITICAL (600)

#define CHARGER_TAPER_CURRENT (20000)

struct twl6030_led_state;

struct charger_info {
	struct device *dev;

	enum charger_state state;
	enum power_supply_type charger_type;

	/* battery state */
	int gg_status;
	int gg_capacity;
	int gg_raw_capacity;
	int gg_voltage_uV;
	int gg_current_uA;
	int gg_temperature_cC;
	int gg_qpassed_mAh;

	/* charger parameters */
	int current_limit_mA;
	int recharge_capacity;

	int termination_current_mA;
	int charger_outcurrent_mA;
	int charger_voltage_mV;

	/* limiting condition flags; protected by mutex */
	int charge_disabled;
	int temperature_lockout;
	int battery_full;

	/* usb state; protected by mutex */
	int vbus_online;
	int usb_online;
	int otg_online;

	/* informational state; protected by mutex */
	int invalid_charger;

	enum power_supply_type charger_supply_type;
	unsigned long usb_event;

	/* shutdown conditions */
	int temperature_critical_count;

	/* charge control irq state */
	u8 charge_control_state;

	/* charge fault irq state */
	u8 charge_fault_int1;
	u8 charge_fault_int2;

	/* power led state */
	struct twl6030_led_state *led;

	unsigned long monitor_interval_jiffies;
	int watchdog_duration;

	struct power_supply usb;
	struct power_supply *battery;

	struct notifier_block notifier;

	struct timer_list timer;
	struct work_struct work;
	struct workqueue_struct *wq;

	struct wake_lock wake_lock;
	struct mutex mutex;

	struct switch_dev sdev;
};

struct twl6030_led_state *twl6030_init_led_state(struct device *dev);
void twl6030_eval_led_state(struct twl6030_led_state *di, bool is_powered, bool is_charging);

static inline void charger_lock(struct charger_info *di)
{
	mutex_lock(&di->mutex);
}

static inline void charger_unlock(struct charger_info *di)
{
	mutex_unlock(&di->mutex);
}

static inline void charger_update_state(struct charger_info *di)
{
	queue_work(di->wq, &di->work);
}

static void charger_timer(unsigned long data)
{
	struct charger_info *di = (struct charger_info *) data;

	charger_update_state(di);
}

static struct power_supply *get_battery_power_supply(
		struct charger_info *di)
{
	if (!di->battery)
		di->battery = power_supply_get_by_name(di->usb.supplied_to[0]);

	return di->battery;
}

/* charger configuration */
static void charger_config_min_vbus_reg(struct charger_info *di,
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
	dev_err(di->dev, "%s: error while accessing i2c: %d\n",
			__func__, ret);
}

static void charger_config_iterm_reg(struct charger_info *di,
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
		dev_err(di->dev, "%s: error while accessing i2c: %d\n",
				__func__, ret);
}

static int charger_get_volimit_config(void)
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

static int charger_get_vilimit_config(void)
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

static int charger_is_volimit_locked(void)
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

static void charger_config_voreg_reg(struct charger_info *di,
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

static void charger_config_vichrg_reg(struct charger_info *di,
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

static void charger_config_cinlimit_reg(struct charger_info *di,
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

static void charger_config_limit1_reg(struct charger_info *di,
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

static void charger_config_limit2_reg(struct charger_info *di,
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

static int charger_pet_watchdog(struct charger_info *di)
{
	return twl_i2c_write_u8(TWL6030_MODULE_CHARGER, di->watchdog_duration, CONTROLLER_WDG);
}

/*
 * Return channel value
 * Or < 0 on failure.
 */
static int charger_get_gpadc_conversion(struct charger_info *di, int channel_no)
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

/* TODO: remove this hack once notle-usb-mux.c takes over the MUX config completely */
extern void usb_mux_force(int use_usb);

static void charger_stop_usb_charger(struct charger_info *di)
{
	int ret;

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
	if (ret)
		goto i2c_error;

	dev_warn(di->dev, "CHARGER OFF\n");

	usb_mux_force(false);

	/* TODO: stop current monitor/control loop */

	return;

i2c_error:
	dev_err(di->dev, "%s: error while accessing i2c: %d\n", __func__, ret);
}

static void charger_start_usb_charger(struct charger_info *di)
{
	int ret;

	dev_dbg(di->dev, "%s: vichrg=%d cinlimit=%d voreg=%d iterm=%d\n",
			__func__, di->charger_outcurrent_mA,
			di->current_limit_mA, di->charger_voltage_mV,
			di->termination_current_mA);

	usb_mux_force(true);

	charger_config_vichrg_reg(di, di->charger_outcurrent_mA);
	charger_config_cinlimit_reg(di, di->current_limit_mA);
	charger_config_voreg_reg(di, di->charger_voltage_mV);
	charger_config_iterm_reg(di, di->termination_current_mA);

	charger_pet_watchdog(di);

	/* disable current termination, suspend mode, boost mode, etc */
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CHARGERUSB_CTRL1);
	if (ret)
		goto i2c_error;

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, CONTROLLER_CTRL1_EN_CHARGER,
			CONTROLLER_CTRL1);
	if (ret)
		goto i2c_error;

	dev_warn(di->dev, "CHARGER ON\n");

	/* TODO: kick off current monitor/control loop */

	return;

i2c_error:
	dev_err(di->dev, "%s: error while accessing i2c: %d\n", __func__, ret);
}

static int charger_ctrl_mask_interrupt(u8 set_mask, u8 clear_mask)
{
	u8 val, mask;
	int ret;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_INT_MASK);
	if (ret)
		goto error;

	mask = (val | set_mask) & ~clear_mask;

	if (mask != val) {
		ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, mask,
				CHARGERUSB_INT_MASK);
		if (ret)
			goto error;
	}

error:
	return ret;
}

static irqreturn_t charger_ctrl_interrupt(int irq, void *_di)
{
	struct charger_info *di = _di;
	u8 stat_toggle, stat_reset, stat_set;
	u8 reg_stat1, reg_hw_cond;
	int charge_control_state;
	int ret;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &reg_stat1, CONTROLLER_STAT1);
	if (ret)
		goto i2c_error;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &reg_hw_cond, STS_HW_CONDITIONS);
	if (ret)
		goto i2c_error;

	charge_control_state = di->charge_control_state;

	stat_toggle = charge_control_state ^ reg_stat1;
	stat_set = stat_toggle & reg_stat1;
	stat_reset = stat_toggle & charge_control_state;

	dev_dbg(di->dev, "%s: stat_toggle=%02x stat_set=%02x stat_reset=%02x hw_cond=%02x\n", __func__,
			stat_toggle, stat_set, stat_reset, reg_hw_cond);

	if (stat_reset & VBUS_DET) {
		dev_info(di->dev, "VBUS_DET OFF\n");

		charger_lock(di);

		di->vbus_online = 0;
		di->otg_online = 0;

		di->invalid_charger = 0;
		switch_set_state(&di->sdev, 0);

		charger_unlock(di);

		charger_update_state(di);
	} else if (stat_set & VBUS_DET) {
		dev_info(di->dev, "VBUS_DET ON\n");

		charger_lock(di);

		if (reg_hw_cond & STS_USB_ID) {
			dev_info(di->dev, "USB_ID ON\n");
			di->otg_online = 1;
		}

		di->vbus_online = 1;

		charger_unlock(di);

		charger_update_state(di);
	}

	di->charge_control_state = reg_stat1;

	return IRQ_HANDLED;

i2c_error:
	dev_err(di->dev, "%s: error while accessing i2c: %d\n", __func__, ret);
	return IRQ_HANDLED;
}

static irqreturn_t charger_fault_interrupt(int irq, void *_di)
{
	struct charger_info *di = _di;
	int ret;

	u8 reg_stat, reg_int1, reg_int2;
	u8 int1_toggle, int2_toggle;
	u8 int1_set, int2_set;
	u8 int1_reset, int2_reset;
	u8 charge_fault_int1, charge_fault_int2;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &reg_stat,
						CHARGERUSB_INT_STATUS);
	if (ret)
		goto err;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &reg_int1,
						CHARGERUSB_STATUS_INT1);
	if (ret)
		goto err;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &reg_int2,
						CHARGERUSB_STATUS_INT2);
	if (ret)
		goto err;

	charge_fault_int1 = di->charge_fault_int1;
	charge_fault_int2 = di->charge_fault_int2;

	int1_toggle = charge_fault_int1 ^ reg_int1;
	int2_toggle = charge_fault_int2 ^ reg_int2;

	int1_set = int1_toggle & reg_int1;
	int2_set = int2_toggle & reg_int2;

	int1_reset = int1_toggle & charge_fault_int1;
	int2_reset = int2_toggle & charge_fault_int2;

	di->charge_fault_int1 = reg_int1;
	di->charge_fault_int2 = reg_int2;

	dev_dbg(di->dev, "%s: int1_set=%02x int1_reset=%02x int2_set=%02x int2_reset=%02x\n",
			__func__, int1_set, int1_reset, int2_set, int2_reset);

	if ((int1_set & (CHARGERUSB_STATUS_INT1_POOR_SRC | CHARGERUSB_STATUS_INT1_SLP_MODE))
			|| (int2_set & (CURRENT_TERM | ANTICOLLAPSE))) {
		charger_lock(di);

		di->invalid_charger = 1;
		switch_set_state(&di->sdev, 1);

		charger_unlock(di);

		charger_update_state(di);
	}

err:
	return IRQ_HANDLED;
}

static enum power_supply_property charger_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int charger_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct charger_info *di = container_of(psy, struct charger_info, usb);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = di->vbus_online;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = charger_get_gpadc_conversion(di, 10) * 1000;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int charger_usb_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct charger_info *di = container_of(nb, struct charger_info, notifier);
	enum power_supply_type *supply = data;

	charger_lock(di);

	switch (event) {
		case TWL6030_USB_EVENT_VBUS_ON:
			di->usb_online = 1;
			break;

		case TWL6030_USB_EVENT_VBUS_OFF:
		case TWL6030_USB_EVENT_OTG_OFF:
			di->charger_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
			di->usb_online = 0;
			break;

		case TWL6030_USB_EVENT_VBUS_DETECT:
			di->charger_supply_type = *supply;
			break;

		default:
			goto done;
	}

	di->usb_event = event;

	dev_info(di->dev, "usb_event=%ld usb_online=%d charger_supply_type=%d\n",
			di->usb_event, di->usb_online, di->charger_supply_type);

done:
	charger_unlock(di);

	charger_update_state(di);

	return NOTIFY_OK;
}

static void charger_gas_gauge_update(struct charger_info *di)
{
	struct power_supply *battery;
	union power_supply_propval val;
	int capacity;
	int raw_capacity;
	int voltage_uV;
	int current_uA;
	int temperature_cC;
	int qpassed_mAh;
	int status;
	int ret;

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

	/* store values in device info */
	di->gg_capacity = capacity;
	di->gg_raw_capacity = raw_capacity;
	di->gg_status = status;
	di->gg_voltage_uV = voltage_uV;
	di->gg_current_uA = current_uA;
	di->gg_temperature_cC = temperature_cC;
	di->gg_qpassed_mAh = qpassed_mAh;

	dev_info(di->dev, "capacity=%d%% raw_capacity=%d%% status=%02x voltage_uV=%d current_uA=%d "
			"temperature_cC=%d qpassed_mAh=%d state=%s\n",
			di->gg_capacity, di->gg_raw_capacity, di->gg_status, di->gg_voltage_uV,
			di->gg_current_uA, di->gg_temperature_cC, di->gg_qpassed_mAh, state_to_str(di->state));

error:
	return;
}

static void charger_check_temp_limits(struct charger_info *di)
{
	/* check against temperature limits */
	if (di->gg_temperature_cC < CHARGER_LOW_TEMP_LIMIT || di->gg_temperature_cC > CHARGER_HIGH_TEMP_LIMIT) {
		if (di->temperature_lockout == 0)
			dev_warn(di->dev, "temperature is outside charging limits\n");
		di->temperature_lockout = 1;
	} else {
		di->temperature_lockout = 0;
	}

	/* check against critical temperature limits */
	if (di->gg_temperature_cC < CHARGER_LOW_TEMP_CRITICAL || di->gg_temperature_cC > CHARGER_HIGH_TEMP_CRITICAL) {
		di->temperature_critical_count++;

		if (di->temperature_critical_count >= 2) {
			dev_err(di->dev, "temperature is critical, powering down!!\n");
			kernel_power_off();
		}
	}
}

static void charger_check_battery_level(struct charger_info *di)
{
	/* check for battery full condition */
	if (is_charging(di) && di->gg_capacity == 100) {
		if (di->gg_status == POWER_SUPPLY_STATUS_FULL) {
			dev_warn(di->dev, "gas gauge reports full capacity\n");
			di->battery_full = 1;
		} else if (di->gg_current_uA < CHARGER_TAPER_CURRENT) {
			dev_warn(di->dev, "gas gauge did not report full capacity\n");
			di->battery_full = 1;
		}
	}

	if (!is_powered(di))
		di->battery_full = 0;

	if (di->battery_full && di->gg_capacity <= di->recharge_capacity) {
		if (is_powered(di) && !is_charging(di))
			dev_warn(di->dev, "battery drained from full to %d%%, charging again\n", di->gg_capacity);
		di->battery_full = 0;
	}
}

static void charger_work(struct work_struct *work)
{
	struct charger_info *di = container_of(work, struct charger_info, work);
	int next_state;
	bool limiting_active;
	bool charge_source;

	if (is_charging(di))
		charger_pet_watchdog(di);

	charger_lock(di);

	/* update inputs and limit conditions */
	charger_gas_gauge_update(di);
	charger_check_temp_limits(di);
	charger_check_battery_level(di);

	/* aggregate limiting factors */
	limiting_active = di->battery_full || di->charge_disabled ||
		di->temperature_lockout;

	/* hardware ready for charging? */
	charge_source = di->vbus_online && !di->otg_online;

	dev_dbg(di->dev, "%s: battery_full=%d charge_disabled=%d temperature_lockout=%d "
			"invalid_charger=%d vbus_online=%d usb_online=%d otg_online=%d\n", __func__,
			di->battery_full, di->charge_disabled, di->temperature_lockout,
			di->invalid_charger, di->vbus_online, di->usb_online, di->otg_online);

	/* determine the next charger state */
	switch (di->state) {
		case CHARGER_STATE_UNPLUGGED:
		case CHARGER_STATE_CHARGING:
			if (charge_source) {
				if (limiting_active)
					next_state = CHARGER_STATE_NOT_CHARGING;
				else
					next_state = CHARGER_STATE_CHARGING;
			} else {
				next_state = CHARGER_STATE_UNPLUGGED;
			}
			break;

		case CHARGER_STATE_NOT_CHARGING:
			if (charge_source) {
				if (limiting_active)
					next_state = CHARGER_STATE_NOT_CHARGING;
				else
					next_state = CHARGER_STATE_CHARGING;
			} else {
				next_state = CHARGER_STATE_UNPLUGGED;
			}
			break;

		default:
			next_state = di->state; // silence compiler warning
	}

	/* if the state changed, fulfill the required changes */
	if (next_state != di->state) {
		dev_info(di->dev, "transitioning from %s to %s\n", state_to_str(di->state),
				state_to_str(next_state));

		switch (next_state) {
			case CHARGER_STATE_UNPLUGGED:
				wake_lock_timeout(&di->wake_lock, HZ / 2);
				charger_stop_usb_charger(di);
				break;

			case CHARGER_STATE_NOT_CHARGING:
				if (!is_powered(di))
					wake_lock(&di->wake_lock);

				charger_stop_usb_charger(di);
				break;

			case CHARGER_STATE_CHARGING:
				if (!is_powered(di))
					wake_lock(&di->wake_lock);

				charger_start_usb_charger(di);
				break;
		}

		di->state = next_state;
		power_supply_changed(&di->usb);

		twl6030_eval_led_state(di->led, is_powered(di), is_charging(di));
	}

	charger_unlock(di);

	/* update timer */
	mod_timer(&di->timer, di->monitor_interval_jiffies + jiffies);
}

static ssize_t show_vbus_voltage(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct charger_info *di = dev_get_drvdata(dev);

	val = charger_get_gpadc_conversion(di, 10);

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_recharge_capacity(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct charger_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 100))
		return -EINVAL;

	charger_lock(di);

	di->recharge_capacity = val;

	charger_unlock(di);

	return status;
}

static ssize_t show_recharge_capacity(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	unsigned int val;
	struct charger_info *di = dev_get_drvdata(dev);

	val = di->recharge_capacity;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_charge_disabled(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	bool changed;
	int charge_disabled;
	struct charger_info *di = dev_get_drvdata(dev);

	if (strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;

	charger_lock(di);

	charge_disabled = di->charge_disabled;
	di->charge_disabled = !!val;

	changed = di->charge_disabled != charge_disabled;

	charger_unlock(di);

	if (changed)
		charger_update_state(di);

	return count;
}

static ssize_t show_charge_disabled(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	unsigned int val;
	struct charger_info *di = dev_get_drvdata(dev);

	val = di->charge_disabled;
	return sprintf(buf, "%u\n", !!val);
}

static DEVICE_ATTR(vbus_voltage, S_IRUGO, show_vbus_voltage, NULL);
static DEVICE_ATTR(recharge_capacity, S_IWUSR | S_IRUGO, show_recharge_capacity,
		set_recharge_capacity);
static DEVICE_ATTR(charge_disabled, S_IWUSR | S_IRUGO, show_charge_disabled,
		set_charge_disabled);

static struct attribute *charger_charger_attributes[] = {
	&dev_attr_vbus_voltage.attr,
	&dev_attr_recharge_capacity.attr,
	&dev_attr_charge_disabled.attr,
	NULL,
};

static const struct attribute_group charger_charger_attr_group = {
	.attrs = charger_charger_attributes,
};

static int __devinit charger_probe(struct platform_device *pdev)
{
	struct twl4030_charger_platform_data *pdata = pdev->dev.platform_data;
	struct charger_info *di;
	struct device *dev = &pdev->dev;
	int irq = -1;
	int ret;

	dev_warn(dev, "enter\n");

	if (!pdata) {
		dev_dbg(dev, "platform_data not available\n");
		return -EINVAL;
	}

	if (!pdata->supplied_to || pdata->num_supplicants < 1) {
		dev_err(dev, "a supplicant must be sepcified; "
				"supplied_to=%p num=%zu\n", pdata->supplied_to,
				pdata->num_supplicants);
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->recharge_capacity = 94;
	di->monitor_interval_jiffies = 15 * HZ;

	di->usb.name = "twl6030_usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = charger_usb_props;
	di->usb.num_properties = ARRAY_SIZE(charger_usb_props);
	di->usb.get_property = charger_usb_get_property;
	di->usb.supplied_to = pdata->supplied_to;
	di->usb.num_supplicants = pdata->num_supplicants;

	if (pdata->num_supplicants > 1) {
		dev_warn(dev, "more than one supplicant specified (%zu), only the "
				"first will be used\n", pdata->num_supplicants);
	}

	wake_lock_init(&di->wake_lock, WAKE_LOCK_SUSPEND, "charger_wake_lock");
	mutex_init(&di->mutex);

	di->wq = create_freezable_workqueue(dev_name(di->dev));
	INIT_WORK(&di->work, charger_work);

	setup_timer(&di->timer, charger_timer, (unsigned long) di);

	/* see if the bootloader already set and locked the limits */
	if (charger_is_volimit_locked() == 1) {
		/* read the vo and vi reg limits set in the bootloader */
		di->charger_voltage_mV = charger_get_volimit_config();
		di->charger_outcurrent_mA = charger_get_vilimit_config();

		dev_warn(&pdev->dev, "Using limits set by bootloader: %d mV %d mA\n",
				di->charger_voltage_mV, di->charger_outcurrent_mA);
	} else {
		/* use the conservative platform data defaults for safety */
		di->charger_voltage_mV = pdata->max_bat_voltage_mV;
		di->charger_outcurrent_mA = pdata->max_charger_current_mA;

		charger_config_limit1_reg(di, pdata->max_charger_voltage_mV);
		charger_config_limit2_reg(di, pdata->max_charger_current_mA);

		dev_warn(&pdev->dev, "Using default limits: %d mV %d mA\n",
				di->charger_voltage_mV, di->charger_outcurrent_mA);
	}

	di->current_limit_mA = 500;

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
			CONTROLLER_INT_MASK);
	if (ret)
		goto init_failed;

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
			MASK_MCHARGERUSB_THMREG | MASK_MCURRENT_TERM,
			CHARGERUSB_INT_MASK);
	if (ret)
		goto init_failed;


	/* setup the initial watchdog state */
	di->watchdog_duration = 32;
	charger_pet_watchdog(di);

	ret = power_supply_register(dev, &di->usb);
	if (ret) {
		dev_err(dev, "failed to register usb power supply\n");
		goto usb_failed;
	}

	di->notifier.notifier_call = charger_usb_notifier_call;
	ret = twl6030_usb_register_notifier(&di->notifier);
	if (ret)
		dev_err(dev, "twl6030_usb_register_notifier failed %d\n", ret);

	/* request charger fault interrupt */
	irq = platform_get_irq(pdev, 1);
	ret = request_threaded_irq(irq, NULL, charger_fault_interrupt,
			0, "twl_charger_fault", di);
	if (ret) {
		dev_err(dev, "could not request irq %d, status %d\n", irq, ret);
		goto init_failed;
	}

	/* request charger ctrl interrupt */
	irq = platform_get_irq(pdev, 0);
	ret = request_threaded_irq(irq, NULL, charger_ctrl_interrupt,
			0, "twl_charger_ctrl", di);
	if (ret) {
		dev_err(dev, "could not request irq %d, status %d\n", irq, ret);
		goto chg_irq_fail;
	}

	di->sdev.name = "invalid_charger";
	ret = switch_dev_register(&di->sdev);
	if (ret) {
		dev_err(di->dev, "error registering switch device: %d\n", ret);
		goto chg_irq_fail;
	}

	ret = sysfs_create_group(&dev->kobj, &charger_charger_attr_group);
	if (ret)
		dev_err(dev, "could not create sysfs files\n");

	/* setup the led state */
	di->led = twl6030_init_led_state(di->dev);
	if (!di->led) {
		dev_err(dev, "failed to alloc led state\n");
		goto chg_irq_fail;
	}

	/* kick ctrl isr to get initial state */
	charger_ctrl_interrupt(0, di);

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK, REG_INT_MSK_STS_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK, REG_INT_MSK_STS_C);

	charger_update_state(di);

	dev_warn(dev, "exit\n");
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

	wake_lock_destroy(&di->wake_lock);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	dev_warn(di->dev, "exit with error: %d\n", ret);
	return ret;
}

#define charger_suspend NULL
#define charger_resume NULL

static const struct dev_pm_ops pm_ops = {
	.suspend = charger_suspend,
	.resume = charger_resume,
};

static struct platform_driver charger_driver = {
	.probe = charger_probe,
	.driver = {
		.name = "twl6030_charger",
		.pm = &pm_ops,
	},
};

static int __init charger_init(void)
{
	return platform_driver_register(&charger_driver);
}
module_init(charger_init);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6030_charger");
