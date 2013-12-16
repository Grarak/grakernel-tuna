/*
 * arch/arm/mach-omap2/notle-usb-mux.c
 *
 * Notle board USB MUX control driver.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/switch.h>
#include <linux/usb/omap_usb.h>
#include <sound/soc.h>

#include "board-notle.h"
#include "notle-usb-mux.h"

#define FACTORY_CABLE_VOLTAGE_THRESHOLD		4200	/* millivolts */
#define CABLE_SHUTDOWN_INTERVAL				30000	/* msec */
#define RID_POLL_INTERVAL					1000	/* msec */
#define ID_READ_DELAY						2		/* msec */
#define ID_READ_DELAY_NONE					0
/* gross min/max levels for Rid */
#define RIDV_500K_MIN						2100
#define RIDV_500K_MAX						2540
#define RIDV_1M_MIN							2541
#define RIDV_1M_MAX							2900

enum usb_mux_mode {
	USB_MUX_MODE_FLOATING = 0,
	USB_MUX_MODE_USB      = 1,
	USB_MUX_MODE_TTY      = 2,
	USB_MUX_MODE_AUDIO    = 3,
};

enum headset_jack_connection {
	HS_JACK_UNKNOWN = -1,
	HS_JACK_NONE = 0,
	HS_JACK_MONO = 1,
	HS_JACK_STEREO = 2,
};

struct usb_mux_device_info {
	struct device *dev;

	int gpio_cb0;
	int gpio_cb1;

	bool force_usb;
	bool usb_online;
	bool tty_priority;
	bool factorycable_bootmode;

	enum usb_mux_mode cur_mode;
	enum usb_mux_mode req_mode;

	struct usb_phy* otg;
	struct notifier_block nb;

	struct mutex lock;

	struct workqueue_struct *wq;
	struct work_struct work;

	/* for headset detection with Rid to control mux */
	enum headset_jack_connection connected;
	bool monitor_id;
	struct delayed_work id_work;
	struct snd_soc_jack *jack;
	int report;
	struct switch_dev sdev;
};

static struct usb_mux_device_info *usb_mux_di;

/*
 * The following is a flag to prevent a reboot into
 * fastboot if factory cable is plugged in.  If it is
 * set then don't reboot.
 */
static bool notle_factorycable_bootmode = false;
#ifdef CONFIG_FIQ_DEBUGGER_CONSOLE_DEFAULT_ENABLE
static bool tty_priority = true;
#else
static bool tty_priority = false;
#endif

/* forward decl prototypes */
static int voltage_id(int, int);
static int get_jack_device(struct usb_mux_device_info *di);

static const char *str_for_mode(enum usb_mux_mode mode)
{
	switch (mode) {
		case USB_MUX_MODE_FLOATING:
			return "floating";

		case USB_MUX_MODE_USB:
			return "usb";

		case USB_MUX_MODE_TTY:
			return "tty";

		case USB_MUX_MODE_AUDIO:
			return "audio";

		default:
			return "unknown";
	}
}

/*
 * Syncs the GPIO MUX control lines with the requested MUX mode.
 * If factory cable is plugged in then reset into fastboot unless we
 * booted with the factorycable_bootmode flag set.
 *
 * Call with di->lock mutex held.
 */
static void sync_usb_mux_mode(struct usb_mux_device_info *di)
{
	enum usb_mux_mode mode;
	int id_voltage;

	/* check for factory cable */
	id_voltage = voltage_id(0, ID_READ_DELAY_NONE);
	if (!di->factorycable_bootmode && id_voltage > FACTORY_CABLE_VOLTAGE_THRESHOLD) {
		kernel_restart("bootloader");
	}

	if (di->force_usb || di->usb_online) {
		mode = USB_MUX_MODE_USB;
	} else if (di->tty_priority) {
		mode = USB_MUX_MODE_TTY;
	} else {
		mode = di->req_mode;
	}

	dev_warn(di->dev, "Setting MUX mode to %s\n", str_for_mode(mode));

	/* always go to float state first */
	if (mode != di->cur_mode) {
		gpio_set_value(di->gpio_cb0, 0);
		gpio_set_value(di->gpio_cb1, 0);
	}

	switch (mode) {
		case USB_MUX_MODE_FLOATING:
			gpio_set_value(di->gpio_cb1, 0);
			gpio_set_value(di->gpio_cb0, 0);
			break;

		case USB_MUX_MODE_USB:
			gpio_set_value(di->gpio_cb1, 0);
			gpio_set_value(di->gpio_cb0, 1);
			break;

		case USB_MUX_MODE_TTY:
			gpio_set_value(di->gpio_cb1, 1);
			gpio_set_value(di->gpio_cb0, 0);
			break;

		case USB_MUX_MODE_AUDIO:
			gpio_set_value(di->gpio_cb1, 1);
			gpio_set_value(di->gpio_cb0, 1);
			break;

		default:
			dev_err(di->dev, "Invalid USB MUX mode requested: %d\n", mode);
			goto error;
	}

	di->cur_mode = mode;

error:
	return;
}

static void request_usb_mux_mode(struct usb_mux_device_info *di, enum usb_mux_mode mode)
{
	mutex_lock(&di->lock);

	di->req_mode = mode;
	sync_usb_mux_mode(di);

	mutex_unlock(&di->lock);
}

static enum usb_mux_mode get_usb_mux_mode(struct usb_mux_device_info *di)
{
	enum usb_mux_mode mode;

	mutex_lock(&di->lock);
	mode = di->cur_mode;
	mutex_unlock(&di->lock);

	return mode;
}

/*
 * called from detect (at init time) and when polling detects change
 * report is just a flag to determine how to pass state to snd_soc_jack_report
 * i.e. report value of 0 "mutes" reporting of jack state, will always report
 * switch of 0
 * XXX - figure out mic vs no mic  is it 0x1 vs 0x2? What do we want to report
 */
static void notle_hs_jack_report(struct usb_mux_device_info *di)
{
	int state = 0;

	mutex_lock(&di->lock);
	di->connected = get_jack_device(di);
	mutex_unlock(&di->lock);
	if (di->connected != HS_JACK_NONE)
		state = di->report;

	/* the following only do work if state has changed */
	snd_soc_jack_report(di->jack, state, di->report);
	switch_set_state(&di->sdev, di->connected);
}

/* called at audio init time */
void notle_hs_jack_detect(struct snd_soc_codec *codec,
				struct snd_soc_jack *jack, int report)
{
	struct usb_mux_device_info *di = usb_mux_di;

	if (!di)
		return;

	di->jack = jack;
	di->report = report;
	di->connected = HS_JACK_UNKNOWN;

	notle_hs_jack_report(di);

	/* set the mux based on connection */
	if (di->connected == HS_JACK_STEREO || di->connected == HS_JACK_MONO) {
		request_usb_mux_mode(di, USB_MUX_MODE_FLOATING);
	} else {
		request_usb_mux_mode(di, USB_MUX_MODE_TTY);
	}
}
EXPORT_SYMBOL_GPL(notle_hs_jack_detect);

/*
 * deferred call to take care of mux and check voltage on ID line
 */
static void usb_mux_work(struct work_struct *work)
{
	struct usb_mux_device_info *di = container_of(work,
			struct usb_mux_device_info, work);

	mutex_lock(&di->lock);
	sync_usb_mux_mode(di);
	mutex_unlock(&di->lock);
}

/*
 * hack to allow charger to force usb mux to usb mode before turning on charger.
 * remove this when this driver takes over USB ID duties and have this driver call
 * a notifier chain that the charger will use to know when it's safe to engage
 */
void usb_mux_force(int use_usb)
{
	struct usb_mux_device_info *di = usb_mux_di;

	if (!di)
		return;

	mutex_lock(&di->lock);
	di->force_usb = !!use_usb;
	sync_usb_mux_mode(di);
	mutex_unlock(&di->lock);
}
EXPORT_SYMBOL(usb_mux_force);

static int usb_mux_usb_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct usb_mux_device_info *di =
		container_of(nb, struct usb_mux_device_info, nb);
	bool usb_online = di->usb_online;

	switch (event) {
		case USB_EVENT_VBUS:
		case USB_EVENT_ENUMERATED:
		case USB_EVENT_CHARGER:
		case USB_EVENT_ID:
			di->usb_online = true;
			break;

		case USB_EVENT_NONE:
			di->usb_online = false;
			break;
	}

	if (usb_online != di->usb_online)
		queue_work(di->wq, &di->work);

	return NOTIFY_OK;
}

/*
 * Interface to twl6030_gpadc_conversion() to return channel value
 * Or < 0 on failure.
 */
static int twl6030_get_gpadc_conversion(int channel_no)
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

static int voltage_id(int mode, int ms_delay)
{
	int voltage;

	mode |= TWL6030_USB_ID_CTRL_MEAS;
	twl_i2c_write_u8(TWL_MODULE_USB, ~(1<<2), REG_USB_ID_CTRL_CLR);
	twl_i2c_write_u8(TWL_MODULE_USB, mode, REG_USB_ID_CTRL_SET);
	if (ms_delay) {
		msleep(ms_delay);
	}
	voltage = twl6030_get_gpadc_conversion(TWL6030_GPADC_ID_CHANNEL);
	return voltage;
}

/*
 * Use r200pu to get Rid or float, return 1 of 3 states
 * Call with di->lock mutex held.
 */
static int get_jack_device(struct usb_mux_device_info *di)
{
	u8 save_idctrl, save_backup;
	int pu220_val;
	enum headset_jack_connection jack = HS_JACK_NONE;

	twl_i2c_read_u8(TWL_MODULE_USB, &save_idctrl, REG_USB_ID_CTRL_SET);
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &save_backup, REG_OTG_BACKUP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, save_backup & ~TWL6030_OTG_BACKUP_ID_WKUP, REG_OTG_BACKUP);

	pu220_val = voltage_id(TWL6030_USB_ID_CTRL_PU_220K, ID_READ_DELAY_NONE);

	/* Restore old values */
	twl_i2c_write_u8(TWL_MODULE_USB, ~(1<<2), REG_USB_ID_CTRL_CLR);
	twl_i2c_write_u8(TWL_MODULE_USB, save_idctrl, REG_USB_ID_CTRL_SET);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, save_backup, REG_OTG_BACKUP);

	/* Wide range of values until we need accurate Rid */
	if (RIDV_500K_MIN <= pu220_val && pu220_val <= RIDV_500K_MAX)
		jack = HS_JACK_STEREO;
	else if (RIDV_1M_MIN <= pu220_val && pu220_val <= RIDV_1M_MAX)
		jack = HS_JACK_MONO;

	return jack;
}

static void usb_id_work(struct work_struct *id_work)
{
	struct usb_mux_device_info *di = container_of(id_work,
			struct usb_mux_device_info, id_work.work);

	notle_hs_jack_report(di);
	if (di->monitor_id) {
		queue_delayed_work(di->wq, &di->id_work, msecs_to_jiffies(RID_POLL_INTERVAL));
	}
}

static ssize_t set_mode(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int status = count;
	struct usb_mux_device_info *di = dev_get_drvdata(dev);

	enum usb_mux_mode mode = USB_MUX_MODE_TTY;

	/* PARANOID: count should be at least 1, but make sure anyway */
	if (!count)
		count = 1;

	if (!strncmp(buf, "floating", count-1))
		mode = USB_MUX_MODE_FLOATING;
	else if (!strncmp(buf, "usb", count-1))
		mode = USB_MUX_MODE_USB;
	else if (!strncmp(buf, "tty", count-1))
		mode = USB_MUX_MODE_TTY;
	else if (!strncmp(buf, "audio", count-1))
		mode = USB_MUX_MODE_AUDIO;
	else
		status = -EINVAL;

	dev_warn(di->dev, "mode=%s status=%d count=%zu buf=%s\n", str_for_mode(mode),
			status, count, buf);

	if (status > 0)
		request_usb_mux_mode(di, mode);

	return status;
}

static ssize_t show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	const char *val;
	struct usb_mux_device_info *di = dev_get_drvdata(dev);

	enum usb_mux_mode mode = get_usb_mux_mode(di);
	val = str_for_mode(mode);

	return sprintf(buf, "%s\n", val);
}

static ssize_t set_id_monitor(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_mux_device_info *di = dev_get_drvdata(dev);
	int r, value;

	r = kstrtoint(buf, 0, &value);
	if (r)
		return r;

	value = !!value;
	if (di->monitor_id != value) {
		di->monitor_id = value;
		if (value) {
			queue_delayed_work(di->wq, &di->id_work, msecs_to_jiffies(RID_POLL_INTERVAL));
		}
	}
	return count;
}

static ssize_t show_id_monitor(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct usb_mux_device_info *di = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", di->monitor_id);
}

/*
 * Capture a number of different voltages on ID line with
 * different sources and sinks.  Useful for experiments
 */
static ssize_t show_id_allvoltages(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct usb_mux_device_info *di = dev_get_drvdata(dev);
	u8 save_idctrl, save_backup;
	u8 sts_c, int_src, trim15, trim16;
	int pu100pd_val, pu100_val, pu220pd_val, pu220_val;
	int src16u_val, src5u_val;
	int src16upd_val, src5upd_val, src16upu220pd_val;
	int n;

	mutex_lock(&di->lock);
	/* Setup and General info */
	twl_i2c_read_u8(TWL_MODULE_USB, &int_src, REG_USB_ID_INT_SRC);
	twl_i2c_read_u8(TWL_MODULE_USB, &sts_c, REG_INT_STS_C);
	twl_i2c_read_u8(TWL6030_MODULE_ID2, &trim15, REG_GPADC_TRIM15);
	twl_i2c_read_u8(TWL6030_MODULE_ID2, &trim16, REG_GPADC_TRIM16);

	twl_i2c_read_u8(TWL_MODULE_USB, &save_idctrl, REG_USB_ID_CTRL_SET);
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &save_backup, REG_OTG_BACKUP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, save_backup & ~TWL6030_OTG_BACKUP_ID_WKUP, REG_OTG_BACKUP);

	pu100pd_val = voltage_id(TWL6030_USB_ID_CTRL_PU_100K | TWL6030_USB_ID_CTRL_GND_DRV, ID_READ_DELAY);
	pu220pd_val = voltage_id(TWL6030_USB_ID_CTRL_PU_220K | TWL6030_USB_ID_CTRL_GND_DRV, ID_READ_DELAY);
	pu100_val = voltage_id(TWL6030_USB_ID_CTRL_PU_100K, ID_READ_DELAY);
	pu220_val = voltage_id(TWL6030_USB_ID_CTRL_PU_220K, ID_READ_DELAY);
	src5u_val = voltage_id(TWL6030_USB_ID_CTRL_SRC_5U, ID_READ_DELAY);
	src16u_val = voltage_id(TWL6030_USB_ID_CTRL_SRC_16U, ID_READ_DELAY);
	src5upd_val = voltage_id(TWL6030_USB_ID_CTRL_SRC_5U | TWL6030_USB_ID_CTRL_GND_DRV, ID_READ_DELAY);
	src16upd_val = voltage_id(TWL6030_USB_ID_CTRL_SRC_16U | TWL6030_USB_ID_CTRL_GND_DRV, ID_READ_DELAY);
	src16upu220pd_val = voltage_id(TWL6030_USB_ID_CTRL_SRC_16U | TWL6030_USB_ID_CTRL_PU_220K | TWL6030_USB_ID_CTRL_GND_DRV, ID_READ_DELAY);

	/* Restore old values */
	twl_i2c_write_u8(TWL_MODULE_USB, ~(1<<2), REG_USB_ID_CTRL_CLR);
	twl_i2c_write_u8(TWL_MODULE_USB, save_idctrl, REG_USB_ID_CTRL_SET);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, save_backup, REG_OTG_BACKUP);
	mutex_unlock(&di->lock);

	n = sprintf(buf, "100K/PD val %d 100K val %d 220K/PD val %d 220K val %d\n",
					pu100pd_val, pu100_val, pu220pd_val, pu220_val);

	n += sprintf(buf+n, "5U val %d, 16U val %d, 5U w/ PD val %d, 16U w/ PD val %d. 16U+220K w/ pd %d\n",
					src5u_val, src16u_val, src5upd_val, src16upd_val,  src16upu220pd_val);
	n += sprintf(buf+n, "sts_c 0x%x, int_src 0x%x trim15 0x%x time16 0x%x\n", sts_c, int_src, trim15, trim16);
	n += sprintf(buf+n, "RAW:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n\n",
					pu100pd_val, pu100_val, pu220pd_val, pu220_val,
					src5u_val, src16u_val, src5upd_val, src16upd_val,
					src16upu220pd_val);
	return n;
}

static ssize_t show_headset(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct usb_mux_device_info *di = dev_get_drvdata(dev);
	int headset, ret;

	mutex_lock(&di->lock);
	headset = get_jack_device(di);
	mutex_unlock(&di->lock);
	switch (headset) {
		case HS_JACK_UNKNOWN:
			ret = sprintf(buf,  "Unknown\n");
			break;
		case HS_JACK_NONE:
			ret = sprintf(buf,  "None\n");
			break;
		case HS_JACK_MONO:
			ret = sprintf(buf,  "Mono\n");
			break;
		case HS_JACK_STEREO:
			ret = sprintf(buf,  "Stereo\n");
			break;
		default:
			ret = sprintf(buf,  "Impossible\n");
			break;
	}
	return ret;
}

static DEVICE_ATTR(mux_mode, S_IWUGO | S_IRUGO, show_mode, set_mode);
static DEVICE_ATTR(id_monitor, S_IWUGO | S_IRUGO, show_id_monitor,
	set_id_monitor);
static DEVICE_ATTR(id_allvoltages, S_IRUGO, show_id_allvoltages, NULL);
static DEVICE_ATTR(headset, S_IRUGO, show_headset, NULL);

static struct attribute *usb_mux_attributes[] = {
	&dev_attr_mux_mode.attr,
	&dev_attr_id_monitor.attr,
	&dev_attr_id_allvoltages.attr,
	&dev_attr_headset.attr,
	NULL,
};


static const struct attribute_group usb_mux_attr_group = {
	.attrs = usb_mux_attributes,
};

static int usb_mux_probe(struct platform_device *pdev)
{
	int ret;
	int temp;
	struct usb_mux_platform_data *pdata = pdev->dev.platform_data;
	struct usb_mux_device_info *di = NULL;

	if (!pdata) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "platform_data is NULL!!\n");
		goto error;
	}

	di = kzalloc(sizeof(struct usb_mux_device_info), GFP_KERNEL);
	if (!di) {
		ret = -ENOMEM;
		goto error;
	}

	di->dev = &pdev->dev;
	di->gpio_cb0 = pdata->gpio_cb0;
	di->gpio_cb1 = pdata->gpio_cb1;

	di->tty_priority = tty_priority;
	di->factorycable_bootmode = notle_factorycable_bootmode;
	ret = gpio_request_one(pdata->gpio_cb0, pdata->gpio_cb0_flags, pdata->gpio_cb0_label);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get usb_mux_cb0 gpio_%d\n", pdata->gpio_cb0);
		goto error;
	}

	ret = gpio_request_one(pdata->gpio_cb1, pdata->gpio_cb1_flags, pdata->gpio_cb1_label);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get usb_mux_cb1 gpio_%d\n", pdata->gpio_cb1);
		goto error;
	}

	/* read the current mux config (should match the gpio flags initial drive) */
	temp  = (!!gpio_get_value(di->gpio_cb0)) << 0;
	temp |= (!!gpio_get_value(di->gpio_cb1)) << 1;
	di->cur_mode = di->req_mode = temp;

	dev_warn(&pdev->dev, "Initial MUX mode is %s\n", str_for_mode(di->cur_mode));

	platform_set_drvdata(pdev, di);

	mutex_init(&di->lock);

	di->wq = create_freezable_workqueue(dev_name(&pdev->dev));
	INIT_WORK(&di->work, usb_mux_work);
	INIT_DELAYED_WORK(&di->id_work, usb_id_work);

	di->nb.notifier_call = usb_mux_usb_notifier_call;
	di->otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (di->otg) {
		ret = twl6030_usb_register_notifier(&di->nb);
		if (ret) {
			dev_err(&pdev->dev, "otg register notifier failed %d\n", ret);
                        goto error;
		}
	} else {
		dev_err(&pdev->dev, "otg_get_transceiver failed %d\n", ret);
		goto error;
	}

	/* use switch-class based headset reporting */
	di->sdev.name = "h2w";
	ret = switch_dev_register(&di->sdev);
	if (ret) {
		dev_err(&pdev->dev, "error registering switch device %d\n", ret);
		goto error;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &usb_mux_attr_group);
	if (ret)
		dev_err(&pdev->dev, "could not create sysfs files\n");

	usb_mux_di = di;
	return 0;

error:
	if (di->otg)
		usb_put_phy(di->otg);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	dev_warn(&pdev->dev, "exiting probe with error: ret=%d\n", ret);

	return ret;
}

static int usb_mux_remove(struct platform_device *pdev)
{
	struct usb_mux_device_info *di = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &usb_mux_attr_group);

	switch_dev_unregister(&di->sdev);
	twl6030_usb_unregister_notifier(&di->nb);
	usb_put_phy(di->otg);

	cancel_delayed_work_sync(&di->id_work);
	cancel_work_sync(&di->work);
	destroy_workqueue(di->wq);

	gpio_free(di->gpio_cb0);
	gpio_free(di->gpio_cb1);

	platform_set_drvdata(pdev, NULL);

	usb_mux_di = NULL;
	kfree(di);

	return 0;
}

// Early initialization; return 0 on failure, 1 on success
static int __init notle_check_factorycable(char *str)
{
	if (!str)
		return 0;
	if (!strcmp(str,"factorycable"))
		notle_factorycable_bootmode = true;
	return 1;
}

static int __init usb_mux_tty_priority(char *str)
{
	if (!str)
		return 0;
	if (!strcmp(str,"true"))
		tty_priority = true;
	return 1;
}

__setup("androidboot.mode=", notle_check_factorycable);
__setup("usb_mux.tty_priority=", usb_mux_tty_priority);

static struct platform_driver usb_mux_driver = {
	.driver = {
		.name	= "usb_mux",
	},
	.probe		= usb_mux_probe,
	.remove		= usb_mux_remove,
};

static int __init usb_mux_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&usb_mux_driver);
	if (ret) {
		pr_err("Failed to register USB MUX driver\n");
		goto error;
	}

error:
	return ret;
}

static void __exit usb_mux_driver_exit(void)
{
	platform_driver_unregister(&usb_mux_driver);
}

module_init(usb_mux_driver_init);
module_exit(usb_mux_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Corey Tabaka <eieio@google.com>");
