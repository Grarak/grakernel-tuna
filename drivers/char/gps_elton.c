/* drivers/char/gps_elton.c
 *
 * Copyright (C) 2012 Google, Inc.
 *
 *  Elton CSR SiRFstar GSD4e driver.
 *
 * Driver used primarily for handling suspend/resume system events.
 */

#include <linux/gps_elton.h>

#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/kernel_stat.h>
#include <linux/slab.h>
#include <linux/module.h>

#define GPS_ELTON_DRVNAME "gps_elton"

/* EVT 1.x */
#define OMAP_UART3_NAME "OMAP UART3"

#define GPS_STATE_AWAKE 1
#define GPS_STATE_HIBERNATE 0
#define GPS_STATE_UNKNOWN -1

#define GPIO_PIN_UNCONNECTED -1

#define TIME_BETWEEN_AWAKE_PULSES_IN_MS (2000)

struct gps_elton_data_s {
	struct gps_elton_platform_data_s *platform_data;
	struct platform_device *pdev;
	struct device *dev;
	struct class *class;
	struct early_suspend early_suspend;

	// The GPS chip can't be pulsed to quickly else states will be lost.
	// This holds the valid time we can pulse the chip again.
	unsigned long valid_pulse_in_jiffies;

	/* NOTE(CMM) Only for EVT1.x.
	 * Tristate flag indicating state driver believes device is in.
	 *  1: GPS chip is in awake and in active mode.
	 *  0: GPS chip is asleep and in hibernate mode.
	 * -1: GPS chip is in an unkown state.
	 * EVT2 uses a direct line from the GPS chip to indicate awake.
	 */
	int is_awake;

	// EVT1.x Workqueue check to poll for uart interrupts.
	struct delayed_work work_check;
	// EXT1.x Number of uart interrupts to infer if GPS is awake or not.
	atomic_t uart_int_cnt;
	// EVT1.x Used to read irq statistics to determine if chip is awake.
	int irq_num;
};

/* kernel/timer.c */
extern void msleep(unsigned int msecs);

/* Time to wait between toggling power pin. */
static const int wait_between_power_toggle_in_ms = 50;

/* Only for EVT1.x */
/* Time to wait between running workqueue to check for uart interrupts. */
static const int wait_between_check_uart_ints_in_ms = 5000;

/* NOTE(CMM) Not sure if we are supposed to hibernate during early suspend or not. */
static int hibernate_during_early_suspend = 0;

/* Global structure to hold gps data.  There can only be on driver instance. */
static struct gps_elton_data_s *gps_elton_data = NULL;

// Show the wake state of the GPS chip.
static ssize_t gps_elton_awake_show(struct device *dev,
                                    struct device_attribute *attr, char *buf);

// Set the wake state of the GPS chip.
static ssize_t gps_elton_awake_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count);

static ssize_t gps_elton_int_offset_show(struct device *dev,
                                    struct device_attribute *attr, char *buf);

// Set the wake state of the GPS chip.
static ssize_t gps_elton_int_offset_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count);

static struct device_attribute attrs[] = {
	__ATTR(awake, 0666,
	       gps_elton_awake_show,
	       gps_elton_awake_store),
	__ATTR(int_offset, 0666,
	       gps_elton_int_offset_show,
	       gps_elton_int_offset_store),
};

// Determines if device is EVT 1.x or not.
static int _is_evt_1_x(struct gps_elton_data_s *gps_elton_data) {
	return gps_elton_data->platform_data->gpio_awake == GPIO_PIN_UNCONNECTED;
}

// EVT 1.x only
static int _get_irq_num(const char *irq_name) {
	unsigned int irq;
	struct irq_desc *desc;

	for_each_irq_desc(irq,  desc) {
		if (desc && desc->action && desc->action->name && !strcmp(desc->action->name, irq_name)) {
			return (int)irq;
		}
	}
	return -1;
}

// EVT 1.x only
// Critical failure of this approach is that we can't determine
// transmit versus receive interrupts.
static int _get_interrupt_count(int irq_num) {
	int cpu_id;
	int uart_int_cnt = 0;

	for_each_online_cpu(cpu_id)
		uart_int_cnt += kstat_irqs_cpu(irq_num, cpu_id);

	return uart_int_cnt;
}

// EVT 1.x only
static void gps_elton_work_task(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct gps_elton_data_s *gps_elton_data = container_of(dw, struct gps_elton_data_s, work_check);
	int uart_int_cnt_curr;
	int uart_int_cnt_last;

	if (-1 == gps_elton_data->irq_num) {
		dev_err(&gps_elton_data->pdev->dev,
		        "%s Unable to determine omap uart3 interrupt; aborting check for GPS awake\n",
		        __func__);
		return;
	}

	// If the interrupt count hasn't changed since last time, assume we are off.
	// Need to make sure the polling time is long enough to allow interrupts to change
	// in between.
	uart_int_cnt_curr = _get_interrupt_count(gps_elton_data->irq_num);
	uart_int_cnt_last = atomic_read(&gps_elton_data->uart_int_cnt);
	if (uart_int_cnt_last >= uart_int_cnt_curr) {
		dev_info(&gps_elton_data->pdev->dev, "EVT 1.x Device was determined to be in hibernation last_cnt:%d curr_cnt:%d\n",
		         uart_int_cnt_last, uart_int_cnt_curr);
		gps_elton_data->is_awake = GPS_STATE_HIBERNATE;
	} else {
		dev_info(&gps_elton_data->pdev->dev, "EVT 1.x Device was determined to be awake last_cnt:%d curr_cnt:%d\n",
		         uart_int_cnt_last, uart_int_cnt_curr);
		gps_elton_data->is_awake = GPS_STATE_AWAKE;
		// Update the uart interrupt count for next time.
		atomic_set(&gps_elton_data->uart_int_cnt, uart_int_cnt_curr);
	}

	// Put us back on the work queue.
	schedule_delayed_work(&gps_elton_data->work_check,
	                      msecs_to_jiffies(wait_between_check_uart_ints_in_ms));
}

/* Toggles the power line to the GPS chip to bring the chip either
 * into hibernation or into awake state based upon previous state.
 */
static void _toggle_power(struct gps_elton_data_s * gps_elton_data)
{
	gpio_set_value(gps_elton_data->platform_data->gpio_on_off, 1);
	msleep(wait_between_power_toggle_in_ms);
	gpio_set_value(gps_elton_data->platform_data->gpio_on_off, 0);
}

static ssize_t gps_elton_awake_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct gps_elton_data_s *gps_elton_data = dev_get_drvdata(dev);
	if (_is_evt_1_x(gps_elton_data)) {
		// EVT 1.x
		return snprintf(buf, PAGE_SIZE, "%d\n", gps_elton_data->is_awake);
	} else {
		// EVT 2.0
		return snprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(gps_elton_data->platform_data->gpio_awake));
	}
}

/* User manually specifying suspend or resume mode of GPS chip. */
static ssize_t gps_elton_awake_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int data;
	unsigned long jiffies_current;
	struct gps_elton_data_s *gps_elton_data = dev_get_drvdata(dev);
	sscanf(buf, "%d", &data);

	jiffies_current = jiffies;

	/* Validate parameters. */
	if (data != 0 && data != 1) {
		return -EINVAL;
	}

	if (time_before(jiffies_current, gps_elton_data->valid_pulse_in_jiffies)) {
		printk(KERN_WARNING "%s Unable to oscillate chip between awake and hibernate that fast %d\n",
		       __func__, jiffies_to_msecs(gps_elton_data->valid_pulse_in_jiffies - jiffies_current));
		return -EBUSY;
	}


	if (data == 1) {
		/* We want to take the GPS chip out of hibernation. */
		if (_is_evt_1_x(gps_elton_data)) {
			/* EVT1.x */
			if (gps_elton_data->is_awake != GPS_STATE_HIBERNATE) {
				return -EAGAIN;
			}
			/* NOTE(cmanton) Disable on EVT 1.x as counting indirectly
			 * determining the state of the GPS chip is difficult for
			 * ensuring a proper suspend. */
			dev_warn(dev, "Disabled enabling GPS on EVT 1.x\n");
			return -ENODEV;
		} else {
			/* EVT2.0 */
			if (gpio_get_value(gps_elton_data->platform_data->gpio_awake) == 1) {
				/* Already awake */
				return -EAGAIN;
			}
		}
		_toggle_power(gps_elton_data);
		if (_is_evt_1_x(gps_elton_data)) {
			/* EVT 1.x - Force cache irq count to not equal system irq count. */
			atomic_set(&gps_elton_data->uart_int_cnt, -1);
			/* EVT1.x */
			gps_elton_data->is_awake = GPS_STATE_AWAKE;
		}
		dev_info(dev, "User awoke GPS chip\n");
	} else {
		/* We want to put the GPS chip into hibernation. */
		if (_is_evt_1_x(gps_elton_data)) {
			/* EVT1.x */
			if (gps_elton_data->is_awake != GPS_STATE_AWAKE) {
				return -EAGAIN;
			}

		} else {
			/* EVT2.0 */
			if (gpio_get_value(gps_elton_data->platform_data->gpio_awake) == 0) {
				/* Already hibernating */
				return -EAGAIN;
			}
		}
		_toggle_power(gps_elton_data);
		if (_is_evt_1_x(gps_elton_data)) {
			/* EVT 1.x - Force cache irq count to be equal to system irq count. */
			atomic_set(&gps_elton_data->uart_int_cnt,  _get_interrupt_count(gps_elton_data->irq_num));
			/* EVT1.x */
			gps_elton_data->is_awake = GPS_STATE_HIBERNATE;
		}
		dev_info(dev, "User put GPS chip into hibernation\n");
	}

	gps_elton_data->valid_pulse_in_jiffies = jiffies_current + msecs_to_jiffies(TIME_BETWEEN_AWAKE_PULSES_IN_MS);
	return count;
}

static ssize_t gps_elton_int_offset_show(struct device *dev,
                                         struct device_attribute *attr, char *buf)
{
	struct gps_elton_data_s *gps_elton_data = dev_get_drvdata(dev);
	if (_is_evt_1_x(gps_elton_data)) {
		// EVT 1.x
		return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gps_elton_data->uart_int_cnt));
	} else {
		// EVT 2.0
		return -ENODEV;
	}
}

/* User manually specifying suspend or resume mode of GPS chip. */
static ssize_t gps_elton_int_offset_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
	int data;
	int old_cnt;
	struct gps_elton_data_s *gps_elton_data = dev_get_drvdata(dev);
	sscanf(buf, "%d", &data);

	old_cnt = atomic_read(&gps_elton_data->uart_int_cnt);
	if (_is_evt_1_x(gps_elton_data)) {
		atomic_add(data, &gps_elton_data->uart_int_cnt);
		dev_info(gps_elton_data->dev, "%s Adding interrupt count old:%d new:%d cnt:%d\n",
		         __func__, old_cnt, atomic_read( &gps_elton_data->uart_int_cnt), data);
		return count;
	} else {
		// EVT 2.0
		return -ENODEV;
	}
}

static void gps_elton_early_suspend(struct early_suspend *h)
{
	if (_is_evt_1_x(gps_elton_data)) {
		/* EVT 1.x - Force cache irq count to be equal to system irq count. */
		// The suspend tristate indicating an unknown state.
		if (gps_elton_data->is_awake == GPS_STATE_UNKNOWN) {
			dev_warn(gps_elton_data->dev, "%s In an unknown state\n", __func__);
			return;
		}
		if (gps_elton_data->is_awake == GPS_STATE_HIBERNATE) {
			dev_warn(gps_elton_data->dev, "%s already hibernated\n", __func__);
			return;
		}
	}

	if (hibernate_during_early_suspend) {
		_toggle_power(gps_elton_data);
		gps_elton_data->is_awake = GPS_STATE_HIBERNATE;
		dev_info(gps_elton_data->dev, "%s system hibernated GPS chip\n", __func__);
	}
}

static void gps_elton_late_resume(struct early_suspend *h)
{
	if (_is_evt_1_x(gps_elton_data)) {
		/* EVT 1.x - Force cache irq count to be equal to system irq count. */
		// The suspend tristate indicating an unknown state.
		if (gps_elton_data->is_awake == GPS_STATE_UNKNOWN) {
			dev_warn(gps_elton_data->dev, "%s In an unknown state\n", __func__);
			return;
		}
		if (gps_elton_data->is_awake == GPS_STATE_AWAKE) {
			dev_warn(gps_elton_data->dev, "%s already awake\n", __func__);
			return;
		}
	}
	if (hibernate_during_early_suspend) {
		_toggle_power(gps_elton_data);
		gps_elton_data->is_awake = GPS_STATE_AWAKE;
		dev_info(gps_elton_data->dev, "%s system awoke GPS chip\n", __func__);
	}
}

static int gps_elton_suspend(struct platform_device *pdev, pm_message_t state) {
	int was_awake = 0;
	if (_is_evt_1_x(gps_elton_data)) {
		// EVT 1.x
		if (gps_elton_data->is_awake == GPS_STATE_AWAKE) {
			_toggle_power(gps_elton_data);
			gps_elton_data->is_awake = GPS_STATE_HIBERNATE;
			/* EVT 1.x - Force cache irq count to be equal to system irq count. */
			atomic_set(&gps_elton_data->uart_int_cnt, _get_interrupt_count(gps_elton_data->irq_num));
			was_awake = 1;
		}
	} else {
		// EVT 2.0
		if (gpio_get_value(gps_elton_data->platform_data->gpio_awake) == 1) {
			_toggle_power(gps_elton_data);
			was_awake = 1;
			if (gpio_get_value(gps_elton_data->platform_data->gpio_awake) == 1) {
				dev_err(&pdev->dev, "Unable to toggle GPS chip to sleep");
			}
		}
	}

	dev_info(&pdev->dev, "Suspending - %s\n", (was_awake?"was awake":"was hibernating"));
	return 0;
}

// Currently don't wake the GPS chip after a suspend cycle.
static int gps_elton_resume(struct platform_device *pdev) {
	dev_info(&pdev->dev, "Resuming\n");
	if (_is_evt_1_x(gps_elton_data)) {
		// EVT 1.x
		gps_elton_data->is_awake = GPS_STATE_HIBERNATE;
		/* EVT 1.x - Force cache irq count to be equal to system irq
		 * count.  This because during suspend/resume cycles the UART
		 * code is called which causes exactly 3 interrupts per
		 * suspend/resume cycle.  Setting the irq count equal will
		 * ensure that the GPS chip awake polling routine finds it
		 * still hibernating.  */
		atomic_set(&gps_elton_data->uart_int_cnt, _get_interrupt_count(gps_elton_data->irq_num));
	} else {
		// EVT 2.0
		if (gpio_get_value(gps_elton_data->platform_data->gpio_awake) == 1) {
			dev_err(&pdev->dev, "Found GPS chip awake during suspend cycle");
		}
	}

	return 0;
}

static int gps_elton_probe(struct platform_device *pdev) {
	int rc = 0;
	int attr_count = 0;

	/* There can only be one elton driver instance. */
	if (gps_elton_data) {
		return -EINVAL;
	}

	gps_elton_data = kzalloc(sizeof(struct gps_elton_data_s), GFP_KERNEL);
	if (!gps_elton_data) {
		return -ENOMEM;
	}

	gps_elton_data->pdev = pdev;

	gps_elton_data->valid_pulse_in_jiffies = INITIAL_JIFFIES;

	gps_elton_data->platform_data = pdev->dev.platform_data;
	if (NULL == gps_elton_data->platform_data) {
		dev_err(&pdev->dev, "%s No platform data\n", __func__);
		rc = -EINVAL;
		goto error_exit;
	}

	// Create a sysfs class.
	gps_elton_data->class = class_create(THIS_MODULE, "gps");
	if (IS_ERR(gps_elton_data->class)) {
		rc = PTR_ERR(gps_elton_data->class);
		gps_elton_data->class = NULL;
		goto error_exit;
	}

	gps_elton_data->dev = device_create(gps_elton_data->class, NULL, MKDEV(0, 0),
	                                    gps_elton_data, GPS_ELTON_DRVNAME);
	if (IS_ERR(gps_elton_data->dev)){
		rc = PTR_ERR(gps_elton_data->dev);
		gps_elton_data->dev = NULL;
		goto error_exit;
	}

	/* Setup the early suspend and late resume functionality */
	INIT_LIST_HEAD(&gps_elton_data->early_suspend.link);
	gps_elton_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	gps_elton_data->early_suspend.suspend = gps_elton_early_suspend;
	gps_elton_data->early_suspend.resume = gps_elton_late_resume;
	register_early_suspend(&gps_elton_data->early_suspend);

	/* NOTE(CMM) Only for EVT1.x.
	 * Tristate flag indicating state driver believes device is in.
	 *  1: GPS chip is in awake and in active mode.
	 *  0: GPS chip is asleep and in hibernate mode.
	 * -1: GPS chip is in an unkown state.
	 * EVT2 uses a direct line from the GPS chip to indicate awake.
	 */
	gps_elton_data->is_awake = GPS_STATE_UNKNOWN;
	atomic_set(&gps_elton_data->uart_int_cnt, 0);

	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		rc = device_create_file(gps_elton_data->dev, &attrs[attr_count]);
		if (rc < 0) {
			dev_err(gps_elton_data->dev,
			        "%s: Failed to create sysfs file for %s.\n",
			        __func__, attrs[attr_count].attr.name);
			goto error_exit;
		}
	}

	// EVT1.x
	if (-1 == gps_elton_data->platform_data->gpio_awake) {
		// gps_elton_data->irq_num = 102; _search_for_interrupt
		gps_elton_data->irq_num = _get_irq_num(OMAP_UART3_NAME);
		dev_info(gps_elton_data->dev, "%s Detected omap uart3 interrupt:%d\n",
		         __func__, gps_elton_data->irq_num);
		INIT_DELAYED_WORK(&gps_elton_data->work_check, gps_elton_work_task);
		schedule_delayed_work(&gps_elton_data->work_check,
		                      msecs_to_jiffies(wait_between_check_uart_ints_in_ms));
	}

	// Check on_off line for default values
	if (gpio_get_value(gps_elton_data->platform_data->gpio_on_off) != 0) {
		dev_warn(gps_elton_data->dev, "%s Detected chip on_off value in uexpected state", __func__);
	}

	// Check reset line and take out of reset if needed.
	// NOTE(cmanton) Not sure how long to wait before we guarantee reset
	// is complete on the chip should we need to access again.
	// e.g. checking awake line
	if (gpio_get_value(gps_elton_data->platform_data->gpio_reset) == 0) {
		dev_info(gps_elton_data->dev, "%s Detected chip in reset; taking out of reset", __func__);
		gpio_set_value(gps_elton_data->platform_data->gpio_reset, 1);
	}

	// Keep in hibernate mode.
	// Evt 2.x
	if (-1 != gps_elton_data->platform_data->gpio_awake) {
		if (gpio_get_value(gps_elton_data->platform_data->gpio_awake) == 1) {
			dev_info(gps_elton_data->dev, "%s Detected chip awake; putting into hibernation", __func__);
			_toggle_power(gps_elton_data);
		}
	}

	// TODO(cmanton) Demote this to debug when confidence has been generated.
	dev_info(&pdev->dev, "gpio on_off:%d reset:%d awake:%d\n",
	         gps_elton_data->platform_data->gpio_on_off,
	         gps_elton_data->platform_data->gpio_reset,
	         gps_elton_data->platform_data->gpio_awake);

	dev_info(gps_elton_data->dev, "%s driver initialized successfully\n", __func__);
	return 0;

error_exit:
	if (gps_elton_data->class && gps_elton_data->dev)
		device_destroy(gps_elton_data->class, gps_elton_data->dev->devt);

	if (gps_elton_data->class)
		class_destroy(gps_elton_data->class);

	if (gps_elton_data)
		kfree(gps_elton_data);

	gps_elton_data = NULL;
	dev_info(&pdev->dev, "%s driver failed to initialize\n", __func__);
	return rc;
}

static int gps_elton_remove(struct platform_device *pdev) {
	int attr_count = 0;

	unregister_early_suspend(&gps_elton_data->early_suspend);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		device_remove_file(gps_elton_data->dev, &attrs[attr_count]);
	}

	device_destroy(gps_elton_data->class, gps_elton_data->dev->devt);
	class_destroy(gps_elton_data->class);
	return 0;
}

static struct platform_driver gps_elton_platform_driver = {
	.probe = gps_elton_probe,
	.remove = gps_elton_remove,
	.suspend = gps_elton_suspend,
	.resume = gps_elton_resume,
	.driver = {
		.name = "gps_elton",
		.owner = THIS_MODULE,
	},
};

static int __init gps_elton_init(void)
{
	return platform_driver_register(&gps_elton_platform_driver);
}

static void __exit gps_elton_cleanup(void)
{
	platform_driver_unregister(&gps_elton_platform_driver);
}

module_init(gps_elton_init);
module_exit(gps_elton_cleanup);

MODULE_AUTHOR("Chris Manton <cmanton@google.com>");
MODULE_DESCRIPTION("Elton SiRF4e GPS Driver");
MODULE_LICENSE("GPL");
