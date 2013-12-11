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
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/module.h>

#define GPS_ELTON_DRVNAME "gps_elton"

struct gps_elton_data_s {
	struct gps_elton_platform_data_s *platform_data;
	struct platform_device *pdev;
	struct device *dev;
	struct class *class;
};


/* Global structure to hold gps data.  There can only be on driver instance. */
static struct gps_elton_data_s *gps_elton_data = NULL;

static ssize_t gps_elton_onoff_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct gps_elton_data_s *gps_elton_data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
		gpio_get_value(gps_elton_data->platform_data->gpio_on_off));
}

static ssize_t gps_elton_onoff_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int onoff = -1;
	sscanf(buf, "%d", &onoff);

	/* Validate parameters. */
	if (onoff != 0 && onoff != 1) {
		return -EINVAL;
	}

	gpio_set_value(gps_elton_data->platform_data->gpio_on_off, onoff);

	dev_info(dev, "Setting on_off: %d\n", onoff);

	return count;
}

static ssize_t gps_elton_reset_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct gps_elton_data_s *gps_elton_data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
		gpio_get_value(gps_elton_data->platform_data->gpio_reset));
}

/* User manually specifying suspend or resume mode of GPS chip. */
static ssize_t gps_elton_reset_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int reset = -1;
	sscanf(buf, "%d", &reset);

	/* Validate parameters. */
	if (reset != 0 && reset != 1) {
		return -EINVAL;
	}

	gpio_set_value(gps_elton_data->platform_data->gpio_reset, reset);

	dev_info(dev, "Setting reset: %d\n", reset);

	return count;
}

static ssize_t gps_elton_awake_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct gps_elton_data_s *gps_elton_data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
		gpio_get_value(gps_elton_data->platform_data->gpio_awake));
}

static struct device_attribute common_attrs[] = {
	__ATTR(onoff, 0666, gps_elton_onoff_show, gps_elton_onoff_store),
	__ATTR(reset, 0666, gps_elton_reset_show, gps_elton_reset_store),
	__ATTR(awake, 0666, gps_elton_awake_show, NULL),
};

static int gps_elton_suspend(struct platform_device *pdev, pm_message_t state) {
	dev_info(&pdev->dev, "Suspending\n");
	if (gpio_get_value(gps_elton_data->platform_data->gpio_awake) == 1) {
		dev_info(&pdev->dev, "GPS receiver seems active!\n");
	}
	return 0;
}

// Currently don't wake the GPS chip after a suspend cycle.
static int gps_elton_resume(struct platform_device *pdev) {
	dev_info(&pdev->dev, "Resuming\n");
	return 0;
}

static int gps_elton_probe(struct platform_device *pdev) {
	int rc = 0;
	int attr_count, count;

	/* There can only be one elton driver instance. */
	if (gps_elton_data) {
		return -EINVAL;
	}

	gps_elton_data = kzalloc(sizeof(struct gps_elton_data_s), GFP_KERNEL);
	if (!gps_elton_data) {
		return -ENOMEM;
	}

	gps_elton_data->pdev = pdev;

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

	/* Set up common sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(common_attrs); attr_count++) {
		rc = device_create_file(gps_elton_data->dev, &common_attrs[attr_count]);
		if (rc < 0) {
			dev_err(gps_elton_data->dev,
				"%s: Failed to create sysfs file for %s.\n",
				__func__, common_attrs[attr_count].attr.name);
			goto error_exit;
		}
	}

	dev_info(gps_elton_data->dev,
		"Intial GPIO states: onoff %d reset %d awake %d\n",
		gpio_get_value(gps_elton_data->platform_data->gpio_on_off),
		gpio_get_value(gps_elton_data->platform_data->gpio_reset),
		gpio_get_value(gps_elton_data->platform_data->gpio_awake));


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
        int attr_count;

	for (attr_count = 0; attr_count < ARRAY_SIZE(common_attrs); attr_count++) {
		device_remove_file(gps_elton_data->dev, &common_attrs[attr_count]);
	}

	device_destroy(gps_elton_data->class, gps_elton_data->dev->devt);
	class_destroy(gps_elton_data->class);
	kzfree(gps_elton_data);

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
