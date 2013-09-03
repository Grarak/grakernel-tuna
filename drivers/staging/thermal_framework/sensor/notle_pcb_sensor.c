/*
 * PCB temperature sensor driver file
 *
 *  Copyright (C) 2012 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Patterened after TI's pcb_temp_sensor driver.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/thermal_framework.h>
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/types.h>

#include <plat/common.h>
#include <linux/i2c/twl6030-gpadc.h>

#define PCB_REPORT_DELAY_MS	1000

/*
 * pcb_temp_sensor structure
 * @pdev - Platform device pointer
 * @dev - device pointer
 */
struct pcb_temp_sensor {
	struct platform_device *pdev;
	struct device *dev;
};
struct pcb_temp_sensor *temp_sensor;
struct pcb_sens notle_pcb_sensor;

struct delayed_work turn_off_turbo_sprint_mode_work;
static bool turbo_sprint = false;
static int debug = 0;
struct timespec turbo_sprint_mode_starttime;
static long long turbo_sprint_mode_duration_requested_nS = 0;
struct mutex turbo_sprint_mode_mutex;

#define TURBO_SPRINT_MODE_MAX_DURATION_SEC 10
#define TURBO_SPRINT_CHECK_PERIOD_MSEC 1000
#define TWL6030_ADC_START_VALUE 0
#define TWL6030_ADC_END_VALUE   1536
#define TWL6030_GPADC_CHANNEL	 6

static int adc_to_temp_conversion(int adc_val)
{
	int p[6];
	int i = 0;
	// Use fifth degree polynomial fitted to measured device data
	// -7.891731528e-13 x5 + 4.956437005e-10 x4 - 3.400326093e-8 x3 - 1.440793512e-5 x2 - 8.099170543e-2 x + 55.54894978
	int s = 0;

	const int adc_polynomial_bias = 555;
	const struct {int v, s; } c[6] = {
		// scale increases by 1<<11 at each step
		{(int)( 5.554894978e+1*1000*(1LL<<0)),8},
		{(int)(-8.099170543e-2*1000*(1LL<<3)),0},
		{(int)(-1.440793512e-5*1000*(1LL<<14)),0},
		{(int)(-3.400326093e-8*1000*(1LL<<25)),0},
		{(int)(+4.956437005e-10*1000*(1LL<<36)),0},
		{(int)(-7.891731528e-13*1000*(1LL<<47)),0}
	};

	// clamp adc_val to 11 bits unsigned
	if (adc_val < TWL6030_ADC_START_VALUE)
		adc_val = TWL6030_ADC_START_VALUE;
	else if (adc_val > TWL6030_ADC_END_VALUE)
		adc_val = TWL6030_ADC_END_VALUE;

	adc_val-=adc_polynomial_bias;
	p[0] = 1<<17;
	for (i = 1; i < 6; i++)
		p[i] = (adc_val*p[i-1])>>11;
	for (i = 0; i < 6; i++) {
		const int r = (1*(i/2));
		int t = ((c[i].v>>r)*(p[i]>>c[i].s));
		s += t>>(5-r);
	}
	s >>= 4;
	return s;
}

#if 0
// residual test code to compare accuracy of integer code to double
// implementation
double
d_adc_to_temp_conversion(int adc_val) {
	int i;
	double x[6];
	double c[6] = {
	    + 55.54894978,
	    - 8.099170543e-2,
	    - 1.440793512e-5,
	    - 3.400326093e-8,
	    + 4.956437005e-10,
	    -7.891731528e-13
	};
	    adc_val -= 555;
	x[0] = 1.0;
	double s = c[0];
	double t[6];
	for (i = 1; i < 6; i++) {
		x[i] = x[i-1]*adc_val;
		t[i] = c[i] * x[i];
		s += c[i] * x[i];
	}
	return s;
}

int main(int argc, char **argv)
{
	int adc_val = atoi(argv[1]);
	double s = d_adc_to_temp_conversion(adc_val);
	printf("%d -> %4.3f\n%d -> %d\n",
		   adc_val,
		   s);
	const int v = adc_to_temp_conversion(adc_val);
	printf("%d -> %d\n",
		   adc_val,
		   v);
	int j;
	for (j = TWL6030_ADC_START_VALUE; j < TWL6030_ADC_END_VALUE; j++) {
		s = d_adc_to_temp_conversion(j);
		int polyout = adc_to_temp_conversion(j);
		fprintf(stderr, "%g %d\n", s*1000, polyout);
		if (fabs(s-polyout/1000.) > 0.4) {
			printf("%d\n", j);
			//exit(9);
		}
	}
	printf("done\n");
	return 0;
}
#endif

static int pcb_read_current_thermistor(void)
{
	int temp = 0;
	struct twl6030_gpadc_request req;
	int ret;

	req.channels = (1 << TWL6030_GPADC_CHANNEL);
	req.method = TWL6030_GPADC_SW2;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);
	if (ret < 0) {
		pr_err("%s:TWL6030_GPADC conversion is invalid %d\n",
			   __func__, ret);
		return -EINVAL;
	}
	temp = req.rbuf[TWL6030_GPADC_CHANNEL];

	return temp;
}

static int pcb_read_current_temp(void)
{
	int temp = pcb_read_current_thermistor();
	return adc_to_temp_conversion(temp);
}

static void turn_off_turbo_sprint_mode_delay_work_fn(struct work_struct *work)
{
	struct timespec current_time;
	long long stop, start, elapsed_time_nS;

	if (!turbo_sprint) return;

	mutex_lock(&turbo_sprint_mode_mutex);

	// Compute the Elapsed Time
	getrawmonotonic(&current_time);
	stop  = ((long long) current_time.tv_sec) * 1000000000LL +
	        ((long long) current_time.tv_nsec);
	start = ((long long) turbo_sprint_mode_starttime.tv_sec) * 1000000000LL +
	        ((long long) turbo_sprint_mode_starttime.tv_nsec);
	elapsed_time_nS = stop - start;

	// Is Turbo Sprint Expired?
	if (elapsed_time_nS > turbo_sprint_mode_duration_requested_nS ) {
	    printk("Turbo Sprint Off elapsed Time %lld nS\n", elapsed_time_nS);
	    turbo_sprint = false;
	}
	else {
	    schedule_delayed_work(&turn_off_turbo_sprint_mode_work,
	            msecs_to_jiffies(TURBO_SPRINT_CHECK_PERIOD_MSEC));
	}
	mutex_unlock(&turbo_sprint_mode_mutex);
}

/*
 * sysfs hook functions
 */
static int pcb_temp_sensor_read_thermistor(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	int temp = pcb_read_current_thermistor();

	return sprintf(buf, "%d\n", temp);
}

static int pcb_temp_sensor_read_temp(struct device *dev,
					  struct device_attribute *devattr,
					  char *buf)
{
	int temp = pcb_read_current_temp();

	return sprintf(buf, "%d\n", temp);
}

static int pcb_temp_get_turbo_sprint_mode(struct device *dev,
	                  struct device_attribute *devattr,
	                  char *buf)
{
	return sprintf(buf, "%d\n", turbo_sprint);
}

static bool pcb_temp_get_turbo_sprint(void) {
	return turbo_sprint;
}

static ssize_t pcb_temp_set_turbo_sprint_mode(struct device *dev,
	                  struct device_attribute *devattr,
	                  const char *buf, size_t len)
{
	int temp     = pcb_read_current_temp();
	int duration = TURBO_SPRINT_MODE_MAX_DURATION_SEC;

	if ((buf != NULL) && (len >= 1)) {
	    sscanf(buf, "%d",&duration);
	}

	printk("Turbo Sprint REQUESTED for <%d> seconds [TEMP: %d]\n", duration, temp);

	if (duration >= 0) {
	    mutex_lock(&turbo_sprint_mode_mutex);
	    if (duration > TURBO_SPRINT_MODE_MAX_DURATION_SEC) {
	        duration = TURBO_SPRINT_MODE_MAX_DURATION_SEC;
	    }

	    turbo_sprint_mode_duration_requested_nS = duration * 1000000000LL;
	    // Update Start Time
	    getrawmonotonic(&turbo_sprint_mode_starttime);

	    if (!turbo_sprint) {
	        schedule_delayed_work(&turn_off_turbo_sprint_mode_work,
	                msecs_to_jiffies(TURBO_SPRINT_CHECK_PERIOD_MSEC));
	        turbo_sprint = true;
	    }
	    mutex_unlock(&turbo_sprint_mode_mutex);
	}
	return len;
}

static ssize_t pcb_temp_set_debug_mode(struct device *dev,
	                  struct device_attribute *devattr,
	                  const char *buf, size_t len)
{
	if ((buf != NULL) && (len >= 1)) {
	    sscanf(buf, "%d",&debug);
	}
	return len;
}

static int pcb_temp_get_debug_mode(struct device *dev,
	                  struct device_attribute *devattr,
	                  char *buf)
{
	return sprintf(buf, "%d\n", debug);
}

static int pcb_temp_get_debug(void) {
	return debug;
}

static DEVICE_ATTR(thermistor, S_IRUGO, pcb_temp_sensor_read_thermistor,
	               NULL);
static DEVICE_ATTR(temperature, S_IRUGO, pcb_temp_sensor_read_temp,
			  NULL);
static DEVICE_ATTR(turbo_sprint, S_IRWXUGO, pcb_temp_get_turbo_sprint_mode,
	    pcb_temp_set_turbo_sprint_mode);
static DEVICE_ATTR(debug, S_IRWXUGO, pcb_temp_get_debug_mode,
	    pcb_temp_set_debug_mode);

static struct attribute *pcb_temp_sensor_attributes[] = {
	&dev_attr_thermistor.attr,
	&dev_attr_temperature.attr,
	&dev_attr_turbo_sprint.attr,
	&dev_attr_debug.attr,
	NULL
};

static const struct attribute_group pcb_temp_sensor_group = {
	.attrs = pcb_temp_sensor_attributes,
};

static int __devinit pcb_temp_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;

	temp_sensor = kzalloc(sizeof(struct pcb_temp_sensor), GFP_KERNEL);
	if (!temp_sensor)
		return -ENOMEM;

	temp_sensor->pdev = pdev;
	temp_sensor->dev = &pdev->dev;

	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);
	platform_set_drvdata(pdev, temp_sensor);

	ret = sysfs_create_group(&pdev->dev.kobj,
				 &pcb_temp_sensor_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs files\n");
		goto sysfs_create_err;
	}

	dev_info(&pdev->dev, "%s\n", "notle_pcb_sensor");

	notle_pcb_sensor.update_temp = pcb_read_current_temp;
	notle_pcb_sensor.turbo_sprint = pcb_temp_get_turbo_sprint;
	notle_pcb_sensor.debug = pcb_temp_get_debug;
	omap4_duty_pcb_register(&notle_pcb_sensor);

	mutex_init(&turbo_sprint_mode_mutex);
	INIT_DELAYED_WORK(&turn_off_turbo_sprint_mode_work,
	        turn_off_turbo_sprint_mode_delay_work_fn);

	return 0;

sysfs_create_err:
	platform_set_drvdata(pdev, NULL);
	kfree(temp_sensor);
	return ret;
}

static int __devexit pcb_temp_sensor_remove(struct platform_device *pdev)
{
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &pcb_temp_sensor_group);
	kobject_uevent(&temp_sensor->dev->kobj, KOBJ_REMOVE);
	platform_set_drvdata(pdev, NULL);
	kfree(temp_sensor);

	return 0;
}

static int pcb_temp_sensor_runtime_suspend(struct device *dev)
{
	return 0;
}

static int pcb_temp_sensor_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops pcb_temp_sensor_dev_pm_ops = {
	.runtime_suspend = pcb_temp_sensor_runtime_suspend,
	.runtime_resume = pcb_temp_sensor_runtime_resume,
};

static struct platform_driver pcb_temp_sensor_driver = {
	.probe = pcb_temp_sensor_probe,
	.remove = pcb_temp_sensor_remove,
	.driver = {
		.name = "notle_pcb_sensor",
		.pm = &pcb_temp_sensor_dev_pm_ops,
	},
};

int __init pcb_temp_sensor_init(void)
{
	return platform_driver_register(&pcb_temp_sensor_driver);
}

static void __exit pcb_temp_sensor_exit(void)
{
	platform_driver_unregister(&pcb_temp_sensor_driver);
}

module_init(pcb_temp_sensor_init);
module_exit(pcb_temp_sensor_exit);

MODULE_DESCRIPTION("Notle PCB Temperature Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Google Inc");
