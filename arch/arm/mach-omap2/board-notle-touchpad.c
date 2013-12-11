/*
 * Notle touchpad board file.
 *
 * Handles all touchpad related devices.
 */
#include "board-notle.h"

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* Synaptics touchpad driver. */
#if defined(CONFIG_RMI4_BUS)
#include <linux/rmi.h>
#endif  /* CONFIG_RMI4_BUS */

/* TI touchpad driver */
#if defined(CONFIG_INPUT_TOUCHPAD_TSC3060)
#include <linux/i2c/tsc3060.h>
#endif  /* TI touchpad driver */

/* I2C bus number hardware for touchpad access */
#define NOTLE_TOUCHPAD_I2C_BUS 3

/* Meta data for touchpad driver */
static struct notle_touchpad_data_s {
	struct i2c_client *client;
} ntpad_data;

#ifdef CONFIG_RMI4_BUS
static int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	int retval;
	struct rmi_device_platform_data *data
		= (struct rmi_device_platform_data *)gpio_data;
	int gpio_touchpad_int = data->attn_gpio;

	if (configure) {
		/* Allocate the gpio */
		retval = gpio_request_one(gpio_touchpad_int, GPIOF_IN,
		                          "touchpad_int_n");
		if (retval) {
			pr_err("_%s Failed to get touchpad_int_n gpio %d\n",
			       __func__, gpio_touchpad_int);
			retval = -EAGAIN;
		}
		/* Allow this interrupt to wake the system */
		retval = irq_set_irq_wake(gpio_to_irq(gpio_touchpad_int), 1);
		if (retval) {
			pr_err("%s Unable to set irq to wake device\n",
			       __func__);
			gpio_free(gpio_touchpad_int);
			return -EAGAIN;
		}

		/* Enable the interrupt */
		enable_irq(gpio_to_irq(gpio_touchpad_int));
		pr_info("%s Setup touchpad gpio %d\n", __func__,
		        gpio_touchpad_int);
	} else {
		/* Disable the interrupt */
		disable_irq(gpio_to_irq(gpio_touchpad_int));

		/* Restrict this interrupt to wake the system */
		retval = irq_set_irq_wake(gpio_to_irq(gpio_touchpad_int), 0);
		if (retval) {
			pr_err("%s Unable to unset irq to wake device\n",
			       __func__);
		}
		/* Deallocate the gpio */
		gpio_free(gpio_touchpad_int);
		pr_info("%s Free touchpad gpio %d\n", __func__,
		        gpio_touchpad_int);
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
	.gpio_data = &synaptics_platformdata,
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

static struct i2c_board_info notle_i2c_touchpad_syn = {
	.type = "rmi_i2c",
	.platform_data = &synaptics_platformdata,
};

/* List of all possible synaptics touchpad i2c addresses */
static const unsigned short normal_i2c_syn[] = { 0x20, I2C_CLIENT_END };

#endif  /* CONFIG_RMI4_BUS */


#ifdef CONFIG_INPUT_TOUCHPAD_TSC3060
#define TSC3060_ADDR_FUNC       0x48
#define TSC3060_ADDR_BSL        0x70

/* Return the irq for the specified gpio */
int ts_query_irq(void)
{
	return OMAP_GPIO_IRQ(notle_get_gpio(GPIO_TOUCHPAD_INT_N_INDEX));
}

/* Initialize the touchpad interrupt */
void ts_init(void)
{
	int r;
	/* Get the gpio assigned for touchpad */
	int gpio_touchpad = notle_get_gpio(GPIO_TOUCHPAD_INT_N_INDEX);
	pr_info("%s Initializing TSC3060 interrupt irq:%d gpio:%d\n",
	        __func__, OMAP_GPIO_IRQ(gpio_touchpad), gpio_touchpad);
	omap_mux_init_gpio(gpio_touchpad, OMAP_PIN_INPUT
	                   | OMAP_PIN_OFF_WAKEUPENABLE);
	r = gpio_request(gpio_touchpad, "TSC3060 Interrupt\n");
	if (r < 0) {
		pr_err("%s Error - no GPIO available\n", __func__);
	}
	gpio_direction_input(gpio_touchpad);
}

struct tsc3060_platform_data tsc_info =
{
	.init_platform_hw       = ts_init,
	.query_irq              = ts_query_irq,
};

static struct i2c_board_info notle_i2c_touchpad_ti = {
	.type = "tsc3060",
	.platform_data  = &tsc_info,
};

/* List of all possible TI touchpad addresses */
static const unsigned short normal_i2c_ti[] = { TSC3060_ADDR_FUNC,
	TSC3060_ADDR_BSL, I2C_CLIENT_END };

#endif  /* CONFIG_INPUT_TOUCHPAD_TSC3060 */


/* Driver i2c access to read real touchpad hardware devices */
static int read_i2c(const struct i2c_client *client, u8 addr, u8 *rx_buf,
                    int len) {
	u8 buf[1] = {addr};
	int rc;
	rc = i2c_master_send(client, buf, sizeof(buf));
	if (rc != sizeof(buf)) {
		rc = (rc < 0) ? rc : -EIO;
		goto exit;
	}

	rc = i2c_master_recv(client, rx_buf, len);
	if (rc < 0)
		goto exit;

	return 0;
exit:
	return rc;
}

static int __devinit touchpad_notle_probe(struct platform_device *pdev)
{
	struct i2c_adapter *i2c_adap = i2c_get_adapter(NOTLE_TOUCHPAD_I2C_BUS);
	int rc = 0;
	u8 buf[2];
	int gpio_touchpad;

	pr_info("%s Initializing touchpad drivers\n", __func__);
	memset(&ntpad_data, 0, sizeof(ntpad_data));

	/* Get the gpio assigned for touchpad */
	gpio_touchpad = notle_get_gpio(GPIO_TOUCHPAD_INT_N_INDEX);

#ifdef CONFIG_RMI4_BUS
	/* Try to instantiate a synaptics device */
	ntpad_data.client = i2c_new_probed_device(i2c_adap,
	                                          &notle_i2c_touchpad_syn,
	                                          normal_i2c_syn, NULL);

	// Check if synaptics is there
	if (ntpad_data.client) {
		pr_info("%s Detected synaptics touchpad\n", __func__);
		if (ntpad_data.client->addr == 0x20) {
			/* Quick double check reading known location */
			rc = read_i2c(ntpad_data.client, 0x0, buf, 1);
			if (rc < 0) {
				pr_err("%s Failed i2c access %d\n",
				       __func__, rc);
				goto exit;
			}

			/* Set the proper gpio pin for interrupt */
			synaptics_platformdata.attn_gpio = gpio_touchpad;

			rc = 0;
			goto exit;
		} else {
			pr_err("%s Unknown synaptics touchpad addr:0x%02x\n",
			       __func__, ntpad_data.client->addr);
		}
	}
#endif  /* CONFIG_RMI4_BUS */

#ifdef CONFIG_INPUT_TOUCHPAD_TSC3060
	/* Set the proper gpio pin for interrupt */
	notle_i2c_touchpad_ti.irq = gpio_touchpad;

	ntpad_data.client = i2c_new_probed_device(i2c_adap,
	                                          &notle_i2c_touchpad_ti,
	                                          normal_i2c_ti, NULL);

	// Check if TI touchpad exists
	if (ntpad_data.client) {
		pr_info("%s Detected TI touchpad addr:0x%02x\n", __func__,
		        ntpad_data.client->addr);
		if (ntpad_data.client->addr == TSC3060_ADDR_FUNC) {
			/* Functional mode */
			rc  = read_i2c(ntpad_data.client, 0xfa, buf, 1);
			if (rc < 0) {
				pr_err("%s Failed i2c access %d\n",
				       __func__, rc);
				goto exit;
			}
			rc = 0;
			goto exit;

		} else if (ntpad_data.client->addr == TSC3060_ADDR_BSL) {
			/* Boot Strap Loader mode */
			rc  = read_i2c(ntpad_data.client, 0xfa, buf, 1);
			if (rc < 0) {
				pr_err("%s Failed i2c access %d\n",
				       __func__, rc);
				goto exit;
			}
			rc = 0;
			goto exit;
		} else {
			pr_err("%s Unknown TI touchpad addr:0x%02x\n", __func__,
			       ntpad_data.client->addr);
		}
	}
#endif  /* CONFIG_INPUT_TOUCHPAD_TSC3060 */
	/* There is no device present that is suitable for us  */
	rc = -ENODEV;

 exit:
	i2c_put_adapter(i2c_adap);

	return rc;
}

static int __devexit touchpad_notle_remove(struct platform_device *pdev)
{
	pr_debug("%s Removing touchpad meta-driver\n", __func__);
	if (ntpad_data.client)
		i2c_unregister_device(ntpad_data.client);
	return 0;
}

static struct platform_driver touchpad_notle_driver = {
	.driver = {
		.name = "touchpad_notle",
		.owner	= THIS_MODULE,
	},
	.probe = touchpad_notle_probe,
	.remove = touchpad_notle_remove,
};

static struct platform_device touchpad_notle_device = {
	.name           = "touchpad_notle",
};

static int __init touchpad_notle_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&touchpad_notle_driver);
	if (ret) {
		pr_err("%s Failed to register notle touchpad meta driver\n",
		       __func__);
		goto error;
	}

	ret = platform_device_register(&touchpad_notle_device);
	if (ret) {
		pr_err("%s Failed to register notle touchpad meta device\n",
		       __func__);
		goto error;
	}
	pr_debug("%s Registered touchpad meta driver and device\n", __func__);
error:
	return ret;
}

static void __exit touchpad_notle_driver_exit(void)
{
	platform_device_unregister(&touchpad_notle_device);
	platform_driver_unregister(&touchpad_notle_driver);
	pr_debug("%s Unregistered touchpad meta driver and device\n", __func__);
}

module_init(touchpad_notle_driver_init);
module_exit(touchpad_notle_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Chris Manton <cmanton@google.com>");
MODULE_DESCRIPTION("notle_touchpad meta-driver");

