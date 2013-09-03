/*
 * Copyright (c) 2011, 2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kconfig.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include "rmi_driver.h"

struct rmi_i2c_data {
	struct mutex page_mutex;
	int page;
	struct rmi_phys_device *phys;

	bool comms_debug;
#ifdef	CONFIG_RMI4_DEBUG
	struct dentry *debugfs_comms;
#endif
};

#ifdef CONFIG_RMI4_DEBUG

#include <linux/debugfs.h>
#include <linux/uaccess.h>

static int setup_debugfs(struct rmi_device *rmi_dev, struct rmi_i2c_data *data);
static void teardown_debugfs(struct rmi_i2c_data *data);

struct i2c_debugfs_data {
	bool done;
	struct rmi_i2c_data *i2c_data;
};

static int debug_open(struct inode *inodep, struct file *filp)
{
	struct i2c_debugfs_data *data;

	data = kzalloc(sizeof(struct i2c_debugfs_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->i2c_data = inodep->i_private;
	filp->private_data = data;
	return 0;
}

static int debug_release(struct inode *inodep, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static ssize_t comms_debug_read(struct file *filp, char __user *buffer,
		size_t size, loff_t *offset) {
	int retval;
	char local_buf[size];
	struct i2c_debugfs_data *dfs = filp->private_data;
	struct rmi_i2c_data *data = dfs->i2c_data;

	if (dfs->done)
		return 0;

	dfs->done = 1;

	retval = snprintf(local_buf, PAGE_SIZE, "%u\n", data->comms_debug);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static ssize_t comms_debug_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset) {
	int retval;
	char local_buf[size];
	unsigned int new_value;
	struct i2c_debugfs_data *dfs = filp->private_data;
	struct rmi_i2c_data *data = dfs->i2c_data;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval)
		return -EFAULT;

	retval = sscanf(local_buf, "%u", &new_value);
	if (retval != 1 || new_value > 1)
		return -EINVAL;

	data->comms_debug = new_value;

	return size;
}


static const struct file_operations comms_debug_fops = {
	.owner = THIS_MODULE,
	.open = debug_open,
	.release = debug_release,
	.read = comms_debug_read,
	.write = comms_debug_write,
};

static int setup_debugfs(struct rmi_device *rmi_dev, struct rmi_i2c_data *data)
{
	if (!rmi_dev->debugfs_root)
		return -ENODEV;

	data->debugfs_comms = debugfs_create_file("comms_debug", RMI_RW_ATTR,
			rmi_dev->debugfs_root, data, &comms_debug_fops);
	if (!data->debugfs_comms || IS_ERR(data->debugfs_comms)) {
		dev_warn(&rmi_dev->dev, "Failed to create debugfs comms_debug.\n");
		data->debugfs_comms = NULL;
	}

	return 0;
}

static void teardown_debugfs(struct rmi_i2c_data *data)
{
	if (data->debugfs_comms)
		debugfs_remove(data->debugfs_comms);
}
#endif

#define COMMS_DEBUG(data) (IS_ENABLED(CONFIG_RMI4_DEBUG) && data->comms_debug)

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_I2C_PAGE(addr) (((addr) >> 8) & 0xff)

static char *phys_proto_name = "i2c";

/*
 * rmi_set_page - Set RMI page
 * @phys: The pointer to the rmi_phys_device struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the physical
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_phys_device *phys, u8 page)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	u8 txbuf[2] = {RMI_PAGE_SELECT_REGISTER, page};
	int retval;

	if (COMMS_DEBUG(data))
		dev_dbg(&client->dev, "writes 3 bytes: %02x %02x\n",
			txbuf[0], txbuf[1]);
	phys->info.tx_count++;
	phys->info.tx_bytes += sizeof(txbuf);
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		phys->info.tx_errs++;
		dev_err(&client->dev,
			"%s: set page failed: %d.", __func__, retval);
		return (retval < 0) ? retval : -EIO;
	}
	data->page = page;
	return 0;
}

static int rmi_i2c_write_block(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			       int len)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	u8 txbuf[len + 1];
	int retval;

	txbuf[0] = addr & 0xff;
	memcpy(txbuf + 1, buf, len);

	mutex_lock(&data->page_mutex);

	if (RMI_I2C_PAGE(addr) != data->page) {
		retval = rmi_set_page(phys, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

	if (COMMS_DEBUG(data)) {
		char debug_buf[len*3 + 1];
		int i;
		int n = 0;
		char *temp = debug_buf;

		for (i = 0; i < len; i++) {
			n = sprintf(temp, " %02x", buf[i]);
			temp += n;
		}
		dev_dbg(&client->dev, "writes %d bytes at %#06x:%s\n",
			len, addr, debug_buf);
	}

	phys->info.tx_count++;
	phys->info.tx_bytes += sizeof(txbuf);
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval < 0)
		phys->info.tx_errs++;
	else
		retval--; /* don't count the address byte */

exit:
	mutex_unlock(&data->page_mutex);
	return retval;
}


static int rmi_i2c_read_block(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			      int len)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	u8 txbuf[1] = {addr & 0xff};
	int retval;

	mutex_lock(&data->page_mutex);

	if (RMI_I2C_PAGE(addr) != data->page) {
		retval = rmi_set_page(phys, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

	if (COMMS_DEBUG(data))
		dev_dbg(&client->dev, "writes 1 bytes: %02x\n", txbuf[0]);

	phys->info.tx_count++;
	phys->info.tx_bytes += sizeof(txbuf);
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		phys->info.tx_errs++;
		retval = (retval < 0) ? retval : -EIO;
		goto exit;
	}

	retval = i2c_master_recv(client, buf, len);

	phys->info.rx_count++;
	phys->info.rx_bytes += len;
	if (retval < 0)
		phys->info.rx_errs++;
	else if (COMMS_DEBUG(data)) {
		char debug_buf[len*3 + 1];
		char *temp = debug_buf;
		int i;
		int n = 0;

		for (i = 0; i < len; i++) {
			n = sprintf(temp, " %02x", buf[i]);
			temp += n;
		}
		dev_dbg(&client->dev, "read %d bytes at %#06x:%s\n",
			len, addr, debug_buf);
	}

exit:
	mutex_unlock(&data->page_mutex);
	return retval;
}

static int __devinit rmi_i2c_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct rmi_phys_device *rmi_phys;
	struct rmi_i2c_data *data;
	struct rmi_device_platform_data *pdata = client->dev.platform_data;
	int retval;

	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}
	dev_info(&client->dev, "Probing %s at %#02x (IRQ %d).\n",
		pdata->sensor_name ? pdata->sensor_name : "-no name-",
		client->addr, pdata->attn_gpio);

	if (pdata->gpio_config) {
		dev_info(&client->dev, "Configuring GPIOs.\n");
		retval = pdata->gpio_config(pdata->gpio_data, true);
		if (retval < 0) {
			dev_err(&client->dev, "Failed to configure GPIOs, code: %d.\n",
				retval);
			return retval;
		}
		dev_info(&client->dev, "Done with GPIO configuration.\n");
	}

	retval = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!retval) {
		dev_err(&client->dev, "i2c_check_functionality error %d.\n",
			retval);
		return retval;
	}

	rmi_phys = kzalloc(sizeof(struct rmi_phys_device), GFP_KERNEL);
	if (!rmi_phys)
		return -ENOMEM;

	data = kzalloc(sizeof(struct rmi_i2c_data), GFP_KERNEL);
	if (!data) {
		retval = -ENOMEM;
		goto err_phys;
	}

	data->phys = rmi_phys;

	rmi_phys->data = data;
	rmi_phys->dev = &client->dev;

	rmi_phys->write_block = rmi_i2c_write_block;
	rmi_phys->read_block = rmi_i2c_read_block;
	rmi_phys->info.proto = phys_proto_name;

	mutex_init(&data->page_mutex);

	/* Setting the page to zero will (a) make sure the PSR is in a
	 * known state, and (b) make sure we can talk to the device.
	 */
	retval = rmi_set_page(rmi_phys, 0);
	if (retval) {
		dev_err(&client->dev, "Failed to set page select to 0.\n");
		goto err_data;
	}

	retval = rmi_register_phys_device(rmi_phys);
	if (retval) {
		dev_err(&client->dev,
			"failed to register physical driver at 0x%.2X.\n",
			client->addr);
		goto err_gpio;
	}
	i2c_set_clientdata(client, rmi_phys);

	if (IS_ENABLED(CONFIG_RMI4_DEBUG))
		retval = setup_debugfs(rmi_phys->rmi_dev, data);

	if (IS_ENABLED(CONFIG_RMI4_DEV)) {
		retval = gpio_export(pdata->attn_gpio, false);
		if (retval) {
			dev_warn(&client->dev,
				"WARNING: Failed to export ATTN gpio!\n");
			retval = 0;
		} else {
			retval = gpio_export_link(&client->dev,
						  "attn", pdata->attn_gpio);
			if (retval) {
				dev_warn(&client->dev,
					"WARNING: Failed to symlink ATTN gpio!\n");
				retval = 0;
			} else {
				dev_info(&client->dev, "Exported ATTN GPIO %d.",
					pdata->attn_gpio);
			}
		}
	}

	dev_info(&client->dev, "registered rmi i2c driver at %#04x.\n",
			client->addr);
	return 0;

err_gpio:
	if (pdata->gpio_config)
		pdata->gpio_config(pdata->gpio_data, false);
err_data:
	kfree(data);
err_phys:
	kfree(rmi_phys);
	return retval;
}

static int __devexit rmi_i2c_remove(struct i2c_client *client)
{
	struct rmi_phys_device *phys = i2c_get_clientdata(client);
	struct rmi_device_platform_data *pd = client->dev.platform_data;

	/* Can I remove this disable_device */
	/*disable_device(phys); */

	if (IS_ENABLED(CONFIG_RMI4_DEBUG))
		teardown_debugfs(phys->data);

	rmi_unregister_phys_device(phys);
	kfree(phys->data);
	kfree(phys);

	if (pd->gpio_config)
		pd->gpio_config(&pd->gpio_data, false);

	return 0;
}

static const struct i2c_device_id rmi_id[] = {
	{ "rmi_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_i2c"
	},
	.id_table	= rmi_id,
	.probe		= rmi_i2c_probe,
	.remove		= __devexit_p(rmi_i2c_remove),
};

static int __init rmi_i2c_init(void)
{
	return i2c_add_driver(&rmi_i2c_driver);
}

static void __exit rmi_i2c_exit(void)
{
	i2c_del_driver(&rmi_i2c_driver);
}

module_init(rmi_i2c_init);
module_exit(rmi_i2c_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI I2C driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
