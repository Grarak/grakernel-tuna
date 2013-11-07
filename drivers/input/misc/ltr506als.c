/* Lite-On LTR-506ALS Android Driver
 *
 * Copyright (C) 2011 Lite-On Technology Corp (Singapore)
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/i2c/ltr506als.h>

#define DRIVER_VERSION "1.2"
#define PARTID 0x90
#define PARTID_V2 0x91

#define MANUID 0x05

#define I2C_RETRY 5

#define DEVICE_NAME "ltr506als"

static const int poll_rate_in_ms = 1000;

/* For a small number of devices we need to share this GPIO interrupt with
   another device.  This is not necessary should this driver obtains
   an exclusive interrupt.
   */
//#define USE_SHARED_IRQ 1

struct ltr506_data {
	/* Device */
	struct i2c_client *i2c_client;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct workqueue_struct *workqueue;
	struct early_suspend early_suspend;
	struct wake_lock ps_wake_lock;

	uint8_t part_id;

	/* ALS */
	int als_enable_flag;
	int als_suspend_enable_flag;
	int als_opened;
	int als_gain;
	uint16_t als_lowthresh;
	uint16_t als_highthresh;
	uint8_t als_resolution:3;
	uint8_t als_meas_rate:3;
	/* Flag to suspend ALS on suspend or not */
	int disable_als_on_suspend;
	int als_filter_interrupts;
	uint16_t als_last_value;
	int als_from_suspend;

	/* PS */
	int ps_enable_flag;
	int ps_suspend_enable_flag;
	int ps_opened;
	int ps_gain;
	uint16_t ps_lowthresh;
	uint16_t ps_highthresh;
	uint8_t ps_meas_rate:3;
	/* Flag to suspend PS on suspend or not */
	int disable_ps_on_suspend;
	int ps_filter_interrupts;

	/* LED */
	uint8_t led_pulse_freq:3;
	uint8_t led_duty_cyc:2;
	uint8_t led_peak_curr:3;
	uint8_t led_pulse_count;

	/* Interrupt */
	int irq;
	int gpio_int_no;
	int gpio_int_wake_dev;
	int is_suspend;
	int use_polling;

	/* Workarounds */
	int ps_must_be_on_while_als_on;
};

struct ltr506_data *sensor_info;

static int als_enable(struct ltr506_data *ltr506);
static int als_disable(struct ltr506_data *ltr506);
static int _hack_check_valid_als_zero(struct ltr506_data *ltr506);

/* I2C Read */
static int I2C_Read(char *rxData,
                    int length)
{
	int rc;
	int index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		rc = i2c_transfer(sensor_info->i2c_client->adapter, data,
		                  sizeof(data)/sizeof(struct i2c_msg));
		if (rc == 2)
			/* Success */
			break;
		else if (rc == 1)
			pr_err("%s %s Sent only one i2c request\n", __func__,
			       DEVICE_NAME);

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n",__func__);
		return -EIO;
	}
	return 0;
}

/* I2C Write */
static int I2C_Write(char *txData, int length)
{
	int index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 1) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}
	return 0;
}

/* Set register bit
 * Returns: 0    OK
 *          -EIO ERROR
 */
static int _ltr506_set_bit(struct i2c_client *client, uint8_t set, uint8_t cmd,
                           uint8_t data)
{
	char buffer[2];
	uint8_t value;
	int ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X\n", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (set)
		value |= data;
	else
		value &= ~data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X\n", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}

/* Set register field */
static int _ltr506_set_field(struct i2c_client *client, uint8_t mask, uint8_t cmd,
                             uint8_t data)
{
	char buffer[2];
	uint8_t value;
	int ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X\n", __func__, buffer[0]);
		return -EIO;
	}

	value = buffer[0] & ~(mask);
	value |= data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X\n", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}

/* Read ALS ADC Channel 1 Value - clear diode */
static uint32_t read_als_adc_ch1_value(struct ltr506_data *ltr506)
{
	int ret;
	uint32_t value;
	uint8_t buffer[3];

	buffer[0] = LTR506_ALS_DATA_CH1_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, sizeof(buffer));
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s Unable to read als adc"
		        " ch1 values\n", __func__);
		return 0;
	}

	value = ((uint32_t)buffer[2] << 12) | ((uint32_t)buffer[1] << 4) | (uint32_t)(buffer[0] >> 4);
	return value;
}

/* Read ALS ADC Channel 2 Value - IR diode */
static uint32_t read_als_adc_ch2_value(struct ltr506_data *ltr506)
{
	int ret;
	uint32_t value;
	uint8_t buffer[3];

	buffer[0] = LTR506_ALS_DATA_CH2_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, sizeof(buffer));
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s Unable to read als adc"
		        " ch2 values\n", __func__);
		return 0;
	}

	value = ((uint32_t)buffer[2] << 12) | ((uint32_t)buffer[1] << 4) | (uint32_t)(buffer[0] >> 4);
	return value;
}

/* Read ADC ALS filtered value */
static int read_adc_value_als_filtered(struct ltr506_data *ltr506)
{
	int ret;
	int val;
	uint8_t buf[2] = {LTR506_ALS_DATA_0};

	/* read data bytes from data regs */
	ret = I2C_Read(buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s addr:0x%02X\n", __func__,
		        buf[0]);
		return ret;
	}

	val = buf[0] | (buf[1] << 8);

	if (val > LTR506_ALS_MAX_MEASURE_VAL) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS overflow:%d\n",
		        __func__, val);
	}
	return (val & LTR506_ALS_VALID_MEASURE_MASK);
}

/* Read ADC ALS Value */
static int read_adc_value_als(struct ltr506_data *ltr506)
{
	int val = read_adc_value_als_filtered(ltr506);

	/* If we received a zero value, we need to double check the
	   raw diode values to see if it's a true zero, or if
	   over zealous filtering occurred. */
	if (val == 0) {
		val = _hack_check_valid_als_zero(ltr506);
	}

	return (val & LTR506_ALS_VALID_MEASURE_MASK);
}

/* Read ADC PS Value */
static int read_adc_value_ps(struct ltr506_data *ltr506)
{
	int ret;
	int val;
	uint8_t buf[2] = {LTR506_PS_DATA_0};

	/* read data bytes from data regs */
	ret = I2C_Read(buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s addr:0x%02X\n", __func__,
		        buf[0]);
		return ret;
	}

	val = buf[0] | (buf[1] << 8);
	dev_dbg(&ltr506->i2c_client->dev, "%s value = 0x%d\n", __func__, val);

	if (val > LTR506_PS_MAX_MEASURE_VAL) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS overflow:%d\n",
		        __func__, val);
	}
	return (val & LTR506_PS_VALID_MEASURE_MASK);
}

/* Set an upper and lower threshold where interrupts will *not* be generated.
 * Interrupts will not be generated if measurement falls within this rangea
 * e.g.
 *  lt < val < ht
 * but would be generated if the measurement falls outside this range.
 * val < lt || val > ht
 *
 * Untested if the threshold window includes or precludes the edge threshold values.
 *
 */
static int set_als_range(struct ltr506_data *ltr506, uint16_t lt, uint16_t ht)
{
	int ret;
	char buffer[5];

	/* Cache the values here */
	ltr506->als_lowthresh = lt;
	ltr506->als_highthresh = ht;

	buffer[0] = LTR506_ALS_THRES_UP_0;
	buffer[1] = ht & 0xFF;
	buffer[2] = (ht >> 8) & 0xFF;
	buffer[3] = lt & 0xFF;
	buffer[4] = (lt >> 8) & 0xFF;

	ret = I2C_Write(buffer, 5);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev, "%s Set als range:0x%04x"
	                                       " - 0x%04x\n", __func__, lt, ht);
	return ret;
}

/* Set an upper and lower threshold where interrupts will *not* be generated.
 * Interrupts will not be generated if measurement falls within this rangea
 * e.g.
 *  lt < val < ht
 * but would be generated if the measurement falls outside this range.
 * val < lt || val > ht
 *
 * Untested if the threshold window includes or precludes the edge threshold values.
 *
 * Set to "0,0" to disable thresholding interrupts.
 */
static int set_ps_range(struct ltr506_data *ltr506, uint16_t lt, uint16_t ht)
{
	int ret;
	char buffer[5];

	/* Cache the values here */
	ltr506->ps_lowthresh = lt;
	ltr506->ps_highthresh = ht;

	buffer[0] = LTR506_PS_THRES_UP_0;
	buffer[1] = ht & 0xFF;
	buffer[2] = (ht >> 8) & 0x0F;
	buffer[3] = lt & 0xFF;
	buffer[4] = (lt >> 8) & 0x0F;

	ret = I2C_Write(buffer, 5);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev, "%s Set ps range:0x%04x"
	                                       " - 0x%04x\n", __func__, lt, ht);
	return ret;
}

/* Report PS input event */
static void report_ps_input_event(struct ltr506_data *ltr506)
{
	int rc;
	uint16_t adc_value;
	int thresh_hi, thresh_lo, thresh_delta;

	adc_value = read_adc_value_ps(ltr506);

	input_report_abs(ltr506->ps_input_dev, ABS_DISTANCE, adc_value);
	input_sync(ltr506->ps_input_dev);

	if (!ltr506->ps_filter_interrupts) {
		return;
	}

	/* Adjust measurement range using a crude filter to prevent interrupt
	 *  jitter. */
	thresh_delta = (adc_value >> 10)+2;
	thresh_lo = adc_value - thresh_delta;
	thresh_hi = adc_value + thresh_delta;
	if (thresh_lo < LTR506_PS_MIN_MEASURE_VAL)
		thresh_lo = LTR506_PS_MIN_MEASURE_VAL;
	if (thresh_hi > LTR506_PS_MAX_MEASURE_VAL)
		thresh_hi = LTR506_PS_MAX_MEASURE_VAL;
	rc = set_ps_range(ltr506, (uint16_t)thresh_lo, (uint16_t)thresh_hi);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : PS Thresholds Write"
		        " Fail...\n", __func__);
	}
}

/* Read raw sensor value which bypasses the LUX filter.
   The LUX filter has logic to remove 60Hz frequencies which
   was proven to be broken for incandescent lights in previous
   versions of this sensor.
   This should only be called if the als filtered value reports
   a zero.  However, a re-read occurs, and if that is non-zero then
   we return that value back. */
static int _hack_check_valid_als_zero(struct ltr506_data *ltr506) {
	int adc_val;
	uint32_t adc_val_raw_ch1;
	uint32_t adc_val_raw_ch2;

	/* Read the RAW diode sensor values  */
	adc_val_raw_ch1 = read_als_adc_ch1_value(ltr506);
	adc_val_raw_ch2 = read_als_adc_ch2_value(ltr506);

	/* Now re-read the filtered value for good measure */
	adc_val = read_adc_value_als_filtered(ltr506);

	/* Return any non-zero values as that is not the signature of
	   the 50/60Hz filter bug. */
	if (adc_val != 0) {
		return adc_val;
	}

	/* Check if true or bogus darkness value.
	   These checks could be done in a few ways, esp. with
	   more data, but if one or the other raw value is zero,
	   there seems to be a high probability that the filtered
	   value is a true zero too. */
	if (adc_val_raw_ch1 == 0 || adc_val_raw_ch2 == 0) {
		/* True darkness value, so let it ride */
		if (adc_val != ltr506->als_last_value) {
			/* Don't spam logs, or reduce this to debug logging
			   when confident this WA actually works-around */
			dev_info(&ltr506->i2c_client->dev, "%s True dark"
			         " adc_val:%d adc_val_raw_ch1:%d"
			         " adc_val_raw_ch2:%d\n", __func__, adc_val,
			         adc_val_raw_ch1, adc_val_raw_ch2);
		}
	} else {
		/* This is the crux of this hacky work around.
		   We believe we received a bogus darkness value.  Present last
		   value instead under the assumption it's closer to the
		   true value then a bogus zero. */
		dev_info(&ltr506->i2c_client->dev, "%s Not dark presenting last"
		         " value adc_val:%d last_val:%d adc_val_raw_ch1:%d"
		         " adc_val_raw_ch2:%d\n", __func__, adc_val,
		         ltr506->als_last_value, adc_val_raw_ch1,
		         adc_val_raw_ch2);
		adc_val = ltr506->als_last_value;
	}
	dev_dbg(&ltr506->i2c_client->dev, "%s Darkness check adc_val:%d"
	        " adc_val_raw_ch1:%d adc_val_raw_ch2:%d\n",
	        __func__, adc_val, adc_val_raw_ch1,
	        adc_val_raw_ch2);
	return adc_val;
}

/* Report ALS input event and select range */
static void report_als_input_event(struct ltr506_data *ltr506)
{
	int rc;
	int adc_value;
	int thresh_hi, thresh_lo, thresh_delta;

	adc_value = read_adc_value_als(ltr506);

	/* Check if we are using a workaround to bypass the LUX filter. */
	if (ltr506->ps_must_be_on_while_als_on == 1) {
		adc_value = read_als_adc_ch1_value(ltr506);
	}

	if (ltr506->als_from_suspend && (ltr506->als_last_value == adc_value)) {
		adc_value = ltr506->als_last_value ^ 0x0001;
		dev_info(&ltr506->i2c_client->dev, "%s Flipping lsb to force"
		         " fresh value through input subsystem value:%d\n",
		         __func__, adc_value);
	}
	ltr506->als_from_suspend = 0;
	ltr506->als_last_value = adc_value;

	input_report_abs(ltr506->als_input_dev, ABS_MISC, adc_value);
	input_sync(ltr506->als_input_dev);

	if (!ltr506->als_filter_interrupts) {
		return;
	}
	/* Adjust measurement range using a crude filter to prevent interrupt
	 *  jitter. */
	thresh_delta = (adc_value >> 12)+2;
	thresh_lo = adc_value - thresh_delta;
	thresh_hi = adc_value + thresh_delta;
	if (thresh_lo < LTR506_ALS_MIN_MEASURE_VAL)
		thresh_lo = LTR506_ALS_MIN_MEASURE_VAL;
	if (thresh_hi > LTR506_ALS_MAX_MEASURE_VAL)
		thresh_hi = LTR506_ALS_MAX_MEASURE_VAL;
	rc = set_als_range(ltr506, (uint16_t)thresh_lo, (uint16_t)thresh_hi);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : ALS Thresholds Write"
		        " Fail...\n", __func__);
	}
}

/* Work when interrupt */
static void ltr506_schedwork(struct work_struct *work)
{
	int ret;
	uint8_t status;
	uint8_t	interrupt_stat, newdata;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2];

	buffer[0] = LTR506_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__, buffer[0]);
		/* Re-enable interrupts */
		enable_irq(ltr506->irq);
		return;
	}

	/* Re-enable interrupts after reading */
	enable_irq(ltr506->irq);

	status = buffer[0];
	interrupt_stat = status & 0x0a;
	newdata = status & 0x05;

	if (!interrupt_stat) {
		/* There was an interrupt with no work to do */
#ifndef USE_SHARED_IRQ
		int i;
                u8 buf[40];
                dev_dbg(&ltr506->i2c_client->dev,"%s Unexpected received"
                        " interrupt with no work to do status:0x%02x\n",
                        __func__, status);
                buf[0] = 0x80;
                I2C_Read(buf, sizeof(buf));
                for (i = 0; i < sizeof(buf); i++) {
	                dev_dbg(&ltr506->i2c_client->dev, "%s reg:0x%02x"
	                        " val:0x%02x\n", __func__, 0x80+i, buf[i]);
                }
#endif
	} else {
		// TODO(cmanton) Ignore newdata flag since it seems to only
		// be set occasionally and we miss data.
		// newdata & 0x01
		if (interrupt_stat & 0x02) {
			dev_err(&ltr506->i2c_client->dev,
			        "%s Unexpected prox val received\n",
			        __func__);
			report_ps_input_event(ltr506);
		}
		// TODO(cmanton) Ignore newdata flag since it seems to only
		// be set occasionally and we miss data.
		// newdata & 0x04
		if (interrupt_stat & 0x08) {
			if (!(newdata & 0x04)) {
				dev_err(&ltr506->i2c_client->dev,
				        "%s New data for als not set newdata:0x%02x\n",
				        __func__, newdata);
			} else {
				report_als_input_event(ltr506);
			}
		}
	}
}

/* Work when polled */
static void ltr506_schedwork_poll(struct work_struct *work)
{
	struct ltr506_data *ltr506 = sensor_info;
	report_als_input_event(ltr506);
}

static void ltr506_schedwork_delayed(struct work_struct *work);

static DECLARE_WORK(irq_workqueue, ltr506_schedwork);
static DECLARE_DELAYED_WORK(irq_workqueue_delayed, ltr506_schedwork_delayed);

static void ltr506_schedwork_delayed(struct work_struct *work)
{
	ltr506_schedwork_poll(work);
	schedule_delayed_work(&irq_workqueue_delayed, msecs_to_jiffies(poll_rate_in_ms));
}

/* IRQ Handler */
static irqreturn_t ltr506_irq_handler(int irq, void *data)
{
	struct ltr506_data *ltr506 = data;

	/* disable an irq without waiting.  We must disable the interrupt 
	   otherwise the kernel interrupt handler will keep calling this 
	   routine until it's killed due to spending too much time in interrupt
	   context */
	disable_irq_nosync(ltr506->irq);

	schedule_work(&irq_workqueue);

	return IRQ_HANDLED;
}

static int ltr506_setup_polling(struct ltr506_data *ltr506)
{
	// Setup workqueue
	schedule_delayed_work(&irq_workqueue_delayed, msecs_to_jiffies(poll_rate_in_ms));
	return 0;
}


#ifdef USE_SHARED_IRQ
/* Logic to obtain a shared interrupt */
static int ltr506_gpio_irq(struct ltr506_data *ltr506)
{
	int rc = 0;
	unsigned long irq_flags = IRQF_TRIGGER_LOW | IRQF_SHARED;

	dev_info(&ltr506->i2c_client->dev, "%s: Using shared interrupt with"
	         " wink detector\n", __func__);

	rc = gpio_request(ltr506->gpio_int_no, DEVICE_NAME);
	if (rc < 0) {
		if (rc == -EBUSY) {
			dev_info(&ltr506->i2c_client->dev, "%s: gpio request"
			         " busy; assuming glasshub already obtained it\n",
			         __func__);
		} else {
			dev_err(&ltr506->i2c_client->dev,"%s: GPIO %d Request Fail"
			        " (%d)\n", __func__, ltr506->gpio_int_no, rc);
			return rc;
		}
	}

	rc = gpio_direction_input(ltr506->gpio_int_no);
	if (rc < 0) {
		if (rc == -EBUSY) {
			dev_info(&ltr506->i2c_client->dev, "%s: gpio input"
			         " direction request busy; assuming glasshub"
			         " already obtained it\n", __func__);
		} else {
			dev_err(&ltr506->i2c_client->dev, "%s: Set GPIO %d as Input"
			        " Fail (%d)\n", __func__, ltr506->gpio_int_no, rc);
			return rc;
		}
	}

	/* Configure an active low trigger interrupt for the device */
	rc = request_irq(ltr506->irq, ltr506_irq_handler, irq_flags,
	                 DEVICE_NAME, ltr506);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Request IRQ (%d) for"
		        " GPIO %d Fail (%d)\n", __func__, ltr506->irq,
		        ltr506->gpio_int_no, rc);
	}
	return rc;
}

#else
/* Logic to obtain an exclusive interrupt */
static int ltr506_gpio_irq(struct ltr506_data *ltr506)
{
	int rc = 0;
	unsigned long irq_flags = IRQF_TRIGGER_LOW;

	rc = gpio_request(ltr506->gpio_int_no, DEVICE_NAME);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev,"%s: GPIO %d Request Fail"
		        " (%d)\n", __func__, ltr506->gpio_int_no, rc);
		return rc;
	}

	rc = gpio_direction_input(ltr506->gpio_int_no);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Set GPIO %d as Input"
		        " Fail (%d)\n", __func__, ltr506->gpio_int_no, rc);
		goto out1;
	}

	/* Configure an active low trigger interrupt for the device */
	rc = request_irq(ltr506->irq, ltr506_irq_handler, irq_flags,
	                 DEVICE_NAME, ltr506);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Request IRQ (%d) for"
		        " GPIO %d Fail (%d)\n", __func__, ltr506->irq,
		        ltr506->gpio_int_no, rc);
		goto out1;
	}
	return rc;
out1:
	gpio_free(ltr506->gpio_int_no);
	return rc;
}
#endif  /* USE_SHARED_IRQ */

/* LED Setup */
static int ps_led_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[3];

	buffer[0] = LTR506_PS_LED;
	buffer[1] = (ltr506->led_pulse_freq << LED_PULSE_FREQ_SHIFT)
		| (ltr506->led_duty_cyc << LED_DUTY_CYC_SHIFT)
		| (ltr506->led_peak_curr << LED_PEAK_CURR_SHIFT);
	buffer[2] = ltr506->led_pulse_count;
	ret = I2C_Write(buffer, 3);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__, buffer[0]);
	}

	return ret;

}

static int ps_meas_rate_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[2];

	buffer[0] = LTR506_PS_MEAS_RATE;
	buffer[1] = ltr506->ps_meas_rate;
	ret = I2C_Write(buffer, 2);
	if (ret < 0)
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__, buffer[0]);

	return ret;
}

/* Set the ALS measurement rate a.k.a. integration time, and the
 * resolutions, a.k.a. number of bits per sample.
 */
static int als_meas_rate_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[2];

	buffer[0] = LTR506_ALS_MEAS_RATE;
	buffer[1] = (ltr506->als_resolution << ADC_RESOLUTION_SHIFT)
		| (ltr506->als_meas_rate << ALS_MEAS_RATE_SHIFT);
	ret = I2C_Write(buffer, 2);
	if (ret < 0)
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__, buffer[0]);

	return ret;
}

/* PS Enable */
static int ps_enable(struct ltr506_data *ltr506)
{
	int rc = 0;

	if (ltr506->ps_enable_flag) {
		dev_info(&ltr506->i2c_client->dev, "%s: already enabled\n", __func__);
		return rc;
	}

	/* Set thresholds so that interrupts will not be suppressed */
	rc = set_ps_range(ltr506, LTR506_PS_MIN_MEASURE_VAL, LTR506_PS_MIN_MEASURE_VAL);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : PS Thresholds Write Fail...\n",
		        __func__);
		return rc;
	}

	if (ltr506->gpio_int_wake_dev && !ltr506->ps_must_be_on_while_als_on) {
		/* Allows this interrupt to wake the system */
		rc = irq_set_irq_wake(ltr506->irq, 1);
		if (rc < 0) {
			dev_err(&ltr506->i2c_client->dev, "%s: IRQ-%d WakeUp"
			        " Enable Fail...\n", __func__, ltr506->irq);
			return rc;
		}
		dev_info(&ltr506->i2c_client->dev, "%s: Allowing interrupts to"
		         " wake system\n", __func__);
	}

	rc = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_PS_CONTR, PS_MODE);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Enable Fail...\n",
		        __func__);
		return rc;
	}

	dev_info(&ltr506->i2c_client->dev, "%s Turned on proximity sensor\n",
	         __func__);
	ltr506->ps_enable_flag = 1;
	return rc;
}

/* PS Disable */
static int ps_disable(struct ltr506_data *ltr506)
{
	int rc = 0;

	if (ltr506->ps_enable_flag == 0) {
		dev_info(&ltr506->i2c_client->dev, "%s: already disabled\n",
		         __func__);
		return 0;
	}

	/* Don't allow this interrupt to wake the system anymore */
	if (ltr506->gpio_int_wake_dev && !ltr506->ps_must_be_on_while_als_on) {
		rc = irq_set_irq_wake(ltr506->irq, 0);
		if (rc < 0) {
			dev_err(&ltr506->i2c_client->dev, "%s: IRQ-%d WakeUp"
			        " Disable Fail...\n", __func__, ltr506->irq);
			return rc;
		}
	}

	rc = _ltr506_set_bit(ltr506->i2c_client, CLR_BIT, LTR506_PS_CONTR, PS_MODE);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Disable Fail...\n",
		        __func__);
		return rc;
	}

	dev_info(&ltr506->i2c_client->dev, "%s Turned off proximity sensor\n",
	         __func__);
	ltr506->ps_enable_flag = 0;
	return rc;
}

/* PS open fops */
static int ps_open(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;

	if (ltr506->ps_opened)
		return -EBUSY;

	ltr506->ps_opened = 1;

	return 0;
}

/* PS release fops */
static int ps_release(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;

	ltr506->ps_opened = 0;

	return ps_disable(ltr506);
}

static const struct file_operations ps_fops = {
	.owner = THIS_MODULE,
	.open = ps_open,
	.release = ps_release,
};

struct miscdevice ps_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr506_ps",
	.fops = &ps_fops
};

static int als_enable(struct ltr506_data *ltr506)
{
	int rc = 0;

	if (ltr506->als_enable_flag != 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS already enabled..."
		        " disabling first\n", __func__);
		rc = als_disable(ltr506);
		if (rc) {
			dev_err(&ltr506->i2c_client->dev, "%s: Unable to"
			        " disable ALS\n", __func__);
			return rc;
		}
		/* Wait some amount of time for the ALS to disable. */
		msleep(30);
	}

	/* NOTE(CMM) This part requires a workaround to enable the PS in order for the
	 * ALS to operate properly. */
	if (ltr506->ps_must_be_on_while_als_on) {
		if (ps_enable(ltr506)) {
			dev_err(&ltr506->i2c_client->dev, "%s : Unable to turn"
			        " on PS", __func__);
			return -EIO;
		}
		/* Wait some amount of time for the PS to get started. */
		msleep(30);
	}

	/* Clear thresholds so that interrupts will not be suppressed */
	rc = set_als_range(ltr506, LTR506_ALS_MAX_MEASURE_VAL, LTR506_ALS_MAX_MEASURE_VAL);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : ALS Thresholds Write"
		        " Fail...\n", __func__);
		return rc;
	}

	rc = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_ALS_CONTR, ALS_MODE);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Enable Fail...\n",
		        __func__);
		return rc;
	}
	dev_info(&ltr506->i2c_client->dev, "%s Turned on ambient light sensor\n",
	         __func__);
	ltr506->als_enable_flag = 1;

	return rc;
}

static int als_disable(struct ltr506_data *ltr506)
{
	int rc = 0;
	if (ltr506->als_enable_flag != 1) {
		dev_err(&ltr506->i2c_client->dev, "%s : ALS already disabled...\n",
		        __func__);
		return rc;
	}

	rc = _ltr506_set_bit(ltr506->i2c_client, CLR_BIT, LTR506_ALS_CONTR, ALS_MODE);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev,"%s: ALS Disable Fail...\n",
		        __func__);
		return rc;
	}
	dev_info(&ltr506->i2c_client->dev, "%s Turned off ambient light sensor\n",
	         __func__);
	ltr506->als_enable_flag = 0;

	/* NOTE(CMM) This part requires a workaround to enable the PS in order for the
	 * ALS to operate properly.  For symmetry we would typically disable the PS here, but
	 * we cannot disable the PS elae the ALS will not work.  */
#if 0
	if (ltr506->ps_must_be_on_while_als_on) {
		if (ps_disable(ltr506)) {
			dev_err(&ltr506->i2c_client->dev, "%s : Unable to turn off PS", __func__);
		}
	}
#endif
	return rc;
}

static int als_open(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;
	int rc = 0;

	if (ltr506->als_opened) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS already Opened...\n",
		        __func__);
		rc = -EBUSY;
	}
	ltr506->als_opened = 1;
	return rc;
}

static int als_release(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;

	ltr506->als_opened = 0;
	return 0;
}

static const struct file_operations als_fops = {
	.owner = THIS_MODULE,
	.open = als_open,
	.release = als_release,
};

static struct miscdevice als_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr506_ls",
	.fops = &als_fops
};

static ssize_t ps_adc_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;

	value = read_adc_value_ps(ltr506);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}


static DEVICE_ATTR(ps_adc, 0666, ps_adc_show, NULL);

/* PS LED */
static ssize_t ps_led_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t value;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2] = {LTR506_PS_LED, 0};
	int led_pulse_freq, led_duty_cyc, led_peak_curr;

	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return ret;
	}
	value = buffer[0];

	led_pulse_freq = (value & LED_PULSE_FREQ) >> LED_PULSE_FREQ_SHIFT;
	led_duty_cyc = (value & LED_DUTY_CYC) >> LED_DUTY_CYC_SHIFT;
	led_peak_curr = (value & LED_PEAK_CURR) >> LED_PEAK_CURR_SHIFT;

	return sprintf(buf, "%d %d %d\n", led_pulse_freq, led_duty_cyc, led_peak_curr);
}

static ssize_t ps_led_store(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
	int ret = 0;
	struct ltr506_data *ltr506 = sensor_info;
	int led_pulse_freq, led_duty_cyc, led_peak_curr;

	sscanf(buf, "%d %d %d", &led_pulse_freq, &led_duty_cyc, &led_peak_curr);

	if (led_pulse_freq & ~(LED_PULSE_FREQ_BITS)
	    || led_duty_cyc & ~(LED_DUTY_CYC_BITS)
	    || led_peak_curr & ~(LED_PEAK_CURR_BITS)) {
		return -EINVAL;
	}

	ltr506->led_pulse_freq = led_pulse_freq;
	ltr506->led_duty_cyc = led_duty_cyc;
	ltr506->led_peak_curr = led_peak_curr;

	ret = ps_led_setup(ltr506);
	return (ret == 0) ? count : -EIO;
}

static DEVICE_ATTR(ps_led, 0666, ps_led_show, ps_led_store);

static ssize_t als_adc_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	int value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;

	value = read_adc_value_als(ltr506);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}

static DEVICE_ATTR(als_adc, 0666, als_adc_show, NULL);

static ssize_t als_adc_ch1_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	int value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;

	value = read_als_adc_ch1_value(ltr506);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}
static DEVICE_ATTR(als_adc_ch1, 0666, als_adc_ch1_show, NULL);

static ssize_t als_adc_ch2_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	int value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;

	value = read_als_adc_ch2_value(ltr506);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}
static DEVICE_ATTR(als_adc_ch2, 0666, als_adc_ch2_show, NULL);

static ssize_t als_enable_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2];

	buffer[0] = LTR506_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return ret;
	}
	value = buffer[0];

	ret = sprintf(buf, "%d\n", (value & ALS_MODE) ? 1 : 0);

	return ret;
}

static ssize_t als_enable_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
	int rc = 0;
	int als_enable_flag;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &als_enable_flag);
	if ((als_enable_flag != 0) && (als_enable_flag != 1)) {
		return -EINVAL;
	}

	if (als_enable_flag && (ltr506->als_enable_flag == 0)) {
		rc = als_enable(ltr506);
	} else if (!als_enable_flag && (ltr506->als_enable_flag == 1)) {
		rc = als_disable(ltr506);
	}
	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(als_enable, 0666, als_enable_show, als_enable_store);


static ssize_t als_gain_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	int rc;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2];
	int als_gain;

	buffer[0] = LTR506_ALS_CONTR;
	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return rc;
	}

	als_gain = (buffer[0] & ALS_GAIN) >> ALS_GAIN_SHIFT;

	return (rc < 0) ? rc : sprintf(buf, "%d\n", als_gain);
}

static ssize_t als_gain_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
	int rc = 0;
	int als_gain;
	struct ltr506_data *ltr506 = sensor_info;
	int als_enable_flag = ltr506->als_enable_flag;

	sscanf(buf, "%d", &als_gain);
	if (als_gain & ~(ALS_GAIN_BITS)) {
		return -EINVAL;
	}

	/* Disable the device if enabled */
	if (als_enable_flag && als_disable(ltr506)) {
		return -EIO;
	}

	rc = _ltr506_set_field(ltr506->i2c_client, ALS_GAIN, LTR506_ALS_CONTR,
	                       als_gain << ALS_GAIN_SHIFT);

	/* Reenable the device if previously enabled */
	if (als_enable_flag && als_enable(ltr506)) {
	    return -EIO;
	}

	return (rc == 0) ? count : rc;
}
static DEVICE_ATTR(als_gain, 0666, als_gain_show, als_gain_store);

static ssize_t als_resolution_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	int rc;
	uint8_t value;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2] = { LTR506_ALS_MEAS_RATE, 0 };
	int als_adc_resolution;

	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return rc;
	}

	value = buffer[0];
	als_adc_resolution = (value & ADC_RESOLUTION) >> ADC_RESOLUTION_SHIFT;

	return (rc < 0) ? rc : sprintf(buf, "%d\n", als_adc_resolution);
}

static ssize_t als_resolution_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc = 0;
	struct ltr506_data *ltr506 = sensor_info;
	int als_adc_resolution;

	sscanf(buf, "%d", &als_adc_resolution);
	if (als_adc_resolution & ~(ADC_RESOLUTION_BITS)) {
		return -EINVAL;
	}

	ltr506->als_resolution = als_adc_resolution;

	rc = als_meas_rate_setup(ltr506);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(als_resolution, 0666, als_resolution_show, als_resolution_store);


static ssize_t als_meas_rate_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2] = { LTR506_ALS_MEAS_RATE, 0 };

	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return ret;
	}
	value = buffer[0];

	ret = sprintf(buf, "%d\n", ((value & ALS_MEAS_RATE) >> 0));

	return ret;
}

static ssize_t als_meas_rate_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	int rc = 0;
	int als_meas_rate;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &als_meas_rate);
	if (als_meas_rate & ~(ALS_MEAS_RATE_BITS)) {
		return -EINVAL;
	}

	ltr506->als_meas_rate = als_meas_rate;
	rc = als_meas_rate_setup(ltr506);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(als_meas_rate, 0666, als_meas_rate_show, als_meas_rate_store);

static ssize_t als_threshold_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct ltr506_data *ltr506 = sensor_info;
	return sprintf(buf, "%u %u\n", ltr506->als_lowthresh, ltr506->als_highthresh);
}

static ssize_t als_threshold_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
	int rc = 0;
	struct ltr506_data *ltr506 = sensor_info;
	unsigned int thresh_low, thresh_high;
	sscanf(buf, "%u %u", &thresh_low, &thresh_high);
	if (thresh_low > 4095 || thresh_high > 4095) {
		return -EINVAL;
	}

	rc = set_als_range(ltr506, (uint16_t)thresh_low, (uint16_t)thresh_high);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(als_threshold, 0666, als_threshold_show, als_threshold_store);


static ssize_t ps_enable_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2];

	buffer[0] = LTR506_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return ret;
	}
	value = buffer[0];

	ret = sprintf(buf, "%d\n", (value & PS_MODE) ? 1 : 0);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
	int rc = 0;
	int ps_en;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &ps_en);
	if ((ps_en != 0) && (ps_en != 1)) {
		return -EINVAL;
	}

	if (ps_en && (ltr506->ps_enable_flag == 0)) {
		rc = ps_enable(ltr506);
	} else if (!ps_en && (ltr506->ps_enable_flag == 1)) {
		rc = ps_disable(ltr506);
	}
	return (rc == 0)?count:rc;
}

static DEVICE_ATTR(ps_enable, 0666, ps_enable_show, ps_enable_store);

static ssize_t ps_gain_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	int rc;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2];
	int ps_gain;

	buffer[0] = LTR506_PS_CONTR;
	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return rc;
	}

	ps_gain = (buffer[0] & PS_GAIN) >> PS_GAIN_SHIFT;

	return (rc < 0) ? rc : sprintf(buf, "%d\n", ps_gain);
}

static ssize_t ps_gain_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc = 0;
	int ps_gain;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &ps_gain);
	if (ps_gain & ~(PS_GAIN_BITS)) {
		return -EINVAL;
	}

	rc = _ltr506_set_field(ltr506->i2c_client, PS_GAIN, LTR506_PS_CONTR,
	                       ps_gain << PS_GAIN_SHIFT);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(ps_gain, 0666, ps_gain_show, ps_gain_store);

static ssize_t ps_pulse_cnt_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	int rc;
	uint8_t value;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2] = { LTR506_PS_N_PULSES, 0 };

	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return rc;
	}

	value = buffer[0];

	return (rc < 0) ? rc : sprintf(buf, "%d\n", value);
}

static ssize_t ps_pulse_cnt_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc = 0;
	int ps_pulse_cnt;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2] = { LTR506_PS_N_PULSES, 0 };

	sscanf(buf, "%d", &ps_pulse_cnt);
	if (ps_pulse_cnt & ~(0xff)) {
		return -EINVAL;
	}

	buffer[1] = ps_pulse_cnt;
	rc = I2C_Write(buffer, 2);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return -EIO;
	}

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(ps_pulse_cnt, 0666, ps_pulse_cnt_show, ps_pulse_cnt_store);

static ssize_t ps_meas_rate_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	int rc;
	uint8_t value;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2] = { LTR506_PS_MEAS_RATE, 0 };

	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X\n", __func__,
		        buffer[0]);
		return rc;
	}

	value = (buffer[0] & PS_MEAS_RATE) >> PS_MEAS_RATE_SHIFT;

	return (rc < 0) ? rc : sprintf(buf, "%d\n", value);
}

static ssize_t ps_meas_rate_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc = 0;
	int ps_meas_rate;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &ps_meas_rate);
	if (ps_meas_rate & ~(PS_MEAS_RATE_BITS)) {
		return -EINVAL;
	}

	ltr506->ps_meas_rate = ps_meas_rate;
	rc = ps_meas_rate_setup(ltr506);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(ps_meas_rate, 0666, ps_meas_rate_show, ps_meas_rate_store);


static ssize_t ps_threshold_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct ltr506_data *ltr506 = sensor_info;
	return sprintf(buf, "%u %u\n", ltr506->ps_lowthresh,
	               ltr506->ps_highthresh);
}

static ssize_t ps_threshold_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
	int rc = 0;
	struct ltr506_data *ltr506 = sensor_info;
	unsigned int thresh_low, thresh_high;
	sscanf(buf, "%u %u", &thresh_low, &thresh_high);
	if (thresh_low > 4095 || thresh_high > 4095) {
		return -EINVAL;
	}

	rc = set_ps_range(ltr506, (uint16_t)thresh_low, (uint16_t)thresh_high);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(ps_threshold, 0666, ps_threshold_show, ps_threshold_store);



#define HOLE_REG_SPACE(a,b) buffer[0] = a; \
	I2C_Read(buffer, (b+1)-a); \
	for (i = 0; i < (b+1)-a; i++) { \
		buf += sprintf(buf, "0x%02x: 0x%02x\n", i+a, buffer[i]); \
	} \

static ssize_t dump_regs_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
	char *tmp_buf = buf;
	uint8_t buffer[64];
	int i;

	/* There are holes in the address space */
	HOLE_REG_SPACE(0x80, 0x9c);
	HOLE_REG_SPACE(0x9e, 0xa1);
	HOLE_REG_SPACE(0xa4, 0xa4);

	return strlen(tmp_buf);

}
static DEVICE_ATTR(dump_regs, 0666, dump_regs_show, NULL);

static ssize_t status_show(struct device *dev,
                           struct device_attribute *attr,
                           char *buf)
{
	int ret = 0;
	char *tmp_buf = buf;
	uint16_t min, max;
	uint8_t buffer[64];

	buffer[0] = LTR506_ALS_THRES_UP_0;

	ret = I2C_Read(buffer, 5);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return 0;
	}

	max = (u16)buffer[2] | ((u16)buffer[1] << 8);
	min = (u16)buffer[4] | ((u16)buffer[3] << 8);
	buf += sprintf(buf, "als min:%d max:%d\n", min, max);

	buffer[0] = LTR506_PS_THRES_UP_0;

	ret = I2C_Read(buffer, 5);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return 0;
	}

	max = (u16)buffer[2] | ((u16)buffer[1] << 8);
	min = (u16)buffer[4] | ((u16)buffer[3] << 8);
	buf += sprintf(buf, "ps min:%d max:%d\n", min, max);

	return strlen(tmp_buf);

}
static DEVICE_ATTR(status, 0666, status_show, NULL);



static ssize_t als_filter_interrupts_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct ltr506_data *ltr506 = sensor_info;
	return sprintf(buf, "%d\n", ltr506->als_filter_interrupts);
}

static ssize_t als_filter_interrupts_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc;
	int als_filter_interrupts;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &als_filter_interrupts);
	if (als_filter_interrupts != 0 && als_filter_interrupts != 1) {
		return -EINVAL;
	}

	/* Clear thresholds so that interrupts will not be suppressed */
	rc = set_als_range(ltr506, LTR506_ALS_MAX_MEASURE_VAL,
	                   LTR506_ALS_MAX_MEASURE_VAL);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : ALS Thresholds Write"
		        " Fail...\n", __func__);
		return -EIO;
	}

	ltr506->als_filter_interrupts = als_filter_interrupts;

	return count;
}

static DEVICE_ATTR(als_filter_interrupts, 0666, als_filter_interrupts_show,
                   als_filter_interrupts_store);

static ssize_t ps_filter_interrupts_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct ltr506_data *ltr506 = sensor_info;
	return sprintf(buf, "%d\n", ltr506->ps_filter_interrupts);
}

static ssize_t ps_filter_interrupts_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc;
	int ps_filter_interrupts;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &ps_filter_interrupts);
	if (ps_filter_interrupts != 0 && ps_filter_interrupts != 1) {
		return -EINVAL;
	}

	rc = set_ps_range(ltr506, LTR506_PS_MIN_MEASURE_VAL, LTR506_PS_MIN_MEASURE_VAL);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : PS Thresholds Write"
		        " Fail...\n", __func__);
		return -EIO;
	}
	ltr506->ps_filter_interrupts = ps_filter_interrupts;

	return count;
}

static DEVICE_ATTR(ps_filter_interrupts, 0666, ps_filter_interrupts_show,
                   ps_filter_interrupts_store);

/*
 * These sysfs routines are not used by the Android HAL layer
 * as they are located in the i2c bus device portion of the
 * sysfs tree.
 */
static void sysfs_register_device(struct i2c_client *client) {
	int rc = 0;

	rc += device_create_file(&client->dev, &dev_attr_als_enable);
	rc += device_create_file(&client->dev, &dev_attr_ps_enable);
	rc += device_create_file(&client->dev, &dev_attr_dump_regs);
	rc += device_create_file(&client->dev, &dev_attr_status);
	rc += device_create_file(&client->dev, &dev_attr_als_filter_interrupts);
	rc += device_create_file(&client->dev, &dev_attr_ps_filter_interrupts);

	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs files\n",
		        __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs files\n", __func__);
	}
}

/*
 * These sysfs routines are exposed to the Android HAL layer as they are
 * created in the class/input portion of the sysfs tree.
 */
static void sysfs_register_als_device(struct i2c_client *client, struct device *dev) {
	int rc = 0;
	rc += device_create_file(dev, &dev_attr_als_adc);
	rc += device_create_file(dev, &dev_attr_als_adc_ch1);
	rc += device_create_file(dev, &dev_attr_als_adc_ch2);
	rc += device_create_file(dev, &dev_attr_als_resolution);
	rc += device_create_file(dev, &dev_attr_als_enable);
	rc += device_create_file(dev, &dev_attr_als_gain);
	rc += device_create_file(dev, &dev_attr_als_meas_rate);
	rc += device_create_file(dev, &dev_attr_als_threshold);
	rc += device_create_file(dev, &dev_attr_als_filter_interrupts);
	if (rc) {
		dev_err(&client->dev, "%s Unable to create als input sysfs"
		        " files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created als input sysfs files\n",
		        __func__);
	}
}

/*
 * These sysfs routines are exposed to the Android HAL layer as they are
 * created in the class/input portion of the sysfs tree.
 */
static void sysfs_register_ps_device(struct i2c_client *client, struct device *dev) {
	int rc = 0;

	rc += device_create_file(dev, &dev_attr_ps_adc);
	rc += device_create_file(dev, &dev_attr_ps_enable);
	rc += device_create_file(dev, &dev_attr_ps_gain);
	rc += device_create_file(dev, &dev_attr_ps_led);
	rc += device_create_file(dev, &dev_attr_ps_meas_rate);
	rc += device_create_file(dev, &dev_attr_ps_pulse_cnt);
	rc += device_create_file(dev, &dev_attr_ps_threshold);
	rc += device_create_file(dev, &dev_attr_ps_filter_interrupts);
	if (rc) {
		dev_err(&client->dev, "%s Unable to create ps input sysfs"
		        " files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created ps input sysfs files\n",
		        __func__);
	}
}


static int als_setup_input_device(struct ltr506_data *ltr506)
{
	int ret;

	ltr506->als_input_dev = input_allocate_device();
	if (!ltr506->als_input_dev) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Input Allocate"
		        " Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr506->als_input_dev->name = "ltr506_als";
	set_bit(EV_ABS, ltr506->als_input_dev->evbit);
	input_set_abs_params(ltr506->als_input_dev, ABS_MISC, LTR506_ALS_MIN_MEASURE_VAL, LTR506_ALS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(ltr506->als_input_dev);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Register Input"
		        " Device Fail...\n", __func__);
		goto err_als_register_input_device;
	}

	ret = misc_register(&als_misc);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Register Misc"
		        " Device Fail...\n", __func__);
		goto err_als_register_misc_device;
	}

	return ret;

err_als_register_misc_device:
	input_unregister_device(ltr506->als_input_dev);
err_als_register_input_device:
	input_free_device(ltr506->als_input_dev);

	return ret;
}

static int ps_setup_input_device(struct ltr506_data *ltr506)
{
	int ret;

	ltr506->ps_input_dev = input_allocate_device();
	if (!ltr506->ps_input_dev) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Input Allocate Device"
		        " Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr506->ps_input_dev->name = "ltr506_ps";
	set_bit(EV_ABS, ltr506->ps_input_dev->evbit);
	input_set_abs_params(ltr506->ps_input_dev, ABS_DISTANCE, LTR506_PS_MIN_MEASURE_VAL, LTR506_PS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(ltr506->ps_input_dev);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Register Input Device"
		        " Fail...\n", __func__);
		goto err_ps_register_input_device;
	}

	ret = misc_register(&ps_misc);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Register Misc Device"
		        " Fail...\n", __func__);
		goto err_ps_register_misc_device;
	}

	return ret;

err_ps_register_misc_device:
	input_unregister_device(ltr506->ps_input_dev);
err_ps_register_input_device:
	input_free_device(ltr506->ps_input_dev);

	return ret;
}

static int _check_part_id(struct ltr506_data *ltr506)
{
	int ret;
	u8 buffer[2];
	buffer[0] = LTR506_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Read failure :0x%02X\n",
		        __func__, buffer[0]);
		return -1;
	}

	if (buffer[0] != PARTID && buffer[0] != PARTID_V2) {
		dev_err(&ltr506->i2c_client->dev, "%s: Part failure miscompare"
		        " act:0x%02x exp:0x%02x\n", __func__, buffer[0], PARTID);
		return -2;
	}
	if (buffer[0] == PARTID) {
		ltr506->ps_must_be_on_while_als_on = 1;
	}
	ltr506->part_id = buffer[0];
	return 0;
}

static int ltr506_setup(struct ltr506_data *ltr506)
{
	int ret = 0;

	/* Reset the devices */
	ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS reset fail...\n",
		        __func__);
		goto err_out1;
	}

	msleep(LTR506_PON_DELAY);
	dev_dbg(&ltr506->i2c_client->dev, "%s: Reset ltr506 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(ltr506) < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Part ID Read Fail after"
		        " reset...\n", __func__);
		goto err_out1;
	}

	ret = ltr506_gpio_irq(ltr506);
	if (ret < 0) {
		dev_warn(&ltr506->i2c_client->dev, "%s: Unable to setup"
		         " interrupts; retrying with polling...\n", __func__);
		ret = ltr506_setup_polling(ltr506);
		if (ret < 0) {
			dev_err(&ltr506->i2c_client->dev, "%s: GPIO Request"
			        " Fail...\n", __func__);
			goto err_out1;
		}
		ltr506->use_polling = 1;
	}
	dev_dbg(&ltr506->i2c_client->dev, "%s Requested interrupt\n", __func__);

	/* Set count of measurements outside data range before interrupt is generated */
	ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_INTERRUPT_PRST, 0x00);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Set Persist Fail...\n",
		        __func__);
		goto err_out2;
	}

	if (!ltr506->use_polling) {
		ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_INTERRUPT_PRST, 0x00);
		if (ret < 0) {
			dev_err(&ltr506->i2c_client->dev,"%s: PS Set Persist Fail...\n",
			        __func__);
			goto err_out2;
		}
		dev_dbg(&ltr506->i2c_client->dev, "%s: Set ltr506 persists\n",
		        __func__);

		/* Enable interrupts on the device and clear only when status is read */
		ret = _ltr506_set_bit(ltr506->i2c_client, CLR_BIT, LTR506_INTERRUPT, 0x08);
		ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_INTERRUPT, INTERRUPT_MODE);
		if (ret < 0) {
			dev_err(&ltr506->i2c_client->dev, "%s: Enabled"
			        " interrupts failed...\n", __func__);
			goto err_out2;
		}
		dev_info(&ltr506->i2c_client->dev, "%s Enabled interrupt to"
		         " device\n", __func__);
	}

	/* Set ALS measurement gain */
	ret = _ltr506_set_field(ltr506->i2c_client, ALS_GAIN, LTR506_ALS_CONTR,
	                        ltr506->als_gain << ALS_GAIN_SHIFT);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS set gain fail...\n",
		        __func__);
		goto err_out2;
	}

	/* Set PS measurement gain */
	ret = _ltr506_set_field(ltr506->i2c_client, PS_GAIN, LTR506_PS_CONTR,
	                        ltr506->ps_gain << PS_GAIN_SHIFT);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS set gain fail...\n",
		        __func__);
		goto err_out2;
	}
	dev_dbg(&ltr506->i2c_client->dev, "%s: Set ltr506 gains\n", __func__);

	/* Set the ALS measurement rate and resolution */
	als_meas_rate_setup(ltr506);

	ret = ps_led_setup(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS LED Setup Fail...\n",
		        __func__);
		goto err_out2;
	}

	ret = ps_meas_rate_setup(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS MeasRate Setup"
		        " Fail...\n",
		        __func__);
		goto err_out2;
	}

	dev_info(&ltr506->i2c_client->dev, "%s Using part id:0x%02x\n",
	         __func__, ltr506->part_id);

	return ret;

err_out2:
	free_irq(ltr506->irq, ltr506);
	gpio_free(ltr506->gpio_int_no);

err_out1:
	dev_err(&ltr506->i2c_client->dev, "%s Unable to setup device\n",
	        __func__);
	return ret;
}

static void ltr506_early_suspend(struct early_suspend *h)
{
	int ret = 0;
	struct ltr506_data *ltr506 = sensor_info;

	if (ltr506->is_suspend != 0) {
		dev_err(&ltr506->i2c_client->dev, "%s Asked to suspend when"
		        " already suspended\n", __func__);
		return;
	}
	ltr506->is_suspend = 1;

	/* Save away the state of the devices at suspend point */
	ltr506->als_suspend_enable_flag = ltr506->als_enable_flag;
	ltr506->ps_suspend_enable_flag = ltr506->ps_enable_flag;

	/* Disable the devices for suspend if configured */
	if (ltr506->disable_als_on_suspend && ltr506->als_enable_flag) {
		ret += als_disable(ltr506);
	}
	if (ltr506->disable_ps_on_suspend && ltr506->ps_enable_flag) {
		ret += ps_disable(ltr506);
	}

	if (ret) {
		dev_err(&ltr506->i2c_client->dev, "%s Unable to complete"
		        " suspend\n", __func__);
	} else {
		dev_dbg(&ltr506->i2c_client->dev, "%s Suspend completed\n",
		        __func__);
	}
}

static void ltr506_late_resume(struct early_suspend *h)
{
	struct ltr506_data *ltr506 = sensor_info;
	int ret = 0;

	if (ltr506->is_suspend != 1) {
		dev_err(&ltr506->i2c_client->dev, "%s Asked to resume when"
		        " not suspended\n", __func__);
		return;
	}
	ltr506->is_suspend = 0;

	ltr506->als_from_suspend = 1;

	/* If ALS was enbled before suspend, enable during resume */
	if (ltr506->disable_als_on_suspend && ltr506->als_suspend_enable_flag) {
		ret += als_enable(ltr506);
		ltr506->als_suspend_enable_flag = 0;
	}

	/* If PS was enbled before suspend, enable during resume */
	if (ltr506->disable_ps_on_suspend && ltr506->ps_suspend_enable_flag) {
		ret += ps_enable(ltr506);
		ltr506->ps_suspend_enable_flag = 0;
	}

	if (ret) {
		dev_err(&ltr506->i2c_client->dev, "%s Unable to complete"
		        " resume\n", __func__);
	} else {
		dev_dbg(&ltr506->i2c_client->dev, "%s Resume completed\n",
		        __func__);
	}
}

static int  __devinit ltr506_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct ltr506_data *ltr506;
	struct ltr506_platform_data *platdata;

	ltr506 = kzalloc(sizeof(struct ltr506_data), GFP_KERNEL);
	if (!ltr506)
	{
		dev_err(&ltr506->i2c_client->dev, "%s: Mem Alloc Fail...\n",
		        __func__);
		return -ENOMEM;
	}

	/* Global pointer for this device */
	sensor_info = ltr506;

	/* Set initial defaults */
	ltr506->als_enable_flag = 0;
	ltr506->ps_enable_flag = 0;

	ltr506->i2c_client = client;
	ltr506->irq = client->irq;

	i2c_set_clientdata(client, ltr506);

	/* Parse the platform data */
	platdata = client->dev.platform_data;
	if (!platdata) {
		dev_err(&ltr506->i2c_client->dev, "%s: Platform Data assign"
		        " Fail...\n", __func__);
		ret = -EBUSY;
		goto err_out;
	}

	/* Get ALS defaults from platform data */
	ltr506->als_resolution = platdata->pfd_als_resolution;
	ltr506->als_meas_rate = platdata->pfd_als_meas_rate;
	ltr506->als_gain = platdata->pfd_als_gain;
	ltr506->als_filter_interrupts = platdata->pfd_als_filter_interrupts;

	/* Get the PS defaults from platform data */
	ltr506->ps_meas_rate = platdata->pfd_ps_meas_rate;
	ltr506->ps_gain = platdata->pfd_ps_gain;
	ltr506->ps_filter_interrupts = platdata->pfd_ps_filter_interrupts;

	/* Get LED defaults from platform data */
	ltr506->led_pulse_freq = platdata->pfd_led_pulse_freq;
	ltr506->led_duty_cyc = platdata->pfd_led_duty_cyc;
	ltr506->led_peak_curr = platdata->pfd_led_peak_curr;
	ltr506->led_pulse_count = platdata->pfd_led_pulse_count;

	ltr506->gpio_int_no = platdata->pfd_gpio_int_no;
	ltr506->gpio_int_wake_dev = platdata->pfd_gpio_int_wake_dev;

	/* Configuration to set or disable devices upon suspend */
	ltr506->disable_als_on_suspend = platdata->pfd_disable_als_on_suspend;
	ltr506->disable_ps_on_suspend = platdata->pfd_disable_ps_on_suspend;

	if (_check_part_id(ltr506) < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Part ID Read Fail...\n",
		        __func__);
		goto err_out;
	}

	/* Setup and configure both the ALS and PS on the ltr506 device */
	ret = ltr506_setup(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Setup Fail...\n",
		        __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the ALS */
	ret = als_setup_input_device(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev,"%s: ALS Setup Fail...\n",
		        __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the PS */
	ret = ps_setup_input_device(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Setup Fail...\n",
		        __func__);
		goto err_out;
	}

	/* Create the workqueue for the interrup handler */
	ltr506->workqueue = create_singlethread_workqueue("ltr506_workqueue");
	if (!ltr506->workqueue) {
		dev_err(&ltr506->i2c_client->dev, "%s: Create WorkQueue"
		        " Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}

	/* Wake lock option for promity sensor */
	wake_lock_init(&(ltr506->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	/* Setup the suspend and resume functionality */
	INIT_LIST_HEAD(&ltr506->early_suspend.link);
	ltr506->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ltr506->early_suspend.suspend = ltr506_early_suspend;
	ltr506->early_suspend.resume = ltr506_late_resume;
	register_early_suspend(&ltr506->early_suspend);

	/* Register the sysfs files */
	sysfs_register_device(client);
	sysfs_register_als_device(client, &ltr506->als_input_dev->dev);
	sysfs_register_ps_device(client, &ltr506->ps_input_dev->dev);

	dev_dbg(&ltr506->i2c_client->dev, "%s: probe complete\n", __func__);
	return ret;

err_out:
	kfree(ltr506);
	return ret;
}

static const struct i2c_device_id ltr506_id[] = {
	{ DEVICE_NAME, 0 },
	{}
};


static struct i2c_driver ltr506_driver = {
	.probe = ltr506_probe,
	.id_table = ltr506_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
};

static int __init ltr506_init(void)
{
	return i2c_add_driver(&ltr506_driver);
}


static void __exit ltr506_exit(void)
{
	i2c_del_driver(&ltr506_driver);
}


	module_init(ltr506_init)
module_exit(ltr506_exit)

	MODULE_AUTHOR("Lite-On Technology Corp");
	MODULE_DESCRIPTION("LTR-506ALS Driver");
	MODULE_LICENSE("Dual BSD/GPL");
	MODULE_VERSION(DRIVER_VERSION);
