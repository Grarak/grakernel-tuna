/* Lite-On LTR-506ALS Android Driver
 *
 * Copyright (C) 2011 Lite-On Technology Corp (Singapore)
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#ifndef __LTR506_H
#define __LTR506_H

/* LTR-506 Registers */
#define LTR506_ALS_CONTR	0x80
#define LTR506_PS_CONTR		0x81
#define LTR506_PS_LED		0x82
#define LTR506_PS_N_PULSES	0x83
#define LTR506_PS_MEAS_RATE	0x84
#define LTR506_ALS_MEAS_RATE	0x85
#define LTR506_PART_ID		0x86
#define LTR506_MANUFACTURER_ID	0x87
#define LTR506_ALS_DATA_0	0x88
#define LTR506_ALS_DATA_1	0x89
#define LTR506_ALS_PS_STATUS	0x8A
#define LTR506_PS_DATA_0	0x8B
#define LTR506_PS_DATA_1	0x8C

#define LTR506_ALS_DATA_CH1_0		0x8D
#define LTR506_ALS_DATA_CH1_1		0x8E
#define LTR506_ALS_DATA_CH1_2		0x8F
#define LTR506_ALS_DATA_CH2_0		0x90
#define LTR506_ALS_DATA_CH2_1		0x91
#define LTR506_ALS_DATA_CH2_2		0x92
#define LTR506_ALS_COEFF1_DATA_0	0x93
#define LTR506_ALS_COEFF1_DATA_1	0x94
#define LTR506_ALS_COEFF2_DATA_0	0x95
#define LTR506_ALS_COEFF2_DATA_1	0x96
#define LTR506_ALS_IRF_CUT_OFF		0x97

#define LTR506_INTERRUPT	0x98
#define LTR506_PS_THRES_UP_0	0x99
#define LTR506_PS_THRES_UP_1	0x9A
#define LTR506_PS_THRES_LOW_0	0x9B
#define LTR506_PS_THRES_LOW_1	0x9C
#define LTR506_ALS_THRES_UP_0	0x9E
#define LTR506_ALS_THRES_UP_1	0x9F
#define LTR506_ALS_THRES_LOW_0	0xA0
#define LTR506_ALS_THRES_LOW_1	0xA1
#define LTR506_INTERRUPT_PRST	0xA4

#define SET_BIT 1
#define CLR_BIT 0

/* Default Settings (Bitshift left: Setting << Bit Number) */
#define ADC_RESOLUTION_BITS     0x7
#define ADC_RESOLUTION_SHIFT    5
#define ADC_RESOLUTION          (ADC_RESOLUTION_BITS << ADC_RESOLUTION_SHIFT)
#define ALS_GAIN_BITS           0x3
#define ALS_GAIN_SHIFT          3
#define ALS_GAIN                (ALS_GAIN_BITS << ALS_GAIN_SHIFT)
#define ALS_SW_RESET		(1 << 2)
#define ALS_MODE		(1 << 1)
#define ALS_MEAS_RATE_BITS      0x7
#define ALS_MEAS_RATE_SHIFT     0
#define ALS_MEAS_RATE           (ALS_MEAS_RATE_BITS << ALS_MEAS_RATE_SHIFT)

#define ALS_INT_PRST		(0xf << 0)

#define ALS_INT_FLAG		(1 << 3)
#define ALS_NEWDATA		(1 << 2)

#define PS_SW_RESET		(1 << 2)

#define PS_GAIN_BITS            0x3
#define PS_GAIN_SHIFT           2
#define PS_GAIN                 (PS_GAIN_BITS << PS_GAIN_SHIFT)

#define PS_MODE			(1 << 1)

#define PS_MEAS_RATE_BITS       0x7
#define PS_MEAS_RATE_SHIFT      0
#define PS_MEAS_RATE            (PS_MEAS_RATE_BITS << PS_MEAS_RATE_SHIFT)

#define PS_INT_PRST		(0xf << 4)

#define PS_INT_FLAG		(1 << 1)
#define PS_NEWDATA		(1 << 0)

#define INTERRUPT_MODE		(3 << 0)
#define INTERRUPT_POL		(1 << 2)
#define INTERRUPT_OUTPUT_MODE	(1 << 3)

#define LED_PULSE_FREQ_BITS     0x7
#define LED_PULSE_FREQ_SHIFT    5
#define LED_PULSE_FREQ          (LED_PULSE_FREQ_BITS << LED_PULSE_FREQ_SHIFT)
#define LED_DUTY_CYC_BITS       0x3
#define LED_DUTY_CYC_SHIFT      3
#define LED_DUTY_CYC            (LED_DUTY_CYC_BITS << LED_DUTY_CYC_SHIFT)
#define LED_PEAK_CURR_BITS      0x7
#define LED_PEAK_CURR_SHIFT     0
#define LED_PEAK_CURR           (LED_PEAK_CURR_BITS << LED_PEAK_CURR_SHIFT)

/* Power On response time in ms */
#define LTR506_PON_DELAY	600
#define LTR506_WAKEUP_DELAY	10

#define LTR506_ALS_MIN_MEASURE_VAL	0
#define LTR506_ALS_MAX_MEASURE_VAL	65535
#define LTR506_ALS_VALID_MEASURE_MASK	LTR506_ALS_MAX_MEASURE_VAL
#define LTR506_PS_MIN_MEASURE_VAL	0x0
#define LTR506_PS_MAX_MEASURE_VAL	4095
#define LTR506_PS_VALID_MEASURE_MASK   LTR506_PS_MAX_MEASURE_VAL


/*
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LTR506_IOCTL_MAGIC      'c'

/* IOCTLs for ltr506 device */
#define LTR506_IOCTL_PS_ENABLE		_IOR(LTR506_IOCTL_MAGIC, 1, int *)
#define LTR506_IOCTL_PS_GET_ENABLED	_IOW(LTR506_IOCTL_MAGIC, 2, int *)
#define LTR506_IOCTL_ALS_ENABLE		_IOR(LTR506_IOCTL_MAGIC, 3, int *)
#define LTR506_IOCTL_ALS_GET_ENABLED	_IOW(LTR506_IOCTL_MAGIC, 4, int *)

struct ltr506_platform_data {
	/* Interrupt */
	int pfd_gpio_int_no;
	int pfd_gpio_int_wake_dev;

	/* ALS */
	int pfd_disable_als_on_suspend;
	int pfd_als_filter_interrupts;
	int pfd_als_resolution;
	int pfd_als_meas_rate;
	int pfd_als_gain;

	/* PS */
	int pfd_disable_ps_on_suspend;
	int pfd_ps_filter_interrupts;
	int pfd_ps_meas_rate;
	int pfd_ps_gain;

	/* LED */
	int pfd_led_pulse_freq;
	int pfd_led_duty_cyc;
	int pfd_led_peak_curr;
	int pfd_led_pulse_count;
};

#endif
