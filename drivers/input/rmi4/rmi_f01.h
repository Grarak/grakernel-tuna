/*
 * Copyright (c) 2012 Synaptics Incorporated
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


#ifndef _RMI_F01_H
#define _RMI_F01_H

#define RMI_PRODUCT_ID_LENGTH    10

/**
 * @manufacturer_id - reports the identity of the manufacturer of the RMI
 * device. Synaptics RMI devices report a Manufacturer ID of $01.
 * @custom_map - at least one custom, non
 * RMI-compatible register exists in the register address map for this device.
 * @non-compliant - the device implements a register map that is not compliant
 * with the RMI specification.
 * @has_lts - the device uses Synaptics' LTS hardware architecture.
 * @has_sensor_id - the SensorID query register (F01_RMI_Query22) exists.
 * @has_charger_input - the ChargerConnected bit (F01_RMI_Ctrl0, bit 5) is
 * meaningful.
 * @has_adjustable_doze - the doze (power management) control registers exist.
 * @has_adjustable_doze_holdoff - the doze holdoff register exists.
 * @has_product_properties - indicates the presence of F01_RMI_Query42,
 * ProductProperties2.
 * @productinfo_1 - meaning varies from product to product, consult your
 * product spec sheet.
 * @productinfo_2 - meaning varies from product to product, consult your
 * product spec sheet.
 * @year - year of manufacture MOD 2000.
 * @month - month of manufacture
 * @day - day of manufacture
 * @wafer_id1_lsb - The wafer-lot ID registers record the lot number of the
 * wafer from which the module’s touch controller was produced.
 * @wafer_id1_msb - The wafer-lot ID registers record the lot number of the
 * wafer from which the module’s touch controller was produced.
 * @wafer_id2_lsb - The wafer-lot ID registers record the lot number of the
 * wafer from which the module’s touch controller was produced.
 * @wafer_id2_msb - The wafer-lot ID registers record the lot number of the
 * wafer from which the module’s touch controller was produced.
 * @wafer_id3_lsb - The wafer-lot ID registers record the lot number of the
 * wafer from which the module’s touch controller was produced.
 */
union f01_basic_queries {
	struct {
		u8 manufacturer_id:8;

		bool custom_map:1;
		bool non_compliant:1;
		bool has_lts:1;
		bool has_sensor_id:1;
		bool has_charger_input:1;
		bool has_adjustable_doze:1;
		bool has_adjustable_doze_holdoff:1;
		bool has_product_properties_2:1;

		u8 productinfo_1:7;
		bool q2_bit_7:1;
		u8 productinfo_2:7;
		bool q3_bit_7:1;

		u8 year:5;
		u8 year_rsvd:3;
		u8 month:4;
		u8 month_rsvd:4;
		u8 day:5;
		u8 day_rsvd:3;
		bool cp1:1;
		bool cp2:1;
		u8 wafer_id1_lsb:8;
		u8 wafer_id1_msb:8;
		u8 wafer_id2_lsb:8;
		u8 wafer_id2_msb:8;
		u8 wafer_id3_lsb:8;
	} __attribute__((__packed__));
	u8 regs[11];
};

union f01_device_status {
	struct {
		u8 status_code:4;
		u8 reserved:2;
		bool flash_prog:1;
		bool unconfigured:1;
	} __attribute__((__packed__));
	u8 regs[1];
};

/* control register bits */
#define RMI_SLEEP_MODE_NORMAL (0x00)
#define RMI_SLEEP_MODE_SENSOR_SLEEP (0x01)
#define RMI_SLEEP_MODE_RESERVED0 (0x02)
#define RMI_SLEEP_MODE_RESERVED1 (0x03)

#define RMI_IS_VALID_SLEEPMODE(mode) \
	(mode >= RMI_SLEEP_MODE_NORMAL && mode <= RMI_SLEEP_MODE_RESERVED1)

/**
 * @sleep_mode - This field controls power management on the device. This
 * field affects all functions of the device together.
 * @nosleep - When set to ‘1’, this bit disables whatever sleep mode may be
 * selected by the sleep_mode field,and forces the device to run at full power
 * without sleeping.
 * @charger_input - When this bit is set to ‘1’, the touch controller employs
 * a noise-filtering algorithm designed for use with a connected battery
 * charger.
 * @report_rate - sets the report rate for the device.  The effect of this
 * setting is highly product dependent.  Check the spec sheet for your
 * particular touch sensor.
 * @configured - written by the host as an indicator that the device has been
 * successfuly configured.
 */
union f01_device_control_0 {
	struct {
		u8 sleep_mode:2;
		bool nosleep:1;
		u8 reserved:2;
		bool charger_input:1;
		bool report_rate:1;
		bool configured:1;
	} __attribute__((__packed__));
	u8 regs[1];
};

#endif
