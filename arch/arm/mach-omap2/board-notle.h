/*
 * Copyright (C) 2011 Google, Inc.
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

// Right now this file exists to coordinate between board-notle.c and
// board-notle-wifi.c

#ifndef _MACH_OMAP2_BOARD_NOTLE_H_
#define _MACH_OMAP2_BOARD_NOTLE_H_

#include <linux/serial_core.h>
#include "mux.h"

// Needed for BoardID checking
#define MUX(x) OMAP4_CTRL_MODULE_PAD_##x##_OFFSET
#define CORE_BASE_ADDR 0xfc100000

// TODO: Figure out why the wlan_wake and wlan_host_wake gpio's
// are swapped compared to what the hardware docs imply them
// to be.

// GPIO settings
// Board ID pins same across all versions
#define GPIO_BOARD_ID0                  34
#define GPIO_BOARD_ID1                  40
#define GPIO_BOARD_ID2                  42
#define GPIO_BOARD_ID0_NAME             "gpmc_ad10.gpio_34"
#define GPIO_BOARD_ID1_NAME             "gpmc_a16.gpio_40"
#define GPIO_BOARD_ID2_NAME             "gpmc_a18.gpio_42"
#define MUX_ID0                         MUX(GPMC_AD10)
#define MUX_ID1                         MUX(GPMC_A16)
#define MUX_ID2                         MUX(GPMC_A18)

// For GPIOs that are different between boards
enum {
    GPIO_MPU9000_INT_TIMER_INDEX = 0,
    GPIO_MPU9000_INT_INDEX,
    GPIO_USB_MUX_CB0_INDEX,
    GPIO_USB_MUX_CB1_INDEX,
    GPIO_GPS_ON_OFF_INDEX,
    GPIO_GPS_RESET_N_INDEX,
    GPIO_GPS_AWAKE_INDEX,
    GPIO_LCD_RST_N_INDEX,
    GPIO_DISP_ENB_INDEX,
    GPIO_FPGA_CDONE_INDEX,
    GPIO_FPGA_CRESET_B_INDEX,
    GPIO_CAM_PWDN_INDEX,
    GPIO_CAMERA_INDEX,
    GPIO_TOUCHPAD_INT_N_INDEX,
    GPIO_PROX_INT_INDEX,
    GPIO_BT_RST_N_INDEX,
    GPIO_BCM_BT_HOST_WAKE_INDEX,
    GPIO_BCM_WLAN_HOST_WAKE_INDEX,
    GPIO_BLINK_INT_INDEX,
    GPIO_SOC_INT_INDEX,
    GPIO_BAT_LOW_INDEX,
    GPIO_MAX_INDEX
};

#define GPIO_MPU9000_INT_TIMER_EVT1     92
#define GPIO_MPU9000_INT_EVT1           95
#define GPIO_MPU9000_INT_EVT2           4
#define GPIO_USB_MUX_CB1                45
#define GPIO_USB_MUX_CB0_EVT1           44
#define GPIO_USB_MUX_CB0_EVT2           46
#define GPIO_GPS_ON_OFF_EVT1            49
#define GPIO_GPS_ON_OFF_EVT2            139
#define GPIO_GPS_RESET_N_EVT1           52
#define GPIO_GPS_RESET_N_EVT2           140
#define GPIO_GPS_AWAKE_EVT1             41
#define GPIO_GPS_AWAKE_EVT2             138
#define GPIO_LCD_RST_N_EVT1             53
#define GPIO_LCD_RST_N_EVT2             94
#define GPIO_DISP_ENB                   84
#define GPIO_FPGA_CDONE                 85
#define GPIO_FPGA_CRESET_B              87
#define GPIO_BT_RST_N_EVT1              151
#define GPIO_BT_RST_N_EVT2              113
#define GPIO_CAM_PWDN_EVT1              91
#define GPIO_CAM_PWDN_EVT2              100
#define GPIO_CAMERA                     30  /* gpio_wk30 */
#define GPIO_TOUCHPAD_INT_N_EVT1        32
#define GPIO_TOUCHPAD_INT_N_EVT2        3
#define GPIO_PROX_INT_EVT1              90
#define GPIO_PROX_INT_EVT2              103
#define GPIO_AUDIO_POWERON              127
#define GPIO_BCM_WLAN_HOST_WAKE_EVT     0
#define GPIO_BCM_WLAN_HOST_WAKE_HOG     97
#define GPIO_BCM_WLAN_WAKE              170
#define GPIO_WL_RST_N                   43
#define GPIO_WL_BT_REG_ON               48
#define GPIO_BCM_BT_WAKE                36
#define GPIO_BT_RST_N_EVT1              151
#define GPIO_BT_RST_N_EVT2              113
#define GPIO_BCM_BT_HOST_WAKE_EVT1      154
#define GPIO_BCM_BT_HOST_WAKE_EVT2      2
#define GPIO_BLINK_INT_EVT2             1
#define GPIO_SOC_INT_EVT2               13
#define GPIO_BAT_LOW_EVT2               31

typedef enum {
        UNVERSIONED = 7,
        V1_EVT1     = 1,
        V1_EVT2     = 2,
        V1_EVT3     = 3,
        V1_DVT1     = 4,
        V1_5_PROTO  = 5,
        SUPPORTED_FROM = V1_EVT1,
        SUPPORTED_TO   = V1_5_PROTO,
} notle_version;

int notle_version_before( notle_version then );
int notle_version_after( notle_version then );
int notle_version_supported(void);

extern struct mmc_platform_data tuna_wifi_data;
// Elton V6 uses GPIO_WL_RST_N to control wifi power; previous versions use
// GPIO_WL_BT_REG_ON.
int notle_wlan_init(void);
void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport);
int notle_get_gpio(int);

#endif // _MACH_OMAP2_BOARD_NOTLE_H_
