/*
 * Header for Notle panel driver
 *
 * Copyright (C) 2010 Canonical Ltd.
 * Author: John Hawley <madsci@google.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OMAP_PANEL_NOTLE_H
#define __OMAP_PANEL_NOTLE_H

struct omap_dss_device;

/**
 * struct panel_notle_data - panel driver configuration data
 * @name: panel name
 * @platform_enable: platform specific panel enable function
 * @platform_disable: platform specific panel disable function
 */
struct panel_notle_data {
        struct omap_dss_device *dssdev;
        int (*platform_enable)(struct omap_dss_device *dssdev);
        void (*platform_disable)(struct omap_dss_device *dssdev);
        int (*panel_enable)(void);
        void (*panel_disable)(void);
        int notle_version;
        int red_max_mw;
        int green_max_mw;
        int blue_max_mw;
        int limit_mw;
        unsigned red_percent;
        unsigned green_percent;
        unsigned blue_percent;
        struct omap_dss_cpr_coefs cpr_coefs;
        bool cpr_enable;
        u32 *gamma_table;
        bool gamma_enable;
        int gpio_fpga_cdone;
        int gpio_fpga_creset_b;
};

#endif /* __OMAP_PANEL_NOTLE_H */
