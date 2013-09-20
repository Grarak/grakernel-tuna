/*
 * Board Notle IO wakeup identification and statistics.
 *
 * Copyright (c) 2013 Google, Inc.
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "board-notle.h"

struct omap_pin_stat {
	const char *name;
	const char *use;
	bool default_display;

	unsigned long long count;
};

struct omap_irq_stat {
	char name[32];
	unsigned long long count;
};

#define OMAP_IO_STAT(_index, _name, _use, _display) \
		[(_index)] = { .name = (_name), .use = (_use), .default_display = (_display), }

#define OMAP_WK_STAT(_index, _name, _use, _display) \
		[(_index)] = { .name = (_name), .use = (_use), .default_display = (_display), }

static struct omap_irq_stat notle_irq_stats[1024];
#define NUM_IRQ_STATS (sizeof(notle_irq_stats) / sizeof(notle_irq_stats[0]))

/*
 * Pins corresponding to GPIO blocks 0-6. These are documented in the OMAP44xx TRM
 * under Table 18-527. CONTROL_PADCONF_WAKEUPEVENT_0 and subsequent sections. Signals
 * are named here based on mux config settings, but the TRM identifies by primary function.
 * Use the elton schematic to resolve these aliases.
 */
static struct omap_pin_stat notle_io_stats[32 * 6] = {
	OMAP_IO_STAT(0*32 +  0, "sdmmc2_dat0", "Unknown", false),
	OMAP_IO_STAT(0*32 +  1, "sdmmc2_dat1", "Unknown", false),
	OMAP_IO_STAT(0*32 +  2, "sdmmc2_dat2", "Unknown", false),
	OMAP_IO_STAT(0*32 +  3, "sdmmc2_dat3", "Unknown", false),
	OMAP_IO_STAT(0*32 +  4, "sdmmc2_dat4", "Unknown", false),
	OMAP_IO_STAT(0*32 +  5, "sdmmc2_dat5", "Unknown", false),
	OMAP_IO_STAT(0*32 +  6, "sdmmc2_dat6", "Unknown", false),
	OMAP_IO_STAT(0*32 +  7, "sdmmc2_dat7", "Unknown", false),
	OMAP_IO_STAT(0*32 +  8, "gpio_32", "Not Connected", false),
	OMAP_IO_STAT(0*32 +  9, "gpio_33", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 10, "gpio_34", "Board ID0", false),
	OMAP_IO_STAT(0*32 + 11, "gpio_35", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 12, "gpio_36", "BT_WAKE", true),
	OMAP_IO_STAT(0*32 + 13, "gpio_37", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 14, "gpio_38", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 15, "gpio_39", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 16, "gpio_40", "Board ID1", false),
	OMAP_IO_STAT(0*32 + 17, "gpio_41", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 18, "gpio_42", "Board ID2", false),
	OMAP_IO_STAT(0*32 + 19, "gpio_154", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 20, "gpio_44", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 21, "gpio_45", "USB_MUX_CB1", false),
	OMAP_IO_STAT(0*32 + 22, "gpio_46", "USB_MUX_CB0", false),
	OMAP_IO_STAT(0*32 + 23, "gpio_152", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 24, "gpio_48", "WL_BT_REG_ON", false),
	OMAP_IO_STAT(0*32 + 25, "gpio_49", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 26, "gpio_50", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 27, "gpio_51", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 28, "gpio_52", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 29, "gpio_53", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 30, "gpio_54", "Not Connected", false),
	OMAP_IO_STAT(0*32 + 31, "gpio_55", "Not Connected", false),

	OMAP_IO_STAT(1*32 +  0, "gpio_56", "Not Connected", false),
	OMAP_IO_STAT(1*32 +  1, "sdmmc2_clk", "Unknown", false),
	OMAP_IO_STAT(1*32 +  2, "sdmmc2_cmd", "Unknown", false),
	OMAP_IO_STAT(1*32 +  3, "gpio_59", "Not Connected", false),
	OMAP_IO_STAT(1*32 +  4, "gpio_60", "Not Connected", false),
	OMAP_IO_STAT(1*32 +  5, "gpio_61", "Not Connected", false),
	OMAP_IO_STAT(1*32 +  6, "gpio_62", "Not Connected", false),

	OMAP_IO_STAT(1*32 + 12, "gpio_63?", "Not Connected", false),
	OMAP_IO_STAT(1*32 + 13, "gpio_64?", "Not Connected", false),
	OMAP_IO_STAT(1*32 + 14, "gpio_65?", "Not Connected", false),
	OMAP_IO_STAT(1*32 + 15, "gpio_66?", "Not Connected", false),
	OMAP_IO_STAT(1*32 + 16, "csi21_dx0", "Unknown", false),
	OMAP_IO_STAT(1*32 + 17, "csi21_dy0", "Unknown", false),
	OMAP_IO_STAT(1*32 + 18, "csi21_dx1", "Unknown", false),
	OMAP_IO_STAT(1*32 + 19, "csi21_dy1", "Unknown", false),
	OMAP_IO_STAT(1*32 + 20, "csi21_dx2", "Unknown", false),
	OMAP_IO_STAT(1*32 + 21, "csi21_dy2", "Unknown", false),
	OMAP_IO_STAT(1*32 + 22, "csi21_dx3", "Unknown", false),
	OMAP_IO_STAT(1*32 + 23, "csi21_dy3", "Unknown", false),
	OMAP_IO_STAT(1*32 + 24, "csi21_dx4", "Unknown", false),
	OMAP_IO_STAT(1*32 + 25, "csi21_dy4", "Unknown", false),
	OMAP_IO_STAT(1*32 + 26, "csi22_dx0", "Unknown", false),
	OMAP_IO_STAT(1*32 + 27, "csi22_dy0", "Unknown", false),
	OMAP_IO_STAT(1*32 + 28, "csi22_dx1", "Unknown", false),
	OMAP_IO_STAT(1*32 + 29, "csi22_dy1", "Unknown", false),
	OMAP_IO_STAT(1*32 + 30, "cam_shutter", "Unknown", false),
	OMAP_IO_STAT(1*32 + 31, "cam_strobe", "Unknown", false),

	OMAP_IO_STAT(2*32 +  0, "gpio_83", "Not Connected", false),
	OMAP_IO_STAT(2*32 +  1, "gpio_84", "Not Connected", false),
	OMAP_IO_STAT(2*32 +  2, "gpio_85", "FPGA_CDONE", false),
	OMAP_IO_STAT(2*32 +  3, "gpio_86", "Not Connected", false),
	OMAP_IO_STAT(2*32 +  4, "gpio_87", "FPGA_CRESET_B", false),
	OMAP_IO_STAT(2*32 +  5, "usbb1_ulpiphy_dat0", "Unknown", false),
	OMAP_IO_STAT(2*32 +  6, "usbb1_ulpiphy_dat1", "Unknown", false),
	OMAP_IO_STAT(2*32 +  7, "usbb1_ulpiphy_dat2", "Unknown", false),
	OMAP_IO_STAT(2*32 +  8, "usbb1_ulpiphy_dat3", "Unknown", false),
	OMAP_IO_STAT(2*32 +  9, "usbb1_ulpiphy_dat4", "Unknown", false),
	OMAP_IO_STAT(2*32 + 10, "usbb1_ulpiphy_dat5", "Unknown", false),
	OMAP_IO_STAT(2*32 + 11, "usbb1_ulpiphy_dat6", "Unknown", false),
	OMAP_IO_STAT(2*32 + 12, "usbb1_ulpiphy_dat7", "Unknown", false),
	OMAP_IO_STAT(2*32 + 13, "gpio_96", "Not Connected", false),
	OMAP_IO_STAT(2*32 + 14, "usbb1_hsic_strobe", "Unknown", false),
	OMAP_IO_STAT(2*32 + 15, "usbc1_icusb_dp", "Unknown", false),
	OMAP_IO_STAT(2*32 + 16, "usbc1_icusb_dm", "Unknown", false),
	OMAP_IO_STAT(2*32 + 17, "sdmmc1_clk", "Unknown", false),
	OMAP_IO_STAT(2*32 + 18, "sdmmc1_cmd", "Unknown", false),
	OMAP_IO_STAT(2*32 + 19, "sdmmc1_dat0", "Unknown", false),
	OMAP_IO_STAT(2*32 + 20, "sdmmc1_dat1", "Unknown", false),
	OMAP_IO_STAT(2*32 + 21, "sdmmc1_dat2", "Unknown", false),
	OMAP_IO_STAT(2*32 + 22, "sdmmc1_dat3", "Unknown", false),
	OMAP_IO_STAT(2*32 + 23, "sdmmc1_dat4", "Unknown", false),
	OMAP_IO_STAT(2*32 + 24, "sdmmc1_dat5", "Unknown", false),
	OMAP_IO_STAT(2*32 + 25, "sdmmc1_dat6", "Unknown", false),
	OMAP_IO_STAT(2*32 + 26, "sdmmc1_dat7", "Unknown", false),
	OMAP_IO_STAT(2*32 + 27, "abe_mcbsp2_clkx", "Unknown", false),
	OMAP_IO_STAT(2*32 + 28, "abe_mcbsp2_dr", "Unknown", false),
	OMAP_IO_STAT(2*32 + 29, "abe_mcbsp2_dx", "Unknown", false),
	OMAP_IO_STAT(2*32 + 30, "abe_mcbsp2_fsx", "Unknown", false),
	OMAP_IO_STAT(2*32 + 31, "abe_mcbsp1_clock", "Unknown", false),

	OMAP_IO_STAT(4*32 +  0, "uart3_tx", "console", true),
	OMAP_IO_STAT(4*32 +  1, "uart3_rts_sd", "console", true),
	OMAP_IO_STAT(4*32 +  2, "uart3_rx", "console", true),
	OMAP_IO_STAT(4*32 +  3, "uart3_tx", "console", true),
	OMAP_IO_STAT(4*32 +  4, "sdmmc5_clk", "Unknown", false),
	OMAP_IO_STAT(4*32 +  5, "sdmmc5_cmd", "Unknown", false),
	OMAP_IO_STAT(4*32 +  6, "sdmmc5_dat0", "Unknown", false),
	OMAP_IO_STAT(4*32 +  7, "sdmmc5_dat1", "Unknown", false),
	OMAP_IO_STAT(4*32 +  8, "sdmmc5_dat2", "Unknown", false),
	OMAP_IO_STAT(4*32 +  9, "sdmmc5_dat3", "Unknown", false),
	OMAP_IO_STAT(4*32 + 10, "mcspi4_clk", "Unknown", false),
	OMAP_IO_STAT(4*32 + 11, "mcspi4_simo", "Unknown", false),
	OMAP_IO_STAT(4*32 + 12, "mcspi4_somi", "Unknown", false),
	OMAP_IO_STAT(4*32 + 13, "mcspi4_cs0", "Unknown", false),
	OMAP_IO_STAT(4*32 + 14, "uart4_rx", "GPS", true),
	OMAP_IO_STAT(4*32 + 15, "uart4_tx", "GPS", true),
	OMAP_IO_STAT(4*32 + 16, "gpio_157", "Not Connected", false),
	OMAP_IO_STAT(4*32 + 17, "dispc2_data23", "Unknown", false),
	OMAP_IO_STAT(4*32 + 18, "dispc2_data22", "Unknown", false),
	OMAP_IO_STAT(4*32 + 19, "dispc2_data21", "Unknown", false),
	OMAP_IO_STAT(4*32 + 20, "dispc2_data20", "Unknown", false),
	OMAP_IO_STAT(4*32 + 21, "dispc2_data19", "Unknown", false),
	OMAP_IO_STAT(4*32 + 22, "dispc2_data18", "Unknown", false),
	OMAP_IO_STAT(4*32 + 23, "dispc2_data15", "Unknown", false),
	OMAP_IO_STAT(4*32 + 24, "dispc2_data14", "Unknown", false),
	OMAP_IO_STAT(4*32 + 25, "dispc2_data13", "Unknown", false),
	OMAP_IO_STAT(4*32 + 26, "dispc2_data12", "Unknown", false),
	OMAP_IO_STAT(4*32 + 27, "dispc2_data11", "Unknown", false),
	OMAP_IO_STAT(4*32 + 28, "gpio_169", "Not Connected", false),
	OMAP_IO_STAT(4*32 + 29, "gpio_170", "Not Connected", false),
	OMAP_IO_STAT(4*32 + 30, "gpio_171", "Not Connected", false),

	OMAP_IO_STAT(5*32 + 23, "gpio_13", "SOC_INT", true),
};
#define NUM_IO_STATS (sizeof(notle_io_stats) / sizeof(notle_io_stats[0]))

static struct omap_pin_stat notle_wk_stats[32] = {
	OMAP_WK_STAT(0, "gpio_wk0", "WLAN_HOST_WAKE", true),
	OMAP_WK_STAT(1, "gpio_wk1", "BLINK_INT", true),
	OMAP_WK_STAT(2, "gpio_wk2", "BT_HOST_WAKE", true),
	OMAP_WK_STAT(3, "gpio_wk3", "TP_INT_N", true),
	OMAP_WK_STAT(4, "gpio_wk4", "MPU9000_INT", true),
	OMAP_WK_STAT(5, "sr_scl", "Unknown", false),
	OMAP_WK_STAT(6, "sr_sda", "Unknown", false),
	OMAP_WK_STAT(7, "fref_clk_ioreq", "Unknown", false),
	OMAP_WK_STAT(8, "sys_drm_msecure", "Unknown", false),
	OMAP_WK_STAT(9, "gpio_wk30", "TOP_SW", true),
	OMAP_WK_STAT(10, "gpio_wk31", "BAT_LOW", true),
	OMAP_WK_STAT(11, "gpio_wk7", "FPGA:PIO1_C9", false),
	OMAP_WK_STAT(12, "gpio_wk8", "FPGA:GBIN2_PIO1_D8", false),
	OMAP_WK_STAT(13, "sys_32k", "Unknown", false),
	OMAP_WK_STAT(14, "sys_nreswarm", "Unknown", false),
	OMAP_WK_STAT(15, "sys_pwr_req", "Unknown", false),
	OMAP_WK_STAT(16, "gpio_wk29", "FPGA_CBSEL0", false),
	OMAP_WK_STAT(17, "gpio_wk9", "Unknown", false),
	OMAP_WK_STAT(18, "gpio_wk10", "Unknown", false),
};
#define NUM_WK_STATS (sizeof(notle_wk_stats) / sizeof(notle_wk_stats[0]))

const char *omap_board_io_name(int index)
{
	if (index < 0 || index >= NUM_IO_STATS)
		return "Invalid IO pad index";

	if (notle_io_stats[index].name)
		return notle_io_stats[index].name;
	else
		return "Unknown";
}

const char *omap_board_io_use(int index)
{
	if (index < 0 || index >= NUM_IO_STATS)
		return "Invalid IO pad index";

	if (notle_io_stats[index].use)
		return notle_io_stats[index].use;
	else
		return "Unknown";
}

const char *omap_board_wk_name(int index)
{
	if (index < 0 || index >= NUM_WK_STATS)
		return "Invalid WK pin index";

	if (notle_wk_stats[index].name)
		return notle_wk_stats[index].name;
	else
		return "Unknown";
}

const char *omap_board_wk_use(int index)
{
	if (index < 0 || index >= NUM_WK_STATS)
		return "Invalid WK pin index";

	if (notle_wk_stats[index].use)
		return notle_wk_stats[index].use;
	else
		return "Unknown";
}

void omap_board_io_event(int index)
{
	if (index < 0 || index >= NUM_IO_STATS)
		return;

	notle_io_stats[index].count++;
}

void omap_board_wk_event(int index)
{
	if (index < 0 || index >= NUM_WK_STATS)
		return;

	notle_wk_stats[index].count++;
}

void omap_board_gpio_event(int index)
{
	omap_board_io_event(index);
}

void omap_board_irq_event(int index, const char *name)
{
	if (index < 0 || index >= NUM_IRQ_STATS)
		return;

	if (name && notle_irq_stats[index].name[0] == 0)
		strlcpy(notle_irq_stats[index].name, name, sizeof(notle_irq_stats[index].name));

	notle_irq_stats[index].count++;
}

static void *pin_for_index(unsigned int index)
{
	if (index >= NUM_WK_STATS + NUM_IO_STATS + NUM_IRQ_STATS)
		return NULL;
	else if (index < NUM_IO_STATS)
		return &notle_io_stats[index];
	else if (index < NUM_IO_STATS + NUM_WK_STATS)
		return &notle_wk_stats[index - NUM_IO_STATS];
	else
		return &notle_irq_stats[index - (NUM_IO_STATS + NUM_WK_STATS)];
}

static void *wakeups_start(struct seq_file *m, loff_t *pos)
{
	return *pos ? pin_for_index(*pos - 1) : SEQ_START_TOKEN;
}

static void wakeups_stop(struct seq_file *m, void *v)
{
}

static void *wakeups_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;

	if (v == SEQ_START_TOKEN)
		return pin_for_index(0);
	else
		return pin_for_index(*pos - 1);
}

static int wakeups_show(struct seq_file *m, void *v)
{
	if (v == SEQ_START_TOKEN) {
		seq_puts(m, "pin            use            count\n");
	} else {
		if ((struct omap_irq_stat *) v >= notle_irq_stats &&
				(struct omap_irq_stat *) v <= &notle_irq_stats[NUM_IRQ_STATS - 1]) {
			struct omap_irq_stat *s = v;

			if (s->count)
				seq_printf(m, "%-16s IRQ %-11d %10llu\n",
						s->name[0] ? s->name : "Unknown",
						s - notle_irq_stats, s->count);
		} else {
			struct omap_pin_stat *s = v;

			if (s->name != NULL && (s->count || s->default_display))
				seq_printf(m, "%-16s %-16s %10llu\n", s->name, s->use, s->count);
		}
	}

	return 0;
}

static struct seq_operations wakeups_ops = {
	.start = wakeups_start,
	.next = wakeups_next,
	.stop = wakeups_stop,
	.show = wakeups_show,
};

static int wakeups_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &wakeups_ops);
}

static struct file_operations proc_wakeups_operations = {
	.open = wakeups_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int __init proc_wakeups_init(void)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry("wakeups", 0, NULL);
	if (entry)
		entry->proc_fops = &proc_wakeups_operations;

	return 0;
}
module_init(proc_wakeups_init);

