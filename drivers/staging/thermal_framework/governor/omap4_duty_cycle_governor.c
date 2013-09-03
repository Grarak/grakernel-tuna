/*
 * Duty cycle governor
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Eugene Mandrenko <Ievgen.mandrenko@ti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/suspend.h>
#include <plat/omap_device.h>

#define NORMAL_TEMP_MONITORING_RATE 1000
#define NORMAL_MONITORING_RATE 10000
#define TEMP_THRESHOLD 1
#define INIT_SECTION -1
#define DEBUG_DUMP_ACTIVE_PROFILE 0x00000001

struct duty_governor {
	struct pcb_sens *tpcb;
	struct duty_cycle *tduty;
	struct pcb_section *tpcb_sections;
	struct pcb_section *turbo_sprint_tpcb_sections;
	int period;
	int previous_temp;
	int curr_pcb_temp;
	int previous_pcb_temp;
	bool cur_turbo_sprint;
	int working_section;
	int npcb_sections;
	int turbo_sprint_working_session;
	int turbo_sprint_npcb_sections;
	struct delayed_work duty_cycle_governor_work;
};

/* protect global data */
static DEFINE_MUTEX(mutex_duty_governor);

static struct duty_governor *t_governor;
static struct pcb_section *pcb_sections;
static int pcb_sections_size;
static struct pcb_section *turbo_sprint_pcb_sections;
static int turbo_sprint_pcb_sections_size;

static void omap4_duty_schedule(struct duty_governor *t_gov)
{
	if (!IS_ERR_OR_NULL(t_gov) &&
	    !IS_ERR_OR_NULL(t_gov->tpcb) &&
	    !IS_ERR_OR_NULL(t_gov->tduty))
		schedule_delayed_work(&t_governor->duty_cycle_governor_work,
				      msecs_to_jiffies(0));
}

int omap4_duty_pcb_register(struct pcb_sens *tpcb)
{
	mutex_lock(&mutex_duty_governor);

	if (!IS_ERR_OR_NULL(t_governor)) {
		if (t_governor->tpcb == NULL) {
			t_governor->tpcb = tpcb;
			t_governor->period = NORMAL_TEMP_MONITORING_RATE;
		} else {
			pr_err("%s:dublicate of pcb registration\n", __func__);
			mutex_unlock(&mutex_duty_governor);

			return -EBUSY;
		}
	}
	omap4_duty_schedule(t_governor);

	mutex_unlock(&mutex_duty_governor);

	return 0;
}
static bool is_threshold(struct duty_governor *tgov)
{
	int delta;

	delta = abs(tgov->previous_pcb_temp - tgov->curr_pcb_temp);

	if (delta > TEMP_THRESHOLD)
		return true;

	return false;
}

static bool has_turbo_sprint(struct duty_governor *tgov)
{
	return (tgov->tpcb && tgov->tpcb->turbo_sprint);
}

static bool turbo_sprint_changed(struct duty_governor *tgov)
{
	return (tgov->tpcb->turbo_sprint() != tgov->cur_turbo_sprint);
}

static bool has_debug(struct duty_governor *tgov)
{
	return (tgov->tpcb && tgov->tpcb->debug);
}

static int get_debug(struct duty_governor *tgov)
{
	if (has_debug(tgov)) {
		return tgov->tpcb->debug();
	}
	return 0;
}

static int omap4_duty_apply_constraint(struct duty_governor *tgov,
										struct pcb_section *t_pcb_sections)
{
	struct duty_cycle_params *tduty_params = &t_pcb_sections->tduty_params;
	int dc_enabled = t_pcb_sections->duty_cycle_enabled;
	struct duty_cycle *t_duty = tgov->tduty;
	int ret = true;

	ret = tgov->tduty->enable(false, false);

	if (ret)
		return ret;
	
	if (dc_enabled) {
		if (t_duty->update_params(tduty_params))
			return ret;
		tgov->tduty->enable(dc_enabled, true);
	}

	return ret;
}

static void dump_active_profile(struct duty_governor *tgov) {
	struct pcb_section * sections = tgov->tpcb_sections;
	int num_sections = tgov->npcb_sections;
	int working_section = tgov->working_section;

	if (tgov->cur_turbo_sprint) {
		num_sections = tgov->turbo_sprint_npcb_sections;
		sections     = tgov->turbo_sprint_tpcb_sections;
		working_section = tgov->turbo_sprint_working_session;
	}

	if ((working_section < 0) ||
			(working_section>num_sections)) {
		return;
	}

	printk("omap4_duty_cycle_governor: Active Profile\n");
	printk("omap4_duty_cycle_governor: ====>         Temperature: %d\n", sections[working_section].pcb_temp_level);
	printk("omap4_duty_cycle_governor: ====>            Max Freq: %d\n", sections[working_section].max_opp);
	printk("omap4_duty_cycle_governor: ====>          Nitro Rate: %d\n", sections[working_section].tduty_params.nitro_rate);
	printk("omap4_duty_cycle_governor: ====>        Cooling Rate: %d\n", sections[working_section].tduty_params.cooling_rate);
	printk("omap4_duty_cycle_governor: ====>      Nitro Interval: %d\n", sections[working_section].tduty_params.nitro_interval);
	printk("omap4_duty_cycle_governor: ====>    Nitro Percentage: %d\n", sections[working_section].tduty_params.nitro_percentage);
	printk("omap4_duty_cycle_governor: ====>  Duty Cycle Enabled: %d\n", sections[working_section].duty_cycle_enabled);
	printk("omap4_duty_cycle_governor: ====>        Current Temp: %d\n", tgov->curr_pcb_temp);
	printk("omap4_duty_cycle_governor: ====>Turbo Sprint Enabled: %d\n", tgov->cur_turbo_sprint);
}

static void omap4_duty_update(struct duty_governor *tgov)
{
	int sect_num;

	int num_sections = tgov->npcb_sections;
	struct pcb_section * sections = tgov->tpcb_sections;
	int * working_section = &tgov->working_section;

	if (IS_ERR_OR_NULL(tgov) ||
			IS_ERR_OR_NULL(tgov->tduty) ||
			IS_ERR_OR_NULL(tgov->tpcb_sections))
		return;

	if (has_turbo_sprint(t_governor)) {
		// Update Overdrive state and reset working section if needed
		bool turbo_sprint = tgov->tpcb->turbo_sprint();
		if (turbo_sprint != tgov->cur_turbo_sprint) {
			// Reset Working Session
			tgov->working_section = INIT_SECTION;
			tgov->turbo_sprint_working_session = INIT_SECTION;
		}
		tgov->cur_turbo_sprint = tgov->tpcb->turbo_sprint();
	}

	// override section if turbo sprint is on
	if (tgov->cur_turbo_sprint) {
		num_sections    = tgov->turbo_sprint_npcb_sections;
		sections        = tgov->turbo_sprint_tpcb_sections;
		working_section = &tgov->turbo_sprint_working_session;
	}

	for (sect_num = 0; sect_num < num_sections; sect_num++)
		if (sections[sect_num].pcb_temp_level > tgov->curr_pcb_temp)
			break;

	if (sect_num >= num_sections)
		sect_num = num_sections - 1;

	// Update working session
	if (*working_section != sect_num) {
		if (omap4_duty_apply_constraint(tgov, &sections[sect_num] ))
			tgov->previous_pcb_temp = tgov->curr_pcb_temp;
		*working_section = sect_num;
	}

	if (get_debug(tgov) & DEBUG_DUMP_ACTIVE_PROFILE)
		dump_active_profile(tgov);
}

static void omap4_duty_governor_delayed_work_fn(struct work_struct *work)
{
	mutex_lock(&mutex_duty_governor);

	if (!IS_ERR_OR_NULL(t_governor->tpcb)) {
		if (!IS_ERR_OR_NULL(t_governor->tpcb->update_temp)) {
			t_governor->curr_pcb_temp =
					t_governor->tpcb->update_temp();
			if (has_turbo_sprint(t_governor)) {
				if (is_threshold(t_governor) || turbo_sprint_changed(t_governor)) {
					omap4_duty_update(t_governor);
				}
			} else {
				if (is_threshold(t_governor))
					omap4_duty_update(t_governor);
			}
		} else {
			pr_err("%s:update_temp() isn't defined\n", __func__);
		}
	}
	schedule_delayed_work(&t_governor->duty_cycle_governor_work,
			      msecs_to_jiffies(t_governor->period));

	mutex_unlock(&mutex_duty_governor);
}

static int omap4_duty_pm_notifier_cb(struct notifier_block *notifier,
				     unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		cancel_delayed_work_sync(&t_governor->duty_cycle_governor_work);
		break;
	case PM_POST_SUSPEND:
		omap4_duty_schedule(t_governor);
		break;
	}

	return NOTIFY_DONE;
}

int omap4_duty_cycle_register(struct duty_cycle *tduty)
{
	mutex_lock(&mutex_duty_governor);

	if (!IS_ERR_OR_NULL(t_governor)) {
		if (t_governor->tduty == NULL) {
			t_governor->tduty = tduty;
		} else {
			pr_err("%s:dublicate of duty cycle registration\n",
			       __func__);
			mutex_unlock(&mutex_duty_governor);

			return -EBUSY;
		}
		/* Setup initial parameters for duty cycle */
		omap4_duty_update(t_governor);
	}

	omap4_duty_schedule(t_governor);

	mutex_unlock(&mutex_duty_governor);

	return 0;
}

void omap4_duty_pcb_section_reg(struct pcb_section *pcb_sect, int sect_size)
{
	mutex_lock(&mutex_duty_governor);

	pcb_sections = pcb_sect;
	pcb_sections_size = sect_size;

	if (!IS_ERR_OR_NULL(t_governor)) {
		t_governor->tpcb_sections = pcb_sections;
		t_governor->npcb_sections = pcb_sections_size;
		omap4_duty_update(t_governor);
	}

	mutex_unlock(&mutex_duty_governor);
}

void omap4_duty_turbo_sprint_pcb_section_reg(struct pcb_section *pcb_sect, int sect_size)
{
	turbo_sprint_pcb_sections = pcb_sect;
	turbo_sprint_pcb_sections_size = sect_size;
}

static struct notifier_block omap4_duty_pm_notifier = {
	.notifier_call = omap4_duty_pm_notifier_cb,
};

static int __init omap4_duty_governor_init(void)
{
	if (!cpu_is_omap443x())
		return 0;

	t_governor = kzalloc(sizeof(struct duty_governor), GFP_KERNEL);
	if (IS_ERR_OR_NULL(t_governor)) {
		pr_err("%s:Cannot allocate memory\n", __func__);
		return -ENOMEM;
	}
	t_governor->period = NORMAL_MONITORING_RATE;
	t_governor->curr_pcb_temp = DUTY_GOVERNOR_DEFAULT_TEMP;
	t_governor->previous_temp = DUTY_GOVERNOR_DEFAULT_TEMP;
	t_governor->tpcb_sections = pcb_sections;
	t_governor->npcb_sections = pcb_sections_size;
	t_governor->cur_turbo_sprint = false;
	t_governor->turbo_sprint_tpcb_sections = turbo_sprint_pcb_sections;
	t_governor->turbo_sprint_npcb_sections = turbo_sprint_pcb_sections_size;
	t_governor->working_section = INIT_SECTION;

	if (register_pm_notifier(&omap4_duty_pm_notifier))
		pr_err("%s:omap4_duty_gov pm registration failed!\n", __func__);

	INIT_DELAYED_WORK(&t_governor->duty_cycle_governor_work,
			  omap4_duty_governor_delayed_work_fn);

	return 0;
}

static void __exit omap4_duty_governor_exit(void)
{
	cancel_delayed_work_sync(&t_governor->duty_cycle_governor_work);
	kfree(t_governor);
}

early_initcall(omap4_duty_governor_init);
module_exit(omap4_duty_governor_exit);

MODULE_AUTHOR("Euvgen Mandrenko <ievgen.mandrenko@ti.com>");
MODULE_DESCRIPTION("OMAP on-die thermal governor");
MODULE_LICENSE("GPL");

