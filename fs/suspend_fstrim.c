#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/earlysuspend.h>
#include <linux/fs.h>

#define SUSPEND_TRIM_SCRIPT_PATH "/sbin/suspend_trim"

static void run_trim_helper(struct early_suspend *handler)
{
	char *argv[] = { "/system/bin/sh", SUSPEND_TRIM_SCRIPT_PATH, NULL };
	static char *envp[] = {
        	"HOME=/",
        	"TERM=linux",
        	"PATH=/sbin:/system/bin:/data", NULL };

	call_usermodehelper( argv[0], argv, envp, UMH_WAIT_PROC );
}

static struct early_suspend suspend_fstrim = {
	.suspend = run_trim_helper,
	.resume = run_trim_helper,
};

void __init suspend_fstrim_init(void)
{
	register_early_suspend(&suspend_fstrim);
}
