#!/sbin/sh

# clean modules
rm -rf /system/modules
rm -rf /system/lib/modules

# clean boot sound
rm -f /system/media/audio/poweron/PowerOn.*
rm -f /system/media/audio/poweron/poweron.*

# clean CMPlus stuff
rm -f /system/etc/cmplus.conf
rm -f /system/bin/cmplus
rm -f /system/bin/list
rm -f /sdcard/cmplus.conf

# clean Popcorn stuff
rm -f /system/etc/sysctl.conf
rm -rf /data/cron

# clean LeanKernelTweaks stuff
rm -f /system/xbin/installbb
rm -f /system/xbin/leantweaks
rm -rf /data/leantweaks

# clean Trinity stuff
rm -f /system/bin/fix_permissions

# clean init.d scripts
INITD_DIR=/system/etc/init.d
# GLaDOS
rm -f $INITD_DIR/90logger
rm -f $INITD_DIR/99nexusinit
rm -f $INITD_DIR/placeholder
# Trinity
rm -f $INITD_DIR/95dimmers
rm -f $INITD_DIR/98tweak
rm -f $INITD_DIR/99complete
# SG
rm -f $INITD_DIR/98_startup_script
rm -f $INITD_DIR/99_startup_complete
# Air
rm -f $INITD_DIR/89airtweaks
rm -f $INITD_DIR/98airtweak
rm -f $INITD_DIR/98airtweaks
rm -f $INITD_DIR/99airtweaks
# CMPlus
rm -f $INITD_DIR/98cmplus
# CM Default
rm -f $INITD_DIR/00banner
rm -f $INITD_DIR/90userinit
# franco
rm -f $INITD_DIR/13overclock
rm -f $INITD_DIR/00turtle
# Popcorn
rm -f $INITD_DIR/99sonic
# Lean
rm -f $INITD_DIR/99leantweaks
# Jame Bond
rm -f $INITD_DIR/007tweaks
# miscellany
rm -f $INITD_DIR/99nstools

# clean kernel setting app shared_prefs
rm -rf /data/data/mobi.cyann.nstools/shared_prefs
rm -rf /data/data/aperture.ezekeel.gladoscontrol/shared_prefs
rm -rf /data/data/com.derkernel.tkt/shared_prefs
rm -rf /data/data/com.franco.kernel/shared_prefs
rm -rf /data/data/com.liquid.control/shared_prefs
