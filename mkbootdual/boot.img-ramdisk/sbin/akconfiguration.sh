#!/sbin/bb/busybox ash

bb="/sbin/bb/busybox"
log="/data/ak/ak-boot.log"
logbck="/data/ak/ak-boot.log.bck"

#
# Set sysctl Optimizations
#
/system/bin/set_sysctl

$bb cp -vr $log $logbck
$bb rm -rf $log

exec >>$log 2>&1

$bb date >>$log

echo""

$bb echo "KERNEL INFO ----------------------------------------"
$bb cat /proc/version
$bb echo "END ------------------------------------------------";echo""

$bb echo "LAST REBOOT ----------------------------------------"
$bb date +"Last Reboot: %d.%m.%y / %H:%m" -d @$(( $(date +%s) - $(cut -f1 -d. /proc/uptime) ));
$bb echo "END ------------------------------------------------";echo""

$bb echo "ROM VERSION ----------------------------------------"
getprop ro.modversion
getprop ro.build.description
getprop ro.build.date
getprop ro.build.display.id
getprop ro.build.id
$bb echo "END ------------------------------------------------";echo""

$bb echo "SYSCTL VM INFO ---------------------------------"
sysctl -a | grep vm;
$bb echo "END ------------------------------------------------";echo""

$bb echo "ZRAM AND SWAP INFO ---------------------------------"
$bb cat /proc/swaps;
$bb echo "END ------------------------------------------------";echo""

$bb echo "MEM INFO -------------------------------------------"
$bb free;
$bb cat /proc/meminfo;
$bb echo "END ------------------------------------------------";echo""

$bb echo "USB INFO -------------------------------------------"
$bb lsusb
$bb echo "END ------------------------------------------------";echo""

$bb echo "LSMOD ----------------------------------------------"
lsmod
$bb echo "END ------------------------------------------------";echo""

$bb echo "PARTITIONS -----------------------------------------"
mount
cat /proc/partitions
$bb echo "END ------------------------------------------------";echo""

$bb echo "LIST HW LIB ----------------------------------------"
$bb ls -l /system/lib/hw/
$bb echo "END ------------------------------------------------";echo""

$bb echo "LIST MODULE LIB ------------------------------------"
$bb ls -l /system/lib/modules/
$bb echo "END ------------------------------------------------";echo""

$bb echo "LIST INIT.D DIR ------------------------------------"
$bb ls -l /system/etc/init.d/
$bb echo "END ------------------------------------------------";echo""

$bb echo "CPU INFO -------------------------------------------"
scaling_governor=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)
$bb echo "CPU Governor: $scaling_governor"
cpuinfo_cur_freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq);
$bb echo "Cur. Freq: $cpuinfo_cur_freq"
scaling_max_freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq);
$bb echo "Max. Freq: $scaling_max_freq"
scaling_min_freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq);
$bb echo "Min. Freq: $scaling_min_freq"
total_trans=$(cat /sys/devices/system/cpu/cpu0/cpufreq/stats/total_trans);
$bb echo "Total Freq Switch: $total_trans"
$bb echo ""
$bb echo "Time In State:"
cat /sys/devices/system/cpu/cpu0/cpufreq/stats/time_in_state
for i in `cat /sys/devices/system/cpu/cpu0/cpufreq/stats/time_in_state | cut -f 2 -d ' '`; do
summe=$(($summe+$i));
done;
summe=$(($summe/100));
summe=$(($summe/60));
if [ $summe -lt 60 ]; then
$bb echo "uptime with CPU on: $summe min"
else
summe=$(($summe/60));
$bb echo "uptime with CPU on: $summe h"
fi;
$bb echo ""
$bb echo "Uptime:"
$bb uptime
$bb echo ""
$bb echo "more CPU Infos:"
cpuinfo_cur_freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq);
for i in `ls /sys/devices/system/cpu/cpu0/cpufreq/`; do
$bb echo $i;
cat /sys/devices/system/cpu/cpu0/cpufreq/$i
done;
$bb echo "END ------------------------------------------------";echo""

$bb echo "PROCESSES INFO -------------------------------------"
$bb ls -l /system/lib/modules/
top -n 1
ps -wTl
$bb echo "END ------------------------------------------------";echo""

$bb echo "DMESG ----------------------------------------------"
dmesg
$bb echo "END ------------------------------------------------";echo""

exit
