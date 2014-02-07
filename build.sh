#!/bin/bash

# Colorize and add text parameters
red=$(tput setaf 1) # red
cya=$(tput setaf 6) # cyan

txtbld=$(tput bold) # Bold

bldred=${txtbld}$(tput setaf 1) # red
bldcya=${txtbld}$(tput setaf 6) # cyan

txtrst=$(tput sgr0) # Reset

version=0.9

DATE_START=$(date +"%s")

##########################################################################
echo -e "${bldcya}Do you want to clean up? ${txtrst} [N/y]"
read cleanup

if [ "$cleanup" == "y" ]; then
        echo -e "Complete Clean? [N/y]"
        read cleanoption

        if [ "$cleanoption" == "n" ] || [ "$cleanoption" == "N" ]; then
                echo -e "${bldcya}make clean ${txtrst}"
        	make clean
        fi

        if [ "$cleanoption" == "y" ]; then
                echo -e "${bldcya}make clean mrproper ${txtrst}"
	        make clean mrproper
        fi
fi
###########################################################################
if [ -e .version ]; then
	rm .version
fi

echo -e "${bldcya}Do you want to edit the kernel version? ${txtrst} [N/y]"
read kernelversion

if [ "$kernelversion" == "y" ]; then
        echo -e "${bldcya}What version has your kernel? ${txtrst}"
        echo "${bldred}NUMBERS ONLY! ${txtrst}"
        read number
 
        echo $number >> .version
fi
###########################################################################

make tuna_defconfig

cp arch/arm/configs/gk_tuna_defconfig .config
sed -i s/CONFIG_LOCALVERSION=\".*\"/CONFIG_LOCALVERSION=\"-GraKernel_${version}\"/ .config

###########################################################################
echo -e "${bldcya}This could take a while .... ${txtrst}"

nice -n 10 make modules -j4 ARCH=arm
nice -n 10 make -j4 ARCH=arm

###########################################################################
if [ -e arch/arm/boot/zImage ]; then

	if [ -d romswitcher ]; then
		cd romswitcher
		git pull
		cd ..
	else
		git clone git@github.com:RomSwitchers/RomSwitcher-tuna.git -b master romswitcher
	fi

	find -name "zImage" -exec cp -vf {} romswitcher/ \;
	find -name "*.ko" -exec cp -vf {} romswitcher/boot.img-ramdisk/lib/modules/ \;

	cd romswitcher

	./build.sh
   
        echo -e "${bldcya} Finished!! ${txtrst}"
        DATE_END=$(date +"%s")
        DIFF=$(($DATE_END - $DATE_START))
        echo "Build completed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds."
        date '+%a, %d %b %Y %H:%M:%S'

else
	echo "${bldred} KERNEL DID NOT BUILD! ${txtrst}"
fi

exit 0
############################################################################
