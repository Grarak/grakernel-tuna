#!/bin/bash

# Colorize and add text parameters
red=$(tput setaf 1) # red
cya=$(tput setaf 6) # cyan

txtbld=$(tput bold) # Bold

bldred=${txtbld}$(tput setaf 1) # red
bldcya=${txtbld}$(tput setaf 6) # cyan

txtrst=$(tput sgr0) # Reset

butterversion=exp
tcf=~/android/kernel/toolchains/

DATE_START=$(date +"%s")

echo -e "${bldcya} Select Toolchain ${txtrst}"
echo -e "> 1: Linaro 4.7.3 13.04"
echo -e "  2: Linaro 4.8.1 13.04"
read toolchain

if [ "$toolchain" == "1" ]; then
        export CROSS_COMPILE=${tcf}linaro-4.7-13.04/bin/arm-linux-androideabi-
fi

if [ "$toolchain" == "2" ]; then
        export CROSS_COMPILE=${tcf}linaro-4.8-13.04/bin/arm-linux-androideabi-
fi

echo -e "${bldcya} Cleaning .... ${txtrst}"

##########################################################################
echo -e "Do you want to clean up? [N/y]"
read cleanup

if [ "$cleanup" == "y" ]; then
        echo -e "Complete Clean? [N/y]"
        read cleanoption

        if [ "$cleanoption" == "n" ] || [ "$cleanoption" == "N" ]; then
                echo -e "${bldcya} make clean ${txtrst}"
        	make clean
        fi

        if [ "$cleanoption" == "y" ]; then
                echo -e "${bldcya} make clean mrproper ${txtrst}"
	        make clean mrproper
        fi
fi
###########################################################################

###########################################################################
if [ -e .version ]; then
	rm .version
fi

echo -e "${bldcya} Do you want to edit the kernel version? ${txtrst} [N/y]"
read kernelversion

if [ "$kernelversion" == "y" ]; then
        echo -e "${bldcya} What version has your kernel? ${txtrst}"
        echo "${bldred} NUMBERS ONLY! ${txtrst}"
        read version
 
        echo $version >> .version
fi
###########################################################################

make tuna_defconfig

###########################################################################
echo -e "${bldcya} Build Dualboot? ${txtrst} [N/y]"
read frage

if [ "$frage" == "n" ] || [ "$frage" == "N" ]; then
	echo -e "${bldcya} Build singleboot kernel ${txtrst}"
        cp arch/arm/configs/butter_tuna_defconfig .config
        sed -i s/CONFIG_LOCALVERSION=\".*\"/CONFIG_LOCALVERSION=\"-Butter_Weekly_${butterversion}_4.2.+\"/ .config
fi

if [ "$frage" == "y" ]; then
        echo -e "${bldcya} Build dualboot kernel ${txtrst}"
        cp arch/arm/configs/butter_tuna_dual_defconfig .config
        sed -i s/CONFIG_LOCALVERSION=\".*\"/CONFIG_LOCALVERSION=\"-Butter_Weekly_${butterversion}_Dual_4.2.+\"/ .config
fi
###########################################################################
echo -e "${bldcya} This could take a while .... ${txtrst}"
make -j4

###########################################################################
if [ -e arch/arm/boot/zImage ]; then

        echo -e "${bldcya} Building boot.img .... ${txtrst}"
        if [ "$frage" == "n" ] || [ "$frage" == "N" ]; then

                cp arch/arm/boot/zImage mkboot/
                cd mkboot

	        ./img.sh
                cd ..
                cp mkboot/boot.img out/ButterKernel/

                rm -rfv out/ButterKernel/system/lib/modules/*
                find -name '*.ko' -exec cp -v {} out/ButterKernel/system/lib/modules \;

                cd out/ButterKernel/META-INF/com/google/android
                sed -i s/buildscriptline/${butterversion}/ updater-script
                cd ../../../..
                zip -r ButterKernel_Weekly_${butterversion}.zip cleaner META-INF system boot.img
                cp updater-script-original META-INF/com/google/android/updater-script
        else
                cp arch/arm/boot/zImage mkbootdual/
                cd mkbootdual

	        ./img.sh
                cd ..
                cp mkbootdual/boot.img out/ButterKernelDual/

                rm -rfv out/ButterKernelDual/system/lib/modules/*
                find -name '*.ko' -exec cp -v {} out/ButterKernelDual/system/lib/modules \;

                cd out/ButterKernelDual/META-INF/com/google/android
                sed -i s/buildscriptline/${butterversion}/ updater-script
                cd ../../../..
                zip -r ButterKernel_Weekly_${butterversion}_dual.zip META-INF system boot.img
                cp updater-script-original META-INF/com/google/android/updater-script
        fi
   
        echo -e "${bldcya} Finished!! ${txtrst}"
        DATE_END=$(date +"%s")
        DIFF=$(($DATE_END - $DATE_START))
        echo "Build completed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds."
        date '+%a, %d %b %Y %H:%M:%S'
        echo -e "${bldcya} ButterKernel.zip is in /out/ButterKernel ${txtrst}"

else
	        echo "${bldred} KERNEL DID NOT BUILD! ${txtrst}"
fi

exit 0
############################################################################
