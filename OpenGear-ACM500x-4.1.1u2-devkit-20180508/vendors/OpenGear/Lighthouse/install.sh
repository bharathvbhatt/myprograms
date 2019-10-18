#!/bin/bash
if ! netflash -ibk /var/mnt/usb/lighthouse-hw.bin; then
	echo "    ERROR: netflash failed."
	exit 0
fi

echo "**************************"
echo "    NETFLASH COMPLETED"
echo "**************************"
echo
echo "Please remove USB stick and hit enter to reboot"

read

reboot
