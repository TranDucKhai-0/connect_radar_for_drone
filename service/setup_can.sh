#!/bin/bash
echo "#####################################"
filename="/proc/device-tree/mttcan@c310000/status"

while read -r line
do
	status=$line
done < $filename


if [ "$line" != "okay" ];
then
	echo "CAN IS NOT ENABLE"
	exit 1
fi

echo "CONFIG PINMUX USING BUSYBOX"
sudo busybox devmem 0x0c303018 w 0x458
sudo busybox devmem 0x0c303010 w 0x400
echo "LOAD KERNEL MODULE"
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
echo "BRING UP VIRTUAL INTERFACE"
sudo ip link set can0 up type can bitrate 500000
echo "######################################"




