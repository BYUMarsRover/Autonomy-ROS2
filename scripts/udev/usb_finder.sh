#!/bin/bash

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
	syspath="${sysdevpath%/dev}"
	devname="$(udevadm info -q name -p $syspath)"
	[[ "$devname" == "bus/"* ]] && continue
	eval "$(udevadm info -q property -p $syspath | grep ID_SERIAL)"
	[[ -z "$ID_SERIAL" ]] && continue
	echo "/dev/$devname - $ID_SERIAL"
    )
done

# This is the command to query all the info about a device.
# Run this command when needing to create new udev rules,
# changing the 002/005 to the appropiate Bus # and Device #
# (which you can find by running 'lsusb').
# udevadm info -a -p $(udevadm info -q path -n /dev/bus/usb/002/005)
