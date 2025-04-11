# udev

We use [udev](https://wiki.debian.org/udev) on the rover to assign permanent names to different devices, so that our software can easily find them.

This applies to cameras, sensors, microcontrollers, and other peripherals.

## How to write new udev rules

1. Unplug your device of interest, if it is plugged in.
1. Run [usb_finder.sh](usb_finder.sh) to see what USB devices are already plugged in.
1. Plug in the USB device of interest
1. Run [usb_finder.sh](usb_finder.sh) again and see what was added. Take note of its device file that looks like `/dev/somedevice`
1. Run [info.sh](info.sh) with the name of the device file (`$ ./info.sh /dev/somedevice`) to see all the `sysfs` attributes of that device and its parents
1. Add a new entry to [10-mars-rover.rules](./rules/10-mars-rover.rules) with the following:
    * A comment describing the device
    * A list of conditions that match some unique attributes that you saw from the output of `info.sh`. Try tu use as few attributes as possible that will uniquely identify the device.
    * Followed by a command to create a symlink that will be a permanent name for that device every time it is plugged in like `/rover/cameras/coolcamera`.
    * Refer to other rules in the file for inspiration. Here is also some [further reading on writing udev rules](http://reactivated.net/writing_udev_rules.html)
1. Run `reload.sh` (assuming that `symlink.sh` has already been run on this computer).
1. Check that there is a symlink at the location you specified with something like `$ ls /dev/rover/`
1. If the symlink isn't there, try modifying or removing conditions in your rule, and reload again to see if that works.

## Scripts

### [symlink.sh](symlink.sh)

Loads our custom udev rules for the first time

### [reload.sh](reload.sh)

Reloads udev rules

### [info.sh](info.sh)

Displays `sysfs` attributes of a device and its parents

### [usb_finder.sh](usb_finder.sh)

Lists USB devices and their ID_SERIAL attribute

