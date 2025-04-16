# This code exists because of a kernel-level bug in NVidia's drivers
# The 2024-2025 team installed the patch for this bug and theoretically 
# removed the need for this. It is being kept just in case
VENDOR=$(../udev/info.sh /dev/rover/onBoardMega | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
PRODUCT=$(../udev/info.sh /dev/rover/onBoardMega | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
sudo usbreset $VENDOR:$PRODUCT
VENDOR=$(../udev/info.sh /dev/rover/rtk | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
PRODUCT=$(../udev/info.sh /dev/rover/rtk | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
sudo usbreset $VENDOR:$PRODUCT
VENDOR=$(../udev/info.sh /dev/rover/scienceArduinoNano | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
PRODUCT=$(../udev/info.sh /dev/rover/scienceArduinoNano | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
sudo usbreset $VENDOR:$PRODUCT
