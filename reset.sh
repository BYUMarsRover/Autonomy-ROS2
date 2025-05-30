#!/bin/bash
# Created by Nelson Durrant, Apr 2025
# 
# Interactively resets the USB devices on the rover

read -p "Do you want to reset the Arduino Mega? (y/n): " answer
if [[ $answer == "y" || $answer == "Y" ]]; then
    VENDOR=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/onBoardMega) | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
    PRODUCT=$(udevadm info --attribute-walk --path $(udevadm info --query path --name $1/dev/rover/onBoardMega) | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
    sudo usbreset $VENDOR:$PRODUCT
fi

read -p "Do you want to reset the Arduino Nano? (y/n): " answer
if [[ $answer == "y" || $answer == "Y" ]]; then
    VENDOR=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/peripheralsBoard) | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
    PRODUCT=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/peripheralsBoard) | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
    sudo usbreset $VENDOR:$PRODUCT
fi

read -p "Do you want to reset the RTK GPS? (y/n): " answer
if [[ $answer == "y" || $answer == "Y" ]]; then
    VENDOR=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/rtk) | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
    PRODUCT=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/rtk) | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
    sudo usbreset $VENDOR:$PRODUCT
fi

read -p "Do you want to reset the ZED camera? (y/n): " answer
if [[ $answer == "y" || $answer == "Y" ]]; then
    VENDOR=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/cameras/ZED_front) | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
    PRODUCT=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/cameras/ZED_front) | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
    sudo usbreset $VENDOR:$PRODUCT
fi

read -p "Do you want to reset the webcam? (y/n): " answer
if [[ $answer == "y" || $answer == "Y" ]]; then
    VENDOR=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/cameras/autonomyWebCam) | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
    PRODUCT=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/cameras/autonomyWebCam) | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
    sudo usbreset $VENDOR:$PRODUCT
fi

