#!/bin/bash

usage() {
  echo "Usage:
          rover-launch-usb-cameras.sh
        
        Launch four-camera feed w/ cameras connected to USB hubs.

        Options:
            -a, --address <client-ip-address>     Address for base station
            -v, --verbose                         Print gst commands
            -b, --background                      Run in background. Used in launch script

        Examples:
          ./rover-launch-usb-cameras.sh -a 127.0.0.1;
          ./rover-launch-usb-cameras.sh -a 192.168.1.65;
          "
  exit 2;
}

_V=0

BLUE=$(tput setaf 4)
NORMAL=$(tput sgr0)

function log () {
    if [[ $_V -eq 1 ]]; then
        printf "${BLUE}LOG:\n$@${NORMAL}"
    fi
}

PARSED_ARGUMENTS=$(getopt \
  --name RoverCamera.sh \
  --options a:vbh \
  --longoptions address:,verbose,background,help \
  -- "$@")
INVALID_ARGUMENTS=$?
if [ "$INVALID_ARGUMENTS" != "0" ]; then
  usage
fi

GRAYSCALE=0

###############################################################################
# Parse arguments
###############################################################################
eval set -- "$PARSED_ARGUMENTS"
while :
do
  case "$1" in
    -a | --address)
      if [[ $2 =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
        ADDRESS=$2
      else
        echo "Invalid address '$2'"
        usage
      fi
      shift 2;;
    -v | --verbose)
      _V=1
      shift ;;
    -b | --background)
      BACKGROUND_FLAG=1
      shift ;;
    -h | --help)
      usage ;;
    --) shift; break ;;
    *) echo "Unexpected option: $1 -- this should not happen."
      usage ;;
  esac
done

[ -z "$ADDRESS" ] && echo "You must specify an address" && usage;

FORMAT="I420"
BITRATE=1000
if [ "$(uname -m)" = "x86_64" ]; then
  H265_ENCODE="videoconvert ! x265enc bitrate=$BITRATE speed-preset=1 tune=4"
else
  H265_ENCODE="nvvidconv ! nvv4l2h265enc bitrate=$BITRATE preset-level=0"
fi


# Port for USB cameras
CAMERA_PORTS=(5010 5020)

###############################################################################
# Prepare pipelines for cameras connected to USB hubs
###############################################################################
ANKER_CAM_PIPELINES=()
WIDTH=640
HEIGHT=480
for i in {0..1}
do
  if [ -e "/dev/rover/cameras/ankerHubVideo$i" ]
  then
    ANKER_CAM_PIPELINES[$i]="v4l2src device=/dev/rover/cameras/ankerHubVideo$i ! videoconvert ! video/x-raw, width=$WIDTH, height=$HEIGHT, format=(string)$FORMAT$FRAMERATE_OPTION$VIDEOCROP_PIPELINE ! videoconvert ! textoverlay text=$i valignment=top halignment=left font-desc=\"Sans, 40\" ! c.sink_$i"
      # ANKER_CAM_PIPELINES+="hi"
    echo "${ANKER_CAM_PIPELINES[$i]}"
  fi
done

j=0
UGREEN_CAM_PIPELINES=()
WIDTH=640
HEIGHT=480
for i in {0..1}
do
  if [ -e "/dev/rover/cameras/ugreenHubVideo$i" ]
  then
    j=$((i+2))
    UGREEN_CAM_PIPELINES[$i]="v4l2src device=/dev/rover/cameras/ugreenHubVideo$i ! videoconvert ! video/x-raw, width=$WIDTH, height=$HEIGHT, format=(string)$FORMAT$FRAMERATE_OPTION$VIDEOCROP_PIPELINE ! videoconvert ! textoverlay text=$j valignment=top halignment=left font-desc=\"Sans, 40\" ! c.sink_$j"
      # UGREEN_CAM_PIPELINES+="hi"
    echo "${UGREEN_CAM_PIPELINES[$i]}"
  fi
done

GRIPPER_CAM_PIPELINE="v4l2src device=/dev/rover/cameras/gripperCam ! \
                        videoconvert ! \
                        video/x-raw, width=$WIDTH, height=$HEIGHT, format=(string)$FORMAT ! \
                        videoconvert ! \
                        textoverlay text=\"Gripper\" valignment=top halignment=left font-desc=\"Sans, 25\" ! c.sink_0"

BACKUP_CAM_PIPELINE="v4l2src device=/dev/rover/cameras/backupCam ! \
                        videoconvert ! \
                        video/x-raw, width=$WIDTH, height=$HEIGHT, format=(string)$FORMAT ! \
                        videoconvert ! \
                        textoverlay text=\"Back-Up\" valignment=top halignment=left font-desc=\"Sans, 25\" ! c.sink_1"

BIRD_CAM_PIPELINE="v4l2src device=/dev/rover/cameras/birdCam ! \
                        videoconvert ! \
                        video/x-raw, width=$WIDTH, height=$HEIGHT, format=(string)$FORMAT ! \
                        videoconvert ! \
                        textoverlay text=\"Bird's Eye\" valignment=top halignment=left font-desc=\"Sans, 25\" ! c.sink_2"

ZED_CAM_PIPELINE="v4l2src device=/dev/rover/cameras/ZED_front ! \
                        videoconvert ! \
                        video/x-raw, width=2560, height=720, format=(string)$FORMAT ! \
                        videocrop top=0 left=0 right=1280 bottom=0 ! \
                        videoconvert ! \
                        textoverlay text=\"ZED (left)\" valignment=top halignment=left font-desc=\"Sans, 25\" ! c.sink_1"


###############################################################################
# LAUNCH CAMERAS CONNECTED TO USB HUBS
###############################################################################
gst-launch-1.0 \
		compositor name=c background=1 ignore-inactive-pads=true \
			sink_0::xpos=200 sink_0::ypos=0 sink_0::width=400 sink_0::height=300 sink_0::fill_color=0x00000000 \
			sink_1::xpos=0 sink_1::ypos=300 sink_1::width=400 sink_1::height=300 sink_1::fill_color=0x11111111 \
			sink_2::xpos=400 sink_2::ypos=300 sink_2::width=400 sink_2::height=300 sink_2::fill_color=0x11111111 \
			! queue max-size-buffers=0 ! video/x-raw, width=800, height=600 \
      ! $H265_ENCODE \
      ! 'video/x-h265, stream-format=(string)byte-stream' \
      ! h265parse \
      ! rtph265pay mtu=1400 \
      ! rtpulpfecenc percentage=100 \
      ! udpsink host=$ADDRESS port=${CAMERA_PORTS[0]} sync=false async=false \
      $GRIPPER_CAM_PIPELINE \
      $BACKUP_CAM_PIPELINE \
      $BIRD_CAM_PIPELINE \
      &

ZED_DEVICE="/dev/$(basename $(readlink /dev/rover/cameras/ZED_front))"

gst-launch-1.0 v4l2src device=$ZED_DEVICE ! videoconvert \
    ! textoverlay text="ZED (left)" valignment=top halignment=left font-desc="Sans, 10" \
    ! "video/x-raw, width=2560, height=720, format=(string)$FORMAT" \
    ! videocrop top=0 left=0 right=1280 bottom=0 \
    ! $H265_ENCODE \
    ! 'video/x-h265, stream-format=(string)byte-stream' \
    ! h265parse \
    ! rtph265pay mtu=1400 \
    ! rtpulpfecenc percentage=100 \
    ! udpsink host=$ADDRESS port=${CAMERA_PORTS[1]} sync=false async=false


