#!/bin/bash


usage() {
  echo "Usage:
          rover-launch-single-camera.sh
        
        Options:
            -a, --address <client-ip-address>     Address for base station
            -c, --channel [0-4]                   Channel for UDP port. Default is all channels
            -d | --device <camera_name>|<camera_number>|<device_file>|<device_symlink>
            -q, --quality [240p|480p|720p|1080p|ZED|ZED-left|ZED-right|Low|Medium|High]
            -v, --verbose                         Print gst commands
            -b, --background                      Run in background. Used in launch script

        Examples:
          ./rover-launch-single-camera.sh -a 192.168.1.65 -c 0 -d ZED_front -q ZED-left
          ./rover-launch-single-camera.sh -a 192.168.1.65 -c 1 -d /dev/video0 -q Low
          ./rover-launch-single-camera.sh -a 127.0.0.1 -c 2 -d pciecam2 -q Low
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
  --options a:c:d:q:vbh \
  --longoptions address:,quality:,verbose,help \
  -- "$@")
INVALID_ARGUMENTS=$?
if [ "$INVALID_ARGUMENTS" != "0" ]; then
  usage
fi

GRAYSCALE=0

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
    -c | --channel)
      if [[ $2 =~ ^[0-4]$ ]]; then
        CHANNEL=$2 
      else
        echo "Invalid channel '$2'"
        usage
      fi
      shift 2;;
    -d | --device)
      if [[ $2 =~ "/" ]]; then
        DEVICE="/dev/$(basename $(readlink $2 || echo $2))"
      else
        if [[ $2 =~ ^[0-9]$ ]]; then
          DEVICE="/dev/video$2"
        else
          DEVICE="/dev/$(basename $(readlink /dev/rover/cameras/$2 || echo "$2"))"
        fi
      fi
      if [ -z "$(v4l2-ctl -d $DEVICE -l)" ]; then
        echo "Invalid device '$2' ($DEVICE)"
        usage
      fi
      shift 2;;
    -q | --quality)
      if [[ $2 =~ ^(240p)|(480p)|(720p)|(1080p)|(ZED)|(ZED-left)|(ZED-right)|(High)|(Medium)|(Low)$ ]]; then
        QUALITY=$2
      else
        echo "Invalid quality '$2'"
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
[ -z "$DEVICE" ] && echo "You must specify a device" && usage;
[ -z "$QUALITY" ] && echo "You must specify quality" && usage;
[ -z "$CHANNEL" ] && echo "You must specify a channel" && usage;

# Default to 480p / Low
WIDTH=640
HEIGHT=480
BITRATE=150000  # TODO: Find good values for all quality settings

case "$QUALITY" in
  240p)
    WIDTH=360
    HEIGHT=240
  ;;
  480p | Low)
    WIDTH=640
    HEIGHT=480
  ;;
  720p | Medium)
    WIDTH=1280
    HEIGHT=720
  ;;
  1080p | High)
    WIDTH=1920
    HEIGHT=1080
  ;;
  ZED | ZED-left | ZED-right)
    WIDTH=2560
    HEIGHT=720
  ;;
esac

CAMERA_PORTS=(5010 5020 5030 5040 5050)
DEVICES=(video0 video1 video2 video3 video4)

CAMERA_PORT="${CAMERA_PORTS[$CHANNEL]}"

FORMAT="I420"

if [[ "$QUALITY" =~ ZED-(right)|(left) ]]; then
  case "$QUALITY" in
    ZED-left)
      LEFT=0
      RIGHT=1280
    ;;
    ZED-right)
      LEFT=1280
      RIGHT=0
    ;;
  esac

  VIDEOCROP_PIPELINE="! videocrop top=0 left=$LEFT right=$RIGHT bottom=0"
fi

if [ "$(uname -m)" = "x86_64" ]; then
  H265_ENCODE="videoconvert ! x265enc bitrate=$BITRATE speed-preset=1 tune=4"
else
  H265_ENCODE="nvvidconv ! nvv4l2h265enc bitrate=$BITRATE preset-level=0"
fi
# echo "Using $H265_ENCODE"
# exit 0

gst-launch-1.0 v4l2src device=$DEVICE ! videoconvert \
! "video/x-raw, width=$WIDTH, height=$HEIGHT, format=(string)$FORMAT, framerate=(fraction)30/1" $VIDEOCROP_PIPELINE \
! $H265_ENCODE \
! 'video/x-h265, stream-format=(string)byte-stream' \
! h265parse ! rtph265pay ! rtpulpfecenc \
! udpsink host=$ADDRESS port=$CAMERA_PORT sync=false async=false
