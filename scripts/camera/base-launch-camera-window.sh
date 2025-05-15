#!/usr/bin/env bash

usage() {
  echo \
  "Usage:
    base-launch-camera-window.sh [OPTION...]
        
  Options:
    (-c | --channel) [0-4]
    (-b | --brightness) [1-200]
    (-o | --contrast) [1-200]
    (-v | --verbose)
    (-h | --help)
              
  Launch GStreamer window for camera feeds coming from the rover.

  Examples:
    ./base-launch-camera-window.sh
    ./base-launch-camera-window.sh -c 0
    ./base-launch-camera-window.sh --channel 0
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
  --options c:b:o:vh \
  --longoptions channel:,brightness:,contrast:,verbose,help \
  -- "$@")
INVALID_ARGUMENTS=$?
if [ "$INVALID_ARGUMENTS" != "0" ]; then
  usage
fi

CHANNEL=0
BRIGHTNESS=100
CONTRAST=100

eval set -- "$PARSED_ARGUMENTS"
while :
do
  case "$1" in
    -c | --channel)
    if (($2 >= 0 && $2 <= 4)); then
      CHANNEL="$2"
    else
      echo "Invalid channel '$2'. Only channels 0-4 are supported."
      usage
    fi
      shift 2;;
    -b | --brightness)
    if (($2 >= 0 && $2 <= 200)); then
      BRIGHTNESS="$2"
    else
      echo "Invalid brightness '$2'. Only 0-200% supported."
      usage
    fi
      shift 2;;
    -o | --contrast)
    if (($2 >= 0 && $2 <= 200)); then
      CONTRAST="$2"
    else
      echo "Invalid contrast '$2'. Only 0-200% supported."
      usage
    fi
      shift 2;;
    -v | --verbose)
      _V=1
      shift ;;
    -h | --help)
      usage ;;
    --) shift; break ;;
    *) echo "Unexpected option: $1 -- this should not happen."
      usage ;;
  esac
done

BRIGHTNESS=`echo "scale=2; ($BRIGHTNESS/200) - 0.5" | bc`
echo $BRIGHTNESS

CONTRAST=`echo "scale=2; ($CONTRAST/100)" | bc`
echo $CONTRAST

function launch_single_cam_window() {
	log "gst-launch-1.0 udpsrc port=$CAM_PORT 
	! application/x-rtp,encoding-name=H265,payload=96 
	! rtpstorage ! rtpjitterbuffer ! rtpulpfecdec 
	! rtph265depay ! h265parse ! queue ! $H265_DECODE 
	! xvimagesink sync=false async=false -e
  
  "

	gst-launch-1.0 udpsrc port=$CAM_PORT \
	! application/x-rtp,encoding-name=H265,payload=96 \
	! rtpstorage ! rtpjitterbuffer ! rtpulpfecdec \
	! rtph265depay ! h265parse ! queue ! $H265_DECODE \
  ! videobalance contrast=$CONTRAST brightness=$BRIGHTNESS \
	! xvimagesink sync=false async=false -e
}

case "$CHANNEL" in
        "0") CAM_PORT=5010
        ;;
        "1") CAM_PORT=5020
        ;;
        "2") CAM_PORT=5030
        ;;
        "3") CAM_PORT=5040
        ;;
        "4") CAM_PORT=5050
        ;;
		*)
		    echo "Invalid channel: '$CHANNEL'"
			usage
			exit 1
		;;
esac

H265_DECODE='avdec_h265' # default

launch_single_cam_window