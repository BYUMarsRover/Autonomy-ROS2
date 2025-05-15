usage() {
  echo "Usage:
          rover-calibrate-fad.sh

        Calibrates the FAD detector to return an intensity value for the amount of green

        FAD camera cannot be busy (streaming video to base station). If so, this
        script will not be able to calibrate the fad.

        Options:
            -d  device
            -s --site <site number>              Site Number


        Examples:
          ./rover-calibrate-fad.sh -d /dev/video0 -s 4;
          ./rover-calibrate-fad.sh -d gripperCam -s 3;
          "
  exit 2;
}

PARSED_ARGUMENTS=$(getopt \
  --name RoverCamera.sh \
  --options d:s:n:h \
  --longoptions device:,site:,number:,help \
  -- "$@")
INVALID_ARGUMENTS=$?
if [ "$INVALID_ARGUMENTS" != "0" ]; then
  usage
fi

eval set -- "$PARSED_ARGUMENTS"
while :
do
  case "$1" in
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
    -s | --site)
        SITE=$2
        shift 2;;
    -h | --help)
      usage ;;
    --) shift; break ;;
    *) echo "Unexpected option: $1 -- this should not happen."
      usage ;;
  esac
done

#ADDRESS="${BASE_USER}@${BASE_ADDRESS}"

[ -z "$DEVICE" ] && echo "You must specify a device" && usage;
[ -z "$SITE" ] && echo "You must specify a site" && usage;

mkdir ~/BYU-Mars-Rover/rover_ws/src/science/src/presentation/resources/ -p
FILE="BYU-Mars-Rover/rover_ws/src/science/src/presentation/resources/${SITE}-$(date +%s)-%d.png"
#FILE="BYU-Mars-Rover/rover_ws/src/science/src/presentation/resources/${SITE}-%d.png"
gst-launch-1.0 v4l2src device=$DEVICE num-buffers=10 \
! videoconvert ! 'video/x-raw, width=1280, height=720' \
! pngenc compression-level=0 \
! multifilesink location="$HOME/${FILE}" sync=false async=false

#scp $HOME/${FILE} $ADDRESS:/home/${BASE_USER}/${FILE}
