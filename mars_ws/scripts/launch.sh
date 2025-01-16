#!/usr/bin/env bash

###############################################################################
# Helper Functions
###############################################################################

# Print messages as a different color to distinguish from logging output or stderr

function printDebug {
  if [ "$VERBOSE" = "1" ]; then
    echo -e "\033[0m\033[35m[DEBUG] $1\033[0m"
  fi
}

function printInfo {
  # Print blue
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  # Print yellow
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  # Print red
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

###############################################################################
# Parse Command Line Options
###############################################################################

function usage {
echo -e \
'usage: ./launch.sh [rover-address] [options]
       ./launch.sh local [options]

Launch all ROS nodes on the rover and base station.

This automatically generates all of the needed environment variables for
both base station and rover, independent of .bashrc, .rosrc, .rosenv, etc.

If the rover-address is unspecified, the default 192.168.1.20 is used.

Default task to launch is equipment servicing.

Default location for mapviz is Hanksville.

options:
  -u, --user USER           user on rover computer [default=marsrover]
  -t, --task TASK_NAME      task to launch (autonomy|retrieval|servicing|science)
  -l, --location LOC_NAME   location for mapviz (hanksville|gravel_pit|rock_canyon|byu|little_moab)
  -w, --wait SECONDS        number of seconds to wait for SSH to connect
  -a, --attach true|false   automatically attach to tmux session [default=true]
  --base-env                show base environment variables without launching
  --rover-env               show rover environment variables without launching
  -v, --verbose             show detailed logging output

examples:
    ./launch.sh
    ./launch.sh 192.168.1.20
    ./launch.sh 192.168.1.20 -t autonomy
    ./launch.sh 192.168.1.20 -t retrieval
    ./launch.sh 192.168.1.20 -t retrieval -l byu
    ./launch.sh 127.0.0.1 --user byumarsrover --attach=false
    export $(./launch.sh --base-env | xargs)
'
exit
}

# Default settings
TIMEOUT=5
ATTACH='true'
TASK_NAME="servicing"

OPTIONS="$(getopt --name launch --options hvu:t:l:w:a: --longoptions help,verbose,base-env,rover-env,user:,task:,location:,wait:,attach: -- "$@")"

if [ $? != 0 ]; then
  # Incorrect options provided
  usage
  exit 1
fi

eval set -- "$OPTIONS"
while true; do
  case "$1" in
  -h | --help)
    usage
    exit
    ;;
  -u | --user)
    shift
    ROVER_USER="$1"
    ;;
  -t | --task)
    shift
    # if [[ $1 =~ ^(autonomy)|(servicing)|(retrieval)|(science)$ ]]; then
    #   TASK_NAME=$1
    # else
    #   echo "Invalid taskname '$1'"
    #   usage
    # fi
    case "$1" in
      autonomy | servicing | retrieval | science)
        TASK_NAME=$1
        ;;
      *)
        echo "Invalid taskname '$1'"
        usage
        exit
        ;;
    esac
    ;;
  -l | --location)
    shift
    case "$1" in
      hanksville | gravel_pit | rock_canyon | byu | little_moab)
        MAPVIZ_LOCATION=$1
        ;;
      *)
        echo "Invalid MapViz location '$1'"
        usage
        exit
        ;;
    esac
    ;;
  -w | --wait)
    shift
    TIMEOUT="$1"
    ;;
  --base-env)
    [ -z "$ROVER_ENV" ] || {
      echo "Only specify one of --base-env or --rover-env"
      exit 1
    }
    BASE_ENV=1
    ;;
  --rover-env)
    [ -z "$BASE_ENV" ] || {
      echo "Only specify one of --base-env or --rover-env"
      exit 1
    }
    ROVER_ENV=1
    ;;
  -v | --verbose)
    VERBOSE=1
    ;;
  -a | --attach)
    shift
    ATTACH="$1"
    ;;
  --)
    shift
    break
    ;;
  esac
  shift
done

# Non-option arguments
ROVER_ADDRESS="$1"

# Check arguments and set defaults

if [ -z "$ROVER_ADDRESS" ]; then
  DEFAULT_ROVER_ADDRESS='192.168.1.20'
  ROVER_ADDRESS=$DEFAULT_ROVER_ADDRESS
fi

#**Check
ROVER_REPO='~' # default

# Get the location of the BYU-Mars-Rover repository on this computer
# This cd's into the directory where this script is
# and then uses git to get the name of the repository's folder
#**Check
# BASE_STATION_REPO="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && git rev-parse --show-toplevel )"
BASE_STATION_REPO='~'

# Set parameters for running the rover and base station on the same computer
if [[ "$ROVER_ADDRESS" = 'local' || "$ROVER_ADDRESS" = 'localhost' || "$ROVER_ADDRESS" = '127.0.0.1' ]]; then
  printDebug "Running base station and rover on localhost"

  ROVER_ADDRESS='127.0.0.1'

  # Automatically assign the user when running locally
  if [ "$(id -u)" = "0" ]; then
    printError "Do not run as root for local testing" && exit 1
  fi
  ROVER_USER=$(whoami)

  ROVER_REPO=$BASE_STATION_REPO
fi

# Get the base station ip address based on what address is used to route to the rover address
# This looks for the field that comes after the keyword "src" in the output of `ip route show`
BASE_ADDRESS="$(ip route get $ROVER_ADDRESS | awk '{ for(i=1; i<NF; i++) { if($i=="src") { ++i; print $i } } }')"
BASE_USER=$USER

if [ -z "$BASE_ADDRESS" ]; then
  printError "No route found to rover address: '$ROVER_ADDRESS'"
  exit 1
fi

# if $ROVER_USER is not set, set it to the default
if [ -z "$ROVER_USER" ]; then
  printDebug "Setting ROVER_USER to default"
  DEFAULT_ROVER_USER='marsrover'
  ROVER_USER=$DEFAULT_ROVER_USER
fi

# if $ROVER_USER is not set, set it to the default
if [ -z "$MAPVIZ_LOCATION" ]; then
  printDebug "Setting MAPVIZ_LOCATION to default"
  MAPVIZ_LOCATION="hanksville"
fi


###############################################################################
# Prepare environment                                                         #
###############################################################################

BASE_ENVIRONMENT="
ROS_IP=$BASE_ADDRESS
ROVER_ADDRESS=$ROVER_ADDRESS
BASE_ADDRESS=$BASE_ADDRESS
MAPVIZ_LOCATION=$MAPVIZ_LOCATION
BASE_USER=$BASE_USER
"
printDebug "BASE_ENVIRONMENT: $BASE_ENVIRONMENT"

ROVER_ENVIRONMENT="
ROS_IP=$ROVER_ADDRESS
ROVER_ADDRESS=$ROVER_ADDRESS
BASE_ADDRESS=$BASE_ADDRESS
BASE_USER=$BASE_USER
"
printDebug "ROVER_ENVIRONMENT: $ROVER_ENVIRONMENT"

unset ROS_HOSTNAME # Not being used, so make sure it doesn't have an inherited value
export $(echo $BASE_ENVIRONMENT | xargs)

# Source the rover workspace, if it has been built, warn the user that it has not been built otherwise
#**Check
BASE_REPO_SETUP="../install/setup.bash"

# Run the ROS setup script
if ! test -f "$BASE_REPO_SETUP"; then
    printWarning "The rover workspace has not been built, so it cannot be set up.
Please build the workspace with catkin_make, and then run the command \"source ${BASE_REPO_SETUP}\""
    exit 1
else
    source "$BASE_REPO_SETUP"
fi

SET_BASE_ENV_CMD="export $(echo $BASE_ENVIRONMENT | xargs) && unset ROS_HOSTNAME && source $BASE_REPO_SETUP"
SET_ROVER_ENV_CMD="export $(echo $ROVER_ENVIRONMENT | xargs) && unset ROS_HOSTNAME && source $ROVER_REPO/mars_ws/install/setup.bash"  #**Check

# Run a command on the rover with the proper ros environment
function rover_cmd {
  ssh $ROVER_USER@$ROVER_ADDRESS \
    -o ConnectTimeout=$TIMEOUT \
    "$SET_ROVER_ENV_CMD && $1" || {
      printError "Unable to connect after $TIMEOUT seconds"
      exit 1
    }
}

# If we are just exporting the environment variables, stop here
[ "$BASE_ENV" = "1" ] && echo $BASE_ENVIRONMENT && exit
[ "$ROVER_ENV" = "1" ] && echo $ROVER_ENVIRONMENT && exit

# Show the settings we are using
printInfo "Using rover address: $ROVER_ADDRESS"
printInfo "Using rover user: $ROVER_USER"
printInfo "Using base address: $BASE_ADDRESS"

###############################################################################
# Start roscore, nodes, and prepare to teardown on exit                       #
###############################################################################

# NOTES ON HOW THIS WORKS
# nohup : used to prevent processes from being halted when SSH logs out
# &>/dev/null : piping stderr and stdout to something makes this command not hang
# printInfo "Launching roscore"
# rover_cmd "nohup bash -c 'roscore & disown' &> /dev/null"

# Use ros2 launch
LAUNCHER='ros2 launch'

# Decide whether to use mosh or ssh
# mosh is preferred because it will not drop when the connection is bad
CONNECT='mosh'
command -v mosh &> /dev/null || {
  printWarning "Falling back to ssh instead of mosh
Please install mosh with: sudo apt install mosh
"
  CONNECT='ssh'
}

SESSION_NAME='rover'
BASE_CAMERA_SCRIPT_DIR=$BASE_STATION_REPO/scripts/camera
ROVER_CAMERA_SCRIPT_DIR=$ROVER_REPO/scripts/camera

docker run -d -p 8080:8080 -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy

# Kill gstreamer processes that may be already open
pkill gst

###############################################################################
# LAUNCH TMUX WINDOWS AND LAUNCH ROS                                          #
###############################################################################
# Kill any existing sessions
tmux kill-server
sleep 0.01
# Start new session
tmux new-session -d -t $SESSION_NAME
# Enable full color output in tmux
tmux set-option -g default-terminal "screen-256color"
# Enable pane labels
tmux set -g pane-border-status top
tmux set -g pane-border-format "#{pane_title}"
tmux set-option -g allow-rename off
tmux set-window-option -g allow-rename off
tmux set-window-option -g automatic-rename off
# Keep history for the mosh terminal
tmux set -g history-limit 30000
# Enable clicking between windows
tmux set -g mouse on
# Split the window horizontally
tmux split-window -h
# Select the right pane and set title
tmux select-pane -t 1 -T rover
# Connect to rover
tmux send-keys "$CONNECT $ROVER_USER@$ROVER_ADDRESS" Enter
# Kill gstreamer processes that may be already open
# if [ ROVER_ADDRESS != '127.0.0.1']; then
tmux send-keys "pkill gst" Enter
# fi
echo $BASE_ADDRESS

###############################################################################
# START ROVER LAUNCH FILE
###############################################################################
tmux send-keys "$SET_ROVER_ENV_CMD && $LAUNCHER start \
  rover_task_${TASK_NAME}.launch.py" Enter

# Select the left pane and set title
tmux select-pane -t 0
tmux select-pane -t 0 -T base-station-${TASK_NAME}-task
# Start base camera window

###############################################################################
# START BASE LAUNCH FILE
###############################################################################
tmux send-keys "$SET_BASE_ENV_CMD && $LAUNCHER start \
  base_task_${TASK_NAME}.launch.py" Enter


# Attach to the beautiful tmux session we have created
if $ATTACH; then
  tmux a -t $SESSION_NAME
else
  printInfo "You may now attach to the running tmux session by running: tmux a -t $SESSION_NAME"
fi
