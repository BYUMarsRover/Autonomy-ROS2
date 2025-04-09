#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches tasks over SSH using the 'rover_runtime' tmux session

function printDebug {
  if [ "$VERBOSE" = "1" ]; then
    echo -e "\033[0m\033[35m[DEBUG] $1\033[0m"
  fi
}

function printInfo {
  # print blue
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  # print yellow
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  # print red
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

If the rover-address is unspecified, the default 192.168.1.120 is used.

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
    ./launch.sh 192.168.1.120
    ./launch.sh 192.168.1.120 -t autonomy
    ./launch.sh 192.168.1.120 -t retrieval
    ./launch.sh 192.168.1.120 -t retrieval -l byu
    ./launch.sh 127.0.0.1 --user byumarsrover --attach=false
    export $(./launch.sh --base-env | xargs)
'
exit
}

function rover_cmd {
  echo 'Running command on rover:'
  ssh $ROVER_USER@$ROVER_ADDRESS \
    -o ConnectTimeout=$TIMEOUT \
    -p $DOCKER_SSH_PORT \
    "$1" || {
      printError "Unable to connect after $TIMEOUT seconds"
      return 1
    }
}

# Default settings
TIMEOUT=5
ATTACH='true'
TASK_NAME="servicing"
ROVER_ADDRESS=192.168.1.120
DOCKER_SSH_PORT=2233

# Parse command line arguments
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
  DEFAULT_ROVER_ADDRESS='192.168.1.120'
  ROVER_ADDRESS=$DEFAULT_ROVER_ADDRESS
fi

ROVER_REPO='~/Autonomy-ROS2' # default

# Get the location of the Autonomy-ROS2 repository on this computer
# This cd's into the directory where this script is
# and then uses git to get the name of the repository's folder
BASE_STATION_REPO="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && git rev-parse --show-toplevel )"


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
ROS_MASTER_URI=http://$ROVER_ADDRESS:11311
ROS_IP=$BASE_ADDRESS
HUSKY_URDF_EXTRAS=$BASE_STATION_REPO/rover_ws/src/husky_custom_description/urdf/custom_description.urdf.xacro
ROVER_ADDRESS=$ROVER_ADDRESS
BASE_ADDRESS=$BASE_ADDRESS
MAPVIZ_LOCATION=$MAPVIZ_LOCATION
BASE_USER=$BASE_USER
"
printDebug "BASE_ENVIRONMENT: $BASE_ENVIRONMENT"

ROVER_ENVIRONMENT="
ROS_MASTER_URI=http://$ROVER_ADDRESS:11311
ROS_IP=$ROVER_ADDRESS
HUSKY_URDF_EXTRAS=$ROVER_REPO/rover_ws/src/husky_custom_description/urdf/custom_description.urdf.xacro
ROVER_ADDRESS=$ROVER_ADDRESS
BASE_ADDRESS=$BASE_ADDRESS
BASE_USER=$BASE_USER
"
printDebug "ROVER_ENVIRONMENT: $ROVER_ENVIRONMENT"

unset ROS_HOSTNAME # Not being used, so make sure it doesn't have an inherited value
export $(echo $BASE_ENVIRONMENT | xargs)

# Source the rover workspace, if it has been built, warn the user that it has not been built otherwise
BASE_REPO_SETUP="$BASE_STATION_REPO/rover_ws/source/setup.sh"

# Run the ROS setup script
if ! test -f "$BASE_REPO_SETUP"; then
    printWarning "The rover workspace has not been built, so it cannot be set up.
Please build the workspace with catkin_make, and then run the command \"source ${BASE_REPO_SETUP}\""
    exit 1
else
    source "$BASE_REPO_SETUP"
fi

SET_BASE_ENV_CMD="export $(echo $BASE_ENVIRONMENT | xargs) && unset ROS_HOSTNAME && source $BASE_REPO_SETUP"
SET_ROVER_ENV_CMD="export $(echo $ROVER_ENVIRONMENT | xargs) && unset ROS_HOSTNAME && source $ROVER_REPO/rover_ws/devel/setup.sh"


echo 'Break'
exit 0

# Check for an SSH connection to the rover's Docker container
if ! rover_cmd "echo"  &> /dev/null
then
    printError "No available SSH connection to the rover's Docker container"
    echo "Here's some debugging suggestions:"
    echo "  - Make sure the SSH keys are setup by running the setup_ssh.sh script"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"
    echo "  - Ensure the rover's Docker container is running"

    exit
fi

# Check if tmux is running on the rover's Docker container
if ! rover_cmd "tmux has-session -t rover_runtime" &> /dev/null
then
    printError "No tmux session found in the rover's Docker container"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover's Docker container is running the 'rover_runtime' tmux session"

    exit
fi

#TODO FIX THIS LATER

# # Check that only one window is open in the 'rover_runtime' tmux session
# if [ $(ssh marsrover@$ROVER_ADDRESS -p $DOCKER_SSH_PORT "tmux list-windows -t rover_runtime | wc -l") -ne 1 ]
# then
#     printWarning "Multiple windows found in the 'rover_runtime' tmux session"
#     echo "Simply entering the current tmux session for cleanup..."
#     ssh -t -X marsrover@$ROVER_ADDRESS -p $DOCKER_SSH_PORT 'tmux attach -t rover_runtime'

#     exit
# fi

# Check that only one pane is open in the 'rover_runtime' tmux session
if [ $(rover_cmd "tmux list-panes -t rover_runtime | wc -l") -ne 1 ]
then
    printWarning "Multiple panes found in the 'rover_runtime' tmux session"
    echo "Simply entering the current tmux session for cleanup..."
    ssh -t -X marsrover@$ROVER_ADDRESS -p $DOCKER_SSH_PORT 'tmux attach -t rover_runtime'

    exit
fi

# Launch the specified task configuration over SSH
case "$1" in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        # Send tmux commands to the rover's Docker container over SSH
        rover_cmd "\
            tmux split-window -h -t rover_runtime:0.0; \
            tmux select-pane -t rover_runtime:0.1; \
            tmux send-keys -t rover_runtime:0.1 'export ROS_DISCOVERY_SERVER=127.0.0.1:11811' Enter; \
            tmux send-keys -t rover_runtime:0.1 'ros2 launch start rover_task_autonomy_new_launch.py'" # NO ENTER 
        ;;
    "servicing")
        printWarning "Not implemented yet"
        ;;
    "retrieval")
        printWarning "Not implemented yet"
        ;;
    "science")
        printInfo "Setting up the science task..."
        # Send tmux commands to the rover's Docker container over SSH
        rover_cmd "\
            tmux split-window -h -t rover_runtime:0.0; \
            tmux select-pane -t rover_runtime:0.1; \
            tmux send-keys -t rover_runtime:0.1 'export ROS_DISCOVERY_SERVER=127.0.0.1:11811' Enter; \
            tmux send-keys -t rover_runtime:0.1 'ros2 launch start rover_task_asciene_launch.py'" # NO ENTER 
        ;;
        ;;
    *)
        printWarning "No task specified, simply entering the current tmux session..."
        echo "Specify a task using 'bash launch.sh <task>' "
        ;;
esac

# Attach to the 'rover_runtime' tmux session
ssh -t -X marsrover@$ROVER_ADDRESS -p $DOCKER_SSH_PORT 'tmux attach -t rover_runtime'

