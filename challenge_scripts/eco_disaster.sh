#!/bin/bash

# The name of this challenge
CHALLENGENAME="Eco-Disaster"

# The basic command for this challenge
PYARGS="piwarsimulator.py --brain CheesedEcoDisasterBrain --attachment gripper"

# Get the directory containing the script
SCRIPT_DIR=$(dirname "$0")

# Load shared functions
cd "$SCRIPT_DIR"
source shared.sh

# Change to the parent directory of the script
cd ..

# Check the first argument
if [ "$1" = "ctrl" ]; then
    ctrl_shared
elif [ "$1" = "sim" ] || [ $# -eq 0 ]; then
    PYARGS+=" --map RandomEcoDisasterMap"
    sim_shared
elif [ "$1" = "-h" ]; then
    # Display usage information
    echo $USAGE
    exit
else
    echo "Invalid argument: $1"
    echo $USAGE
    exit 1
fi

echo "Launching $CHALLENGENAME..."
echo python3 $PYARGS "${@:2}"
python3 $PYARGS "${@:2}"