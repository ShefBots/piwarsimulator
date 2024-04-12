#!/bin/bash

CHALLENGENAME="Lava Palav"

# Get the directory containing the script
SCRIPT_DIR=$(dirname "$0")

# The basic command for this challenge
PYARGS="piwarsimulator.py --brain CheesedEcoDisasterBrain --attachment gripper"

# How to use
USAGE="Usage: $0 [ctrl|sim (default)]"

# Change to the parent directory of the script
cd "$SCRIPT_DIR"/..

# Check the first argument
if [ "$1" = "ctrl" ]; then
    PYARGS+=" --mode control"
elif [ "$1" = "sim" ] || [ $# -eq 0 ]; then
    PYARGS+=" --map RandomEcoDisasterMap"
elif [ "$1" = "-h" ]; then
    # Display usage information
    echo $USAGE
    exit
else
    echo "Invalid argument: $1"
    echo $USAGE
    exit 1
fi

python3 $PYARGS "${@:2}"
