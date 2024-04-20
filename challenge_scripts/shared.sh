#!/bin/bash

# How to use
USAGE="Usage: $0 [ctrl|sim (default)]"

# Shared for all control calls
ctrl_shared() {
    PYARGS+=" --mode control"
#    PYARGS+=" --radio true"
    PYARGS+=" --frame_rate 30"
    PYARGS+=" --leds true"
}

# Shared for all simulation calls
sim_shared() {
    PYARGS+=" --leds true"
}
