#!/bin/bash

alias build="colcon build \
                --cmake-args -DCMAKE_CXX_COMPILER=/usr/bin/g++-16\
                --packages-select "
