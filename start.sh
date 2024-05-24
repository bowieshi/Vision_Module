#!/bin/bash

cd /home/infantry_1/catkin_ws

catkin_make

source devel/setup.bash

roslaunch detector predictor.launch


