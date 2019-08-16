#!/bin/sh

set -eu

# make catkin_ws
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd ~/catkin_ws/src
ln -s /aisaac-strategy

cd ~/catkin_ws
catkin_make -j4 || catkin_make -j4 || catkin_make -j4
source devel/setup.bash

mkdir -p ~/.config/matplotlib/
echo "backend: Agg" >> ~/.config/matplotlib/matplotlibrc

roscore &

sleep 3

rosrun aisaac test1.py
rosrun aisaac test2.py
rosrun aisaac test3.py

killall -9 roscore
killall -9 rosmaster
