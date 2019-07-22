#!/bin/bash
# del old key
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
# add the new key
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install -y ros-kinetic-dbw-mkz-msgs
# ./ros
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
