#!/bin/bash

source /opt/ros/noetic/setup.bash
# 第一个：topiclistOS0.sh
gnome-terminal -- bash -c '"${HOME}/workspace/mspace/topiclistOS0.sh"; exec bash' &
sleep 3
# 第二个：ego_fastlio2_one_livox.sh
gnome-terminal -- bash -c '"${HOME}/workspace/mspace/ego_fastlio2_one_livox.sh"; exec bash' &