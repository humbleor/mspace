#!/bin/bash
# 第一个：topiclistOS0.sh
gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash && /home/joey/workspace/mspace/topiclist_more.sh; exec bash" &
sleep 3
# 第二个：ego_fastlio2_one_livox.sh
gnome-terminal -- bash -c "/home/joey/workspace/mspace/ego_fastlio2_one_livox.sh; exec bash" &