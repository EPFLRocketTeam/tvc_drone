#!/bin/bash
(roslaunch drone_utils replay_nodes.launch)&
sleep 1
rosbag play $(rospack find drone_utils)/replay_analysis/best_data/log2_short.bag /drone_state:=/simu_drone_state
rosnode kill -a
sleep 3
rosrun drone_utils plot_bag_trajectories.py