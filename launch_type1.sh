#!/bin/bash
source /mnt/ros_entrypoint.sh
ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/mnt/nav2_params1.yaml default_bt_xml_filename:=/mnt/navigate_w_replanning_and_recovery.xml