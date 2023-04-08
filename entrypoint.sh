#!/bin/bash

roslaunch kuka_kvp_hw_interface test_joint_command_interface.launch && sleep 10 && roslaunch kuka_kr4_moveit demo.launch