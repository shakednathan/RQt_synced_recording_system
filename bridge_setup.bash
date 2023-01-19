#!/bin/bash
#source both ros distros
source ~/ros1_commands.bash
source ~/ros2_commands.bash

ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
