#!/bin/bash

# gt_ds="MH_02_easy_mav0"
# tm="machine_hall_02"

gt_ds="V1_02_medium_mav0"
tm="vicon_room_1_02"

# gt_ds="V2_03_difficult_mav0"
# tm="vicon_room_2_03"


dir="vio5"

script="../euroc.py"
opt="-v"
gt="/Users/songyang/Downloads/datasets/EuRoC_ASL/"${gt_ds}"/state_groundtruth_estimate0/data.csv"

mkdir ${dir}
cd ${dir}

input="vio_5-1.csv"
output="align_vio_5-1.csv"
python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 

input="vio_5-2.csv"
output="align_vio_5-2.csv"
python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 

# input="vio_2-3.csv"
# output="align_vio_2-3.csv"
# python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 

# input="vio_2-4.csv"
# output="align_vio_2-4.csv"
# python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 

# input="vio_2-5.csv"
# output="align_vio_2-5.csv"
# python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 
