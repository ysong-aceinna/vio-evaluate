#!/bin/bash



# gt_ds="V1_02_medium_mav0"
# tm="vicon_room_1_02"
gt_ds="V2_03_difficult_mav0"
tm="vicon_room_2_03"

dir="loop"

script="../euroc.py"
opt="-l"
gt="/Users/songyang/Downloads/datasets/EuRoC_ASL/"${gt_ds}"/state_groundtruth_estimate0/data.csv"

mkdir ${dir}
cd ${dir}

input="vio_loop_1.csv"
output="align_loop_1.csv"
python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 

input="vio_loop_2.csv"
output="align_loop_2.csv"
python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 

# input="vio_loop_3.csv"
# output="align_loop_3.csv"
# python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 

# input="vio_loop_4.csv"
# output="align_loop_4.csv"
# python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 


