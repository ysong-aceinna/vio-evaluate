#!/bin/bash

# gt_ds="MH_02_easy_mav0"
# tm="machine_hall_02"

# gt_ds="V1_02_medium_mav0"
# tm="vicon_room_1_02"

gt_ds="V2_03_difficult_mav0"
tm="vicon_room_2_03"

echo "dataset:"${gt_ds}

dir="vio1"

script="../euroc.py"
opt="-v"
gt="/Users/songyang/Downloads/datasets/EuRoC_ASL/"${gt_ds}"/state_groundtruth_estimate0/data.csv"

mkdir ${dir}
cd ${dir}

input="vio_1.csv"
output="align_vio_1.csv"
python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 
echo "******<"${output}">*******"
python3 ${script} "-r" "/Users/songyang/project/code/github/vio-evaluate/"${dir}/${output}
echo "**************************"


input="vio_2.csv"
output="align_vio_2.csv"
python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 
echo "******<"${output}">*******"
python3 ${script} "-r" "/Users/songyang/project/code/github/vio-evaluate/"${dir}/${output}
echo "**************************"


input="vio_3.csv"
output="align_vio_3.csv"
python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 
echo "******<"${output}">*******"
python3 ${script} "-r" "/Users/songyang/project/code/github/vio-evaluate/"${dir}/${output}
echo "**************************"


input="vio_4.csv"
output="align_vio_4.csv"
python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 
echo "******<"${output}">*******"
python3 ${script} "-r" "/Users/songyang/project/code/github/vio-evaluate/"${dir}/${output}
echo "**************************"


input="vio_5.csv"
output="align_vio_5.csv"
python3 ${script} ${opt} ${output} ${gt}  "/Users/songyang/project/code/ros/catkin_ws/test/"${input}  ${tm} 
echo "******<"${output}">*******"
python3 ${script} "-r" "/Users/songyang/project/code/github/vio-evaluate/"${dir}/${output}
echo "**************************"

