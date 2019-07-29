# vio-evaluate

test on python 3.7.

pip install:
numpy
PyYAML

commands:
python3 euroc.py -v "align_vio_1.csv"  "/Users/songyang/Downloads/datasets/EuRoC_ASL/V1_02_medium_mav0/state_groundtruth_estimate0/data.csv" "/Users/songyang/project/code/ros/catkin_ws/test/vio_1.csv" "vicon_room_1_02"

python3 euroc.py -l "align_vio_loop_1.csv"  "/Users/songyang/Downloads/datasets/EuRoC_ASL/V1_02_medium_mav0/state_groundtruth_estimate0/data.csv" "/Users/songyang/project/code/ros/catkin_ws/test/vio_loop_1.csv" "vicon_room_1_02"

python3 euroc.py -r "align_vio_1.csv"
