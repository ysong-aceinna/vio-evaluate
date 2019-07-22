# vio-evaluate

test on python 3.7.

pip install:
numpy

commands:
python3 euroc.py -o "align_vins_1.csv" -v "/Users/songyang/Downloads/datasets/EuRoC_ASL/V1_02_medium_mav0/state_groundtruth_estimate0/data.csv" "/Users/songyang/project/code/ros/catkin_ws/test/vins_1.csv"

python3 euroc.py -o "align_loop_2.csv" -l "/Users/songyang/Downloads/datasets/EuRoC_ASL/V1_02_medium_mav0/state_groundtruth_estimate0/data.csv" "/Users/songyang/project/code/ros/catkin_ws/test/loop_2.csv"

python3 euroc.py -r "align_vins_1.csv"
