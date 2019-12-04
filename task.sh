#!/bin/bash

ROOT_PATH="/home/song/catkin_ws/src/kalibr/cal/task"

dir=${ROOT_PATH}/"cal-cam_16"
cd ${dir}
date
echo ${dir}
rosrun kalibr kalibr_calibrate_cameras --bag stereo_calibra_16.bag --topics /left /right --models pinhole-equi pinhole-equi --target april_6x6_80x80cm.yaml --approx-sync 0.1

dir=${ROOT_PATH}/"cal-cam_17"
cd ${dir}
date
echo ${dir}
rosrun kalibr kalibr_calibrate_cameras --bag stereo_calibra_17.bag --topics /left /right --models pinhole-equi pinhole-equi --target april_6x6_80x80cm.yaml --approx-sync 0.1


dir=${ROOT_PATH}/"cal-cam_18"
cd ${dir}
date
echo ${dir}
rosrun kalibr kalibr_calibrate_cameras --bag stereo_calibra_18.bag --topics /left /right --models pinhole-equi pinhole-equi --target april_6x6_80x80cm.yaml --approx-sync 0.1

dir=${ROOT_PATH}/"cal-cam_19"
cd ${dir}
date
echo ${dir}
rosrun kalibr kalibr_calibrate_cameras --bag stereo_calibra_19.bag --topics /left /right --models pinhole-equi pinhole-equi --target april_6x6_80x80cm.yaml --approx-sync 0.1


dir=${ROOT_PATH}/"cal-cam_20"
cd ${dir}
date
echo ${dir}
rosrun kalibr kalibr_calibrate_cameras --bag stereo_calibra_20.bag --topics /left /right --models pinhole-equi pinhole-equi --target april_6x6_80x80cm.yaml --approx-sync 0.1


dir=${ROOT_PATH}/"cal-cam_21"
cd ${dir}
date
echo ${dir}
rosrun kalibr kalibr_calibrate_cameras --bag stereo_calibra_21.bag --topics /left /right --models pinhole-equi pinhole-equi --target april_6x6_80x80cm.yaml --approx-sync 0.1

data