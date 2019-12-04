# -*- coding: utf-8 -*
import sys
import os
import datetime
import time
import math
import numpy as np
import csv
import yaml
import getopt
import attitude
import geoparams
import kml_gen
'''
这个脚本用于将INS1000的数据作为reference，来评估VINS的性能。
'''

'''
实现ins1000和vins算法输出数据对齐。

参数：
    output_align_file: 保存对齐数据的文件名；
    ground_truth_file: ground_truth文件，由python-ins1000 driver录制
    algo_result_file: vins algo的数据结果文件，由 rostopi echo -p 录制。
    delta_yaw: 因为ins1000的heading不准确，delta_yaw用于在heading对齐时的微调，是手动试出来的。

delta_yaw手动调整的方法为：微调delta_yaw，用matlab对比algo和对齐后的ins1000的（主要是y轴）位置，直到初始阶段的轨迹重合。

姿态的手动对齐：由于ins1000和camera安装位置的不同，会存在初始姿态的offset。需要在调用save_aligned_data_vins并得到output_align_file后，手动去除offset。


'''
def convert_vio2kml(output_align_file, algo_result_file, start_lla, start_att):
    output_file = open(output_align_file, 'a+')
    output_file.write("time,\
        algo_pose_x, algo_pose_y, algo_pose_z, algo_roll, algo_pitch, algo_yaw, algo_vel_x, algo_vel_y, algo_vel_z,\
        algo_lat, algo_lon, algo_alt \n".replace(" ", ""))

    cur_line = 1
    R2D = 180/math.pi
    b_align = False
    timestamp = 0
    first_timestamp = 0

    algo_llas = np.array([])
    algo_headings = np.array([])
    start_att = start_att/R2D
    start_lla = start_lla/R2D

    with open(algo_result_file,'r') as csv_file:
        algo_result_reader = csv.reader(csv_file)

        for idx,item in enumerate(algo_result_reader):
            timestamp = float(item[0][:-6])/1000  # ms to sec
            algo_x = float(item[1])
            algo_y = float(item[2])
            algo_z = float(item[3])
            algo_v_x = float(item[11]) #都转换为body系下的表示的速度,主要是沿行驶方向上的速度.
            algo_v_y = float(item[12])
            algo_v_z = float(item[13])
            algo_qx = float(item[5])
            algo_qy = float(item[6])
            algo_qz = float(item[7])
            algo_qw = float(item[4])

            (algo_yaw, algo_pitch, algo_roll) = attitude.quat2euler(np.array([algo_qw, algo_qx, algo_qy, algo_qz]))

            if not b_align:
                b_align = True
                first_timestamp = timestamp

                # ins1000在NED下的姿态矩阵c_bn
                c_bn = attitude.euler2dcm(start_att)
                # NED转VINS的导航系，注意此时还没有yaw对齐。
                c_vn = attitude.rot_x(math.pi)
                # 要将ins1000的body系转到vins下的body系。此处是围绕x轴转180°.
                c_ib_vb = attitude.rot_x(math.pi)
                # ins1000在VINS导航系下的姿态矩阵c_bv，注意此时还没有yaw对齐
                # c_bv1 = c_bn.dot(c_vn.T)
                c_bv = c_ib_vb.dot(c_bn.dot(c_vn.T))
                ins1000_euler = attitude.dcm2euler(c_bv)
                # ins1000的yaw不准，需要增加一个delta_yaw项做人工对齐。
                # c_vn = attitude.rot_z(algo_yaw - ins1000_euler[0] + delta_yaw).dot(c_vn)
                c_vn = attitude.rot_z(ins1000_euler[0] - algo_yaw).dot(c_vn)
                t_vn = np.array([algo_x, algo_y, algo_z])

                c_ne = attitude.ecef_to_ned(start_lla[0], start_lla[1]) # DCM of efef to ned
                ins1000_pos_ecef = geoparams.lla2ecef(start_lla).reshape(1,3)

                c_ev = c_vn.dot(c_ne).T
            # else:
            #     ins1000_pos_ecef = np.vstack((ins1000_pos_ecef,geoparams.lla2ecef(np.array([gt_lat,gt_lon,gt_alt]))))

            # ins1000_pos_ned = c_ne.dot(ins1000_pos_ecef[-1] - ins1000_pos_ecef[0])
            # ins1000_pos_v = c_vn.dot(ins1000_pos_ned) + t_vn
            # ins1000_rpy_v = attitude.dcm2euler(c_ib_vb.dot(attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll])).dot(c_vn.T)))
            # ins1000_vel_v = attitude.rot_x(math.pi).dot(attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll])).dot(np.array([gt_v_x, gt_v_y, gt_v_z])))

            algo_ecef = c_ev.dot((np.array([algo_x, algo_y, algo_z]) - t_vn)) + ins1000_pos_ecef[0]
            algo_lla = geoparams.ecef2lla(algo_ecef)
            # algo的yaw转为ned系的heading。
            algo_att = attitude.dcm2euler(attitude.rot_x(math.pi).dot(attitude.euler2dcm(np.array([algo_yaw, algo_pitch, algo_roll]))).dot(c_vn))
            # algo在body下对vins导航系速度的表示
            algo_vel = attitude.euler2dcm(np.array([algo_yaw, algo_pitch, algo_roll])).dot(np.array([algo_v_x, algo_v_y, algo_v_z]))

            str1 = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n'.format(str(timestamp),\
                algo_x, algo_y, algo_z, algo_roll*R2D, algo_pitch*R2D, algo_yaw*R2D, algo_vel[0], algo_vel[1], algo_vel[2], \
                algo_lla[0]*R2D, algo_lla[1]*R2D, algo_lla[2])

            output_file.write(str1)

            if algo_llas.shape[0] == 0:
                algo_llas = np.array([algo_lla])
                algo_headings = np.array([algo_att[0]*R2D])
            else:
                algo_llas = np.append(algo_llas, np.array([algo_lla]),axis=0)
                algo_headings = np.append(algo_headings, np.array([algo_att[0]*R2D]))

    output_file.close()
    kml_path = os.path.abspath('./kmlpath')
    if not os.path.exists(kml_path):
        os.makedirs(kml_path)
    # kml_gen.kml_gen(kml_path, gt_llas, gt_headings, 'ground_truth', color='ff00ff00')
    kml_gen.kml_gen(kml_path, algo_llas, algo_headings, 'algo', color='ff0000ff')
    print("time length:{0:0.3f}s".format(timestamp - first_timestamp))

def convert_loop2kml(output_align_file, algo_result_file, start_lla, start_att):
    output_file = open(output_align_file, 'a+')
    output_file.write("time,\
        algo_pose_x, algo_pose_y, algo_pose_z, algo_roll, algo_pitch, algo_yaw,\
        algo_lat, algo_lon, algo_alt \n".replace(" ", ""))

    cur_line = 1
    R2D = 180/math.pi
    b_align = False
    first_timestamp = 0
    timestamp = 0

    algo_llas = np.array([])
    algo_headings = np.array([])
    start_att = start_att/R2D
    start_lla = start_lla/R2D

    with open(algo_result_file,'r') as csv_file:
        algo_result_reader = csv.reader(csv_file)

        for idx,item in enumerate(algo_result_reader):
            timestamp = float(item[0][:-6])/1000  # ms to sec
            algo_x = float(item[1])
            algo_y = float(item[2])
            algo_z = float(item[3])
            algo_qx = float(item[5])
            algo_qy = float(item[6])
            algo_qz = float(item[7])
            algo_qw = float(item[4])

            (algo_yaw, algo_pitch, algo_roll) = attitude.quat2euler(np.array([algo_qw, algo_qx, algo_qy, algo_qz]))

            if not b_align:
                b_align = True
                first_timestamp = timestamp

                # ins1000在NED下的姿态矩阵c_bn
                c_bn = attitude.euler2dcm(start_att)
                # NED转VINS的导航系，注意此时还没有yaw对齐。
                c_vn = attitude.rot_x(math.pi)
                # 要将ins1000的body系转到vins下的body系。此处是围绕x轴转180°.
                c_ib_vb = attitude.rot_x(math.pi)
                # ins1000在VINS导航系下的姿态矩阵c_bv，注意此时还没有yaw对齐
                # c_bv1 = c_bn.dot(c_vn.T)
                c_bv = c_ib_vb.dot(c_bn.dot(c_vn.T))
                ins1000_euler = attitude.dcm2euler(c_bv)
                # ins1000的yaw不准，需要增加一个delta_yaw项做人工对齐。
                # c_vn = attitude.rot_z(algo_yaw - ins1000_euler[0] + delta_yaw).dot(c_vn)
                c_vn = attitude.rot_z(ins1000_euler[0] - algo_yaw).dot(c_vn)
                t_vn = np.array([algo_x, algo_y, algo_z])

                c_ne = attitude.ecef_to_ned(start_lla[0], start_lla[1]) # DCM of efef to ned
                ins1000_pos_ecef = geoparams.lla2ecef(start_lla).reshape(1,3)

                c_ev = c_vn.dot(c_ne).T
            # else:
            #     ins1000_pos_ecef = np.vstack((ins1000_pos_ecef,geoparams.lla2ecef(np.array([gt_lat,gt_lon,gt_alt]))))

            # ins1000_pos_ned = c_ne.dot(ins1000_pos_ecef[-1] - ins1000_pos_ecef[0])
            # ins1000_pos_v = c_vn.dot(ins1000_pos_ned) + t_vn
            # ins1000_rpy_v = attitude.dcm2euler(c_ib_vb.dot(attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll])).dot(c_vn.T)))
            # ins1000_vel_v = attitude.rot_x(math.pi).dot(attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll])).dot(np.array([gt_v_x, gt_v_y, gt_v_z])))

            algo_ecef = c_ev.dot((np.array([algo_x, algo_y, algo_z]) - t_vn)) + ins1000_pos_ecef[0]
            algo_lla = geoparams.ecef2lla(algo_ecef)
            # algo的yaw转为ned系的heading。
            algo_att = attitude.dcm2euler(attitude.rot_x(math.pi).dot(attitude.euler2dcm(np.array([algo_yaw, algo_pitch, algo_roll]))).dot(c_vn))

            str1 = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9}\n'.format(str(timestamp),\
                algo_x, algo_y, algo_z, algo_roll*R2D, algo_pitch*R2D, algo_yaw*R2D, \
                algo_lla[0]*R2D, algo_lla[1]*R2D, algo_lla[2])

            output_file.write(str1)

            if algo_llas.shape[0] == 0:
                algo_llas = np.array([algo_lla])
                algo_headings = np.array([algo_att[0]*R2D])
            else:
                algo_llas = np.append(algo_llas, np.array([algo_lla]),axis=0)
                algo_headings = np.append(algo_headings, np.array([algo_att[0]*R2D]))

    output_file.close()
    kml_path = os.path.abspath('./kmlpath')
    if not os.path.exists(kml_path):
        os.makedirs(kml_path)
    # kml_gen.kml_gen(kml_path, gt_llas, gt_headings, 'ground_truth', color='ff00ff00')
    kml_gen.kml_gen(kml_path, algo_llas, algo_headings, 'algo', color='ff00ffff')
    print("time length:{0:0.3f}s".format(timestamp - first_timestamp))

def eval_rmse(align_file):
    algo_x_list = []
    algo_y_list = []
    algo_z_list = []
    algo_vel_x_list = []
    algo_vel_y_list = []
    algo_vel_z_list = []
    algo_roll_list= []
    algo_pitch_list= []
    algo_yaw_list = []

    align_x_list= []
    align_y_list= []
    align_z_list = []
    align_vel_x_list= []
    align_vel_y_list= []
    align_vel_z_list = []
    align_roll_list= []
    align_pitch_list= []
    align_yaw_list = []  

    with open(align_file,'r') as csv_file:
        align_reader = csv.reader(csv_file)

        for idx,item in enumerate(align_reader):
            if 0 == idx: continue

            algo_x_list.append(float(item[10]))
            algo_y_list.append(float(item[11]))
            algo_z_list.append(float(item[12]))
            algo_roll_list.append(float(item[13]))
            algo_pitch_list.append(float(item[14]))
            algo_yaw_list.append(float(item[15]))
            algo_vel_x_list.append(float(item[16]))
            algo_vel_y_list.append(float(item[17]))
            algo_vel_z_list.append(float(item[18]))

            align_x_list.append(float(item[19]))
            align_y_list.append(float(item[20]))
            align_z_list.append(float(item[21]))
            align_vel_x_list.append(float(item[28]))
            align_vel_y_list.append(float(item[29]))
            align_vel_z_list.append(float(item[30]))
            align_roll_list.append(float(item[25])) #注意：使用手动减去offset后的yaw pitch roll.
            align_pitch_list.append(float(item[26]))
            align_yaw_list.append(float(item[27]))

    algo_x_list = np.array(algo_x_list)
    algo_y_list = np.array(algo_y_list)
    algo_z_list = np.array(algo_z_list)
    algo_vel_x_list = np.array(algo_vel_x_list)
    algo_vel_y_list = np.array(algo_vel_y_list)
    algo_vel_z_list = np.array(algo_vel_z_list)
    algo_roll_list = np.array(algo_roll_list)
    algo_pitch_list = np.array(algo_pitch_list)
    algo_yaw_list = np.array(algo_yaw_list)
    align_x_list = np.array(align_x_list)
    align_y_list = np.array(align_y_list)
    align_z_list = np.array(align_z_list)
    align_vel_x_list = np.array(align_vel_x_list)
    align_vel_y_list = np.array(align_vel_y_list)
    align_vel_z_list = np.array(align_vel_z_list)
    align_roll_list = np.array(align_roll_list)
    align_pitch_list = np.array(align_pitch_list)
    align_yaw_list = np.array(align_yaw_list)

    rmse_x = cal_rmse(algo_x_list, align_x_list)
    rmse_y = cal_rmse(algo_y_list, align_y_list)
    rmse_z = cal_rmse(algo_z_list, align_z_list)
    rmse_vel_x = cal_rmse(algo_vel_x_list, align_vel_x_list)
    rmse_vel_y = cal_rmse(algo_vel_y_list, align_vel_y_list)
    rmse_vel_z = cal_rmse(algo_vel_z_list, align_vel_z_list)
    rmse_roll = cal_rmse(algo_roll_list, align_roll_list)
    rmse_pitch = cal_rmse(algo_pitch_list, align_pitch_list)
    rmse_yaw = cal_rmse(algo_yaw_list, align_yaw_list, True)

    rmse_pose = math.sqrt(rmse_x*rmse_x + rmse_y*rmse_y + rmse_z*rmse_z)
    rmse_vel = math.sqrt(rmse_vel_x*rmse_vel_x + rmse_vel_y*rmse_vel_y + rmse_vel_z*rmse_vel_z)
    rmse_attitude = math.sqrt(rmse_roll*rmse_roll + rmse_pitch*rmse_pitch + rmse_yaw*rmse_yaw)

    print("rmse_pose_x:{0}".format(rmse_x))
    print("rmse_pose_y:{0}".format(rmse_y))
    print("rmse_pose_z:{0}".format(rmse_z))
    print("rmse_pose:{0}".format(rmse_pose))

    print("rmse_vel_x:{0}".format(rmse_vel_x))
    print("rmse_vel_y:{0}".format(rmse_vel_y))
    print("rmse_vel_z:{0}".format(rmse_vel_z))
    print("rmse_vel:{0}".format(rmse_vel))

    print("rmse_roll:{0}".format(rmse_roll))
    print("rmse_pitch:{0}".format(rmse_pitch))
    print("rmse_yaw:{0}".format(rmse_yaw))
    print("rmse_attitude:{0}".format(rmse_attitude))

def cal_rmse(x, y, yaw=False):
    if x.size != y.size:
        raise ValueError("Size of is incorrect!")

    e = x - y
    if yaw:
        for i in range(e.size):
            e[i] = attitude.angle_range_pi(e[i])

    e2 = np.multiply(e, e) #equal to: pow(e,2) or e**2  
    rmse = math.sqrt(e2.sum()/e.size) # 'e2.sum()' equal to 'np.sum(e2)', 'math.sqrt' equal to '**0.5'
    return rmse

def cal_attitude(q0, q1, q2, q3):
    '''
    (q0, q1, q2, q3) are (Scalar, X, Y, Z) respective.
    Note that the quaternion in ROS is defined as (X, Y, Z, scalar). 

    return: (roll, pitch, heading), unit: radians
    '''
    q0_2 = q0 * q0
    q1_2 = q1 * q1
    q2_2 = q2 * q2
    q3_2 = q3 * q3

    c11 = q0_2 + q1_2 - q2_2 - q3_2
    c21 = 2.0 * (q1 * q2 + q0 * q3)
    c31 = 2.0 * (q1 * q3 - q0 * q2)
    c32 = 2.0 * (q2 * q3 + q0 * q1)
    c33 = q0_2 - q1_2 - q2_2 + q3_2

    roll = 0.0
    pitch = 0.0
    heading = 0.0

    if abs(c31) < 0.9999:
        pitch = math.atan(-1*c31 / math.sqrt(c32 * c32 + c33 * c33))
        roll = math.atan2(c32, c33)
        heading = math.atan2(c21, c11)
    return (roll, pitch, heading)

def show_help_info():
    str = '''
    Usage: 

    Options:

    -h,                     show this help message and exit
    -v ground_truth_file vins_result_file
                            save aligned data of ground-truth and vins algo result, 
                            and save aligned data to a csv file which set by -o option.
                            eg: python e.py -o "align.csv" -s "data.csv" "vins.csv"
    -l ground_truth_file loop_result_file
                            save aligned data of ground-truth and loop fusion algo result.
                            and save aligned data to a csv file which set by -o option.
                            eg: python e.py -o "align.csv" -s "data.csv" "loop.csv"
    -o,                     output file name, only valid with -v or -l option
    -r aligned_file         calculate RMSE by aligned data generated by -v or -l option above.
                            eg: python e.py -r "align.csv"
    '''
    print(str)

def main():
    # dataset = 'vio_1_[2019-10-8]'
    # ground_truth_file = 'ins1000/CNM-20191008_152547.csv'
    # algo_result_file = './data_2019_10_8/algo/vio_1_[2019-10-8].csv'
    # delta_yaw = -0.097

    # ground_truth_file = 'ins1000/CNM-20191008_154144.csv'
    # algo_result_file = './data_2019_10_8/algo/vio_2_[2019-10-8].csv'
    # delta_yaw = -0.07

    # ground_truth_file = 'ins1000/2019-10-15/CNM-20191014_131445.csv' # -0.073
    # algo_result_file = './data_2019_10_15/vio_company_20hz_roof_1.csv'

    # ground_truth_file = 'ins1000/2019-10-15/CNM-20191015_094301.csv' #0.106
    # algo_result_file = './data_2019_10_15/vio_company_15hz_hood_2.csv'
    # delta_yaw = 0.106

    # ground_truth_file = 'ins1000/2019-10-15/CNM-20191015_095239.csv' #-0.035
    # algo_result_file = './data_2019_10_15/vio_company_20hz_hood_2.csv'

    # ground_truth_file = 'ins1000/2019-10-15/CNM-20191015_103542.csv' #-0.057
    # algo_result_file = './data_2019_10_15/vio_road_20hz_hood_2.csv'
    # delta_yaw = -0.057

    # algo_result_file = 'vio_10-23_company_walk_side_15hz_2_1.csv'
    # start_lla = np.array([31.50769096, 120.4017507, 0]) #[Lat Lon Alt]
    # start_att = np.array([103, 0, 0]) #[yaw, pitch, roll] deg
    # align_flie = 'align.csv'
    # convert_vio2kml(align_flie, algo_result_file, start_lla, start_att)

    # align_flie = './data_2019_10_15/align_2019-10-15_road_hood_15hz-Demo.csv'
    # algo_result_file = 'vio_loop_2.csv'
    algo_result_file = 'vio_loop_10-23_company_walk_side_15hz_2_1.csv'
    start_lla = np.array([31.50769096, 120.4017507, 0]) #[Lat Lon Alt]
    start_att = np.array([104.7, 0, 0]) #[yaw, pitch, roll] deg
    align_flie = 'align.csv'
    convert_loop2kml(align_flie, algo_result_file, start_lla, start_att)
    
    # eval_rmse(align_flie)


if __name__=="__main__":
    print('start:{0}'.format(datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S')))
    time_start=time.time()
    main()
    time_end=time.time()
    print('Time cost:{0:0.2f}s'.format(time_end-time_start))
    print('end:{0}'.format(datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S')))
