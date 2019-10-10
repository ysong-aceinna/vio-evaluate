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
def save_aligned_data_vins(output_align_file, ground_truth_file, algo_result_file, delta_yaw):
    output_file = open(output_align_file, 'a+')
    output_file.write("time,\
        ins1000_lat, ins1000_lon, ins1000_alt, ins1000_roll, ins1000_pitch, ins1000_yaw, \
        ins1000_ned_x, ins1000_ned_y, ins1000_ned_z, \
        algo_pose_x, algo_pose_y, algo_pose_z, algo_roll, algo_pitch,algo_yaw,\
        align_pose_x, align_pose_y, align_pose_z, align_roll,a lign_pitch, align_yaw,\
        algo_lat, algo_lon, algo_alt \n".replace(" ", ""))

    cur_line = 1
    R2D = 180/math.pi
    d_x = d_y = d_z = 0
    d_yaw = d_pitch = d_roll = 0
    b_align = False
    first_timestamp = 0
    timestamp = 0
    time_delta = 0.015 #15ms
    ins1000_pos_ecef = np.array([])
    ins1000_pos_v = np.array([])

    #get row count of ground-truth file.
    with open(ground_truth_file,'r') as csv_file:
        ground_truth_reader = csv.reader(csv_file)
        row_count = sum(1 for row in ground_truth_reader)

    with open(algo_result_file,'r') as csv_file:
        algo_result_reader = csv.reader(csv_file)

        for idx,item in enumerate(algo_result_reader):
            if 0 == idx: continue

            timestamp = float(item[0][:-6])/1000  # ms to sec
            with open(ground_truth_file,'r') as csv_file:
                ground_truth_reader = csv.reader(csv_file)

                for _idx,_item in enumerate(ground_truth_reader):
                    if _idx < cur_line: continue

                    if abs(float(_item[0]) - timestamp) > time_delta: 
                        if _idx == row_count - 1:
                            print('[{0}]:can not find same timestamp {1} in ground-truth.'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), timestamp))
                        else: 
                            continue
                    else:
                        cur_line = _idx
                        gt_lat = float(_item[2])/R2D
                        gt_lon = float(_item[3])/R2D
                        gt_alt = float(_item[4])
                        gt_qw = float(_item[8])
                        gt_qx = float(_item[9])
                        gt_qy = float(_item[10])
                        gt_qz = float(_item[11])

                        algo_x = float(item[5])
                        algo_y = float(item[6])
                        algo_z = float(item[7])
                        algo_qx = float(item[8])
                        algo_qy = float(item[9])
                        algo_qz = float(item[10])
                        algo_qw = float(item[11])

                        (gt_yaw, gt_pitch, gt_roll) = attitude.quat2euler(np.array([gt_qw, gt_qx, gt_qy, gt_qz]))
                        (algo_yaw, algo_pitch, algo_roll) = attitude.quat2euler(np.array([algo_qw, algo_qx, algo_qy, algo_qz]))

                        if not b_align:
                            b_align = True
                            first_timestamp = timestamp

                            # ins1000在NED下的姿态矩阵c_bn
                            c_bn = attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll]))
                            # NED转VINS的导航系，注意此时还没有yaw对齐。
                            c_vn = attitude.rot_x(math.pi)
                            # 要将ins1000的body系转到vins下的body系。此处是围绕x轴转180°.
                            c_ib_vb = attitude.rot_x(math.pi)
                            # ins1000在VINS导航系下的姿态矩阵c_bv，注意此时还没有yaw对齐
                            c_bv = c_ib_vb.dot(c_bn.dot(c_vn.T))
                            ins1000_euler = attitude.dcm2euler(c_bv)
                            # ins1000在VINS导航系下的姿态矩阵c_bv，注意此时还没有yaw对齐
                            c_vn = attitude.rot_z(ins1000_euler[0] - algo_yaw + delta_yaw).dot(c_vn)
                            t_vn = np.array([algo_x, algo_y, algo_z])

                            c_ne = attitude.ecef_to_ned(gt_lat,gt_lon) # DCM of efef to ned
                            ins1000_pos_ecef = geoparams.lla2ecef(np.array([gt_lat,gt_lon,gt_alt])).reshape(1,3)

                            c_ev = c_vn.dot(c_ne).T
                        else:
                            ins1000_pos_ecef = np.vstack((ins1000_pos_ecef,geoparams.lla2ecef(np.array([gt_lat,gt_lon,gt_alt]))))

                        ins1000_pos_ned = c_ne.dot(ins1000_pos_ecef[-1] - ins1000_pos_ecef[0])
                        ins1000_pos_v = c_vn.dot(ins1000_pos_ned) + t_vn
                        ins1000_rpy_v = attitude.dcm2euler(c_ib_vb.dot(attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll])).dot(c_vn.T)))
                        
                        # print("{0:0.3f},{1},{2},{3}".format(timestamp,ins1000_pos_ned[0],ins1000_pos_ned[1],ins1000_pos_ned[2]))
                        # print("{0},{1},{2}".format(ins1000_pos_ned[0],ins1000_pos_ned[1],ins1000_pos_ned[2]))
                        
                        algo_ecef = c_ev.dot((np.array([algo_x, algo_y, algo_z]) - t_vn)) + ins1000_pos_ecef[0]
                        algo_lla = geoparams.ecef2lla(algo_ecef)

                        str1 = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19},{20},{21},{22},{23},{24}\n'.format(str(timestamp),\
                            gt_lat*R2D, gt_lon*R2D, gt_alt*R2D, gt_roll*R2D, gt_pitch*R2D, gt_yaw*R2D, \
                            ins1000_pos_ned[0], ins1000_pos_ned[1], ins1000_pos_ned[2], \
                            algo_x, algo_y, algo_z, algo_roll*R2D, algo_pitch*R2D, algo_yaw*R2D, \
                            ins1000_pos_v[0], ins1000_pos_v[1], ins1000_pos_v[2], ins1000_rpy_v[2]*R2D, ins1000_rpy_v[1]*R2D, ins1000_rpy_v[0]*R2D, \
                            algo_lla[0]*R2D, algo_lla[1]*R2D, algo_lla[2])

                        output_file.write(str1)
                        break
    output_file.close()
    print("time length:{0:0.3f}s".format(timestamp - first_timestamp))


def eval_rmse(align_file):
    algo_x_list = []
    algo_y_list = []
    algo_z_list = []
    algo_roll_list= []
    algo_pitch_list= []
    algo_yaw_list = []

    align_x_list= []
    align_y_list= []
    align_z_list = []
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

            align_x_list.append(float(item[16]))
            align_y_list.append(float(item[17]))
            align_z_list.append(float(item[18]))
            align_roll_list.append(float(item[22])) #注意：使用手动减去offset后的yaw pitch roll.
            align_pitch_list.append(float(item[23]))
            align_yaw_list.append(float(item[24]))

    algo_x_list = np.array(algo_x_list)
    algo_y_list = np.array(algo_y_list)
    algo_z_list = np.array(algo_z_list)
    algo_roll_list = np.array(algo_roll_list)
    algo_pitch_list = np.array(algo_pitch_list)
    algo_yaw_list = np.array(algo_yaw_list)
    align_x_list = np.array(align_x_list)
    align_y_list = np.array(align_y_list)
    align_z_list = np.array(align_z_list)
    align_roll_list = np.array(align_roll_list)
    align_pitch_list = np.array(align_pitch_list)
    align_yaw_list = np.array(align_yaw_list)

    rmse_x = cal_rmse(algo_x_list, align_x_list)
    rmse_y = cal_rmse(algo_y_list, align_y_list)
    rmse_z = cal_rmse(algo_z_list, align_z_list)
    rmse_roll = cal_rmse(algo_roll_list, align_roll_list)
    rmse_pitch = cal_rmse(algo_pitch_list, align_pitch_list)
    rmse_yaw = cal_rmse(algo_yaw_list, align_yaw_list, True)

    rmse_pose = math.sqrt(rmse_x*rmse_x + rmse_y*rmse_y + rmse_z*rmse_z)
    rmse_attitude = math.sqrt(rmse_roll*rmse_roll + rmse_pitch*rmse_pitch + rmse_yaw*rmse_yaw)

    print("rmse_pose_x:{0}".format(rmse_x))
    print("rmse_pose_y:{0}".format(rmse_y))
    print("rmse_pose_z:{0}".format(rmse_z))
    print("rmse_pose:{0}".format(rmse_pose))

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

def main(argv):
    output_align_file = ''
    align_file = ''
    ground_truth_file = '' 
    algo_result_file = ''
    dataset = ''
    try:
        opts, args = getopt.getopt(argv, "hvlr:", [])
        if 0 == len(opts):
            show_help_info()    
            sys.exit(0)

        for opt, arg in opts:
            if opt in ("-h",):
                show_help_info()
                sys.exit(0)
            elif opt in ("-v",):
                output_align_file = args[0]
                ground_truth_file = args[1]
                algo_result_file = args[2]
                dataset = args[3]
                save_aligned_data_vins(output_align_file, ground_truth_file, algo_result_file, dataset)
                break
            elif opt in ("-l",):
                output_align_file = args[0]
                ground_truth_file = args[1]
                algo_result_file = args[2]
                dataset = args[3]
                save_aligned_data_loop(output_align_file, ground_truth_file, algo_result_file, dataset)
                break 
            elif opt in ("-r",):
                align_file = arg
                eval_rmse(align_file)
                break  
            else:
                show_help_info()
                sys.exit()
    except getopt.GetoptError as e:
        print(e.msg)
        sys.exit(1)


if __name__=="__main__":
    print('start:{0}'.format(datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S')))
    time_start=time.time()

    # main(sys.argv[1:])

    # dataset = 'vio_1_[2019-10-8]'
    # ground_truth_file = 'ins1000/CNM-20191008_152547.csv'
    # algo_result_file = 'algo/vio_1_[2019-10-8].csv'
    # delta_yaw = -0.097

    # ground_truth_file = 'ins1000/CNM-20191008_154144.csv'
    # algo_result_file = 'algo/vio_2_[2019-10-8].csv'
    # delta_yaw = -0.07

    align_flie = 'align_vio_2_[2019-10-8].csv'
    # 先用prepare_data只做时间对齐，matlab分析align.csv得到转换矩阵后，再用save_aligned_data_vins做数据对齐。
    # prepare_data('align.csv', ground_truth_file, algo_result_file)
    # save_aligned_data_vins(align_flie, ground_truth_file, algo_result_file, delta_yaw)
    
    eval_rmse(align_flie)

    time_end=time.time()
    print('Time cost:{0:0.2f}s'.format(time_end-time_start))
    print('end:{0}'.format(datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S')))

'''
1. 把algo的位姿，ins1000的lla,ned下的位姿保存到csv，利用Matlab来计算ins1000 ned到对应的algo导航系的转换矩阵。
2. 用save_aligned_data_vins0根据上步得到的装换矩阵，做数据对齐。
3. 这种方式并不能确保起始点对齐，只是让RMSE好看些，并不能评估VINS实际的性能，所以prepare_data和save_aligned_data_vins0都废弃不用。
'''
def prepare_data(output_align_file, ground_truth_file, algo_result_file):
    output_file = open(output_align_file, 'a+')
    output_file.write("time,\
        ins1000_lat, ins1000_lon, ins1000_alt, ins1000_roll, ins1000_pitch, ins1000_yaw, \
        ins1000_ned_x, ins1000_ned_y, ins1000_ned_z, \
        algo_pose_x, algo_pose_y, algo_pose_z, algo_roll, algo_pitch,algo_yaw\n".replace(" ", ""))

    cur_line = 1
    R2D = 180/math.pi
    d_x = d_y = d_z = 0
    d_yaw = d_pitch = d_roll = 0
    b_align = False
    first_timestamp = 0
    timestamp = 0
    time_delta = 0.012 #12ms
    ins1000_pos_ecef = np.array([])
    ins1000_pos_v = np.array([])

    #get row count of ground-truth file.
    with open(ground_truth_file,'r') as csv_file:
        ground_truth_reader = csv.reader(csv_file)
        row_count = sum(1 for row in ground_truth_reader)

    with open(algo_result_file,'r') as csv_file:
        algo_result_reader = csv.reader(csv_file)

        for idx,item in enumerate(algo_result_reader):
            if 0 == idx: continue

            timestamp = float(item[0][:-6])/1000  # ms to sec
            with open(ground_truth_file,'r') as csv_file:
                ground_truth_reader = csv.reader(csv_file)

                for _idx,_item in enumerate(ground_truth_reader):
                    if _idx < cur_line: continue

                    if abs(float(_item[0]) - timestamp) > time_delta: 
                        if _idx == row_count - 1:
                            print('[{0}]:can not find same timestamp {1} in ground-truth.'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), timestamp))
                        else: 
                            continue
                    else:
                        cur_line = _idx
                        gt_lat = float(_item[2])/R2D
                        gt_lon = float(_item[3])/R2D
                        gt_alt = float(_item[4])
                        gt_qw = float(_item[8])
                        gt_qx = float(_item[9])
                        gt_qy = float(_item[10])
                        gt_qz = float(_item[11])

                        algo_x = float(item[5])
                        algo_y = float(item[6])
                        algo_z = float(item[7])
                        algo_qx = float(item[8])
                        algo_qy = float(item[9])
                        algo_qz = float(item[10])
                        algo_qw = float(item[11])

                        (gt_yaw, gt_pitch, gt_roll) = attitude.quat2euler(np.array([gt_qw, gt_qx, gt_qy, gt_qz]))
                        (algo_yaw, algo_pitch, algo_roll) = attitude.quat2euler(np.array([algo_qw, algo_qx, algo_qy, algo_qz]))

                        if not b_align:
                            b_align = True
                            first_timestamp = timestamp

                            # ins1000在NED下的姿态矩阵c_bn
                            c_bn = attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll]))
                            # NED转VINS的导航系，注意此时还没有yaw对齐。
                            c_vn = attitude.rot_x(math.pi)
                            # ins1000在VINS导航系下的姿态矩阵c_bv，注意此时还没有yaw对齐
                            c_bv = c_bn.dot(c_vn.T)
                            ins1000_euler = attitude.dcm2euler(c_bv)
                            # ins1000在VINS导航系下的姿态矩阵c_bv，注意此时还没有yaw对齐
                            c_vn = attitude.rot_z(ins1000_euler[0] - algo_yaw).dot(c_vn)
                            t_vn = np.array([algo_x, algo_y, algo_z])

                            c_ne = attitude.ecef_to_ned(gt_lat,gt_lon) # DCM of efef to ned
                            ins1000_pos_ecef = geoparams.lla2ecef(np.array([gt_lat,gt_lon,gt_alt])).reshape(1,3)

                            c_ev = c_vn.dot(c_ne).T
                        else:
                            ins1000_pos_ecef = np.vstack((ins1000_pos_ecef,geoparams.lla2ecef(np.array([gt_lat,gt_lon,gt_alt]))))

                        ins1000_pos_ned = c_ne.dot(ins1000_pos_ecef[-1] - ins1000_pos_ecef[0])
                        ins1000_pos_v = c_vn.dot(ins1000_pos_ned) + t_vn
                        ins1000_rpy_v = attitude.dcm2euler(attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll])).dot(c_vn.T))
                        # print("{0:0.3f},{1},{2},{3}".format(timestamp,ins1000_pos_ned[0],ins1000_pos_ned[1],ins1000_pos_ned[2]))
                        # print("{0},{1},{2}".format(ins1000_pos_ned[0],ins1000_pos_ned[1],ins1000_pos_ned[2]))
                        
                        algo_ecef = c_ev.dot((np.array([algo_x, algo_y, algo_z]) - t_vn)) + ins1000_pos_ecef[0]
                        algo_lla = geoparams.ecef2lla(algo_ecef)*R2D

                        str1 = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15}\n'.format(str(timestamp),\
                            gt_lat*R2D, gt_lon*R2D, gt_alt*R2D, gt_roll*R2D, gt_pitch*R2D, gt_yaw*R2D, \
                            ins1000_pos_ned[0], ins1000_pos_ned[1], ins1000_pos_ned[2], \
                            algo_x, algo_y, algo_z, algo_roll*R2D, algo_pitch*R2D, algo_yaw*R2D)

                        output_file.write(str1)
                        break
    output_file.close()
    print("time length:{0:0.3f}s".format(timestamp - first_timestamp))

def save_aligned_data_vins0(output_align_file, ground_truth_file, algo_result_file, dataset):
    output_file = open(output_align_file, 'a+')
    output_file.write("time,\
        ins1000_lat, ins1000_lon, ins1000_alt, ins1000_roll, ins1000_pitch, ins1000_yaw, \
        ins1000_ned_x, ins1000_ned_y, ins1000_ned_z, \
        algo_pose_x, algo_pose_y, algo_pose_z, algo_roll, algo_pitch,algo_yaw,\
        align_pose_x, align_pose_y, align_pose_z, align_roll,a lign_pitch, align_yaw,\
        algo_lat, algo_lon, algo_alt \n".replace(" ", ""))

    cur_line = 1
    R2D = 180/math.pi
    d_x = d_y = d_z = 0
    d_yaw = d_pitch = d_roll = 0
    b_align = False
    first_timestamp = 0
    timestamp = 0
    time_delta = 0.012 #12ms
    ins1000_pos_ecef = np.array([])
    ins1000_pos_v = np.array([])

    cfg = yaml.load(open('/Users/songyang/project/code/github/vio-evaluate/ins1000.yaml'))
    tm_ned2vins = cfg[dataset]['NED2VINS'] #shape:[3,4]
    tm_ned2vins = np.array(tm_ned2vins).reshape(3,4)
    dcm_ned2vins = np.split(tm_ned2vins,[3],1)[0] #取旋转矩阵，即转换矩阵中开始的3X3元素。

    tm_vins2ned = cfg[dataset]['VINS2NED'] #shape:[3,4]
    tm_vins2ned = np.array(tm_vins2ned).reshape(3,4)
    dcm_vins2ned = np.split(tm_vins2ned,[3],1)[0] #取旋转矩阵，即转换矩阵中开始的3X3元素。

    #get row count of ground-truth file.
    with open(ground_truth_file,'r') as csv_file:
        ground_truth_reader = csv.reader(csv_file)
        row_count = sum(1 for row in ground_truth_reader)

    with open(algo_result_file,'r') as csv_file:
        algo_result_reader = csv.reader(csv_file)

        for idx,item in enumerate(algo_result_reader):
            if 0 == idx: continue

            timestamp = float(item[0][:-6])/1000  # ms to sec
            with open(ground_truth_file,'r') as csv_file:
                ground_truth_reader = csv.reader(csv_file)

                for _idx,_item in enumerate(ground_truth_reader):
                    if _idx < cur_line: continue

                    if abs(float(_item[0]) - timestamp) > time_delta: 
                        if _idx == row_count - 1:
                            print('[{0}]:can not find same timestamp {1} in ground-truth.'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), timestamp))
                        else: 
                            continue
                    else:
                        cur_line = _idx
                        gt_lat = float(_item[2])/R2D
                        gt_lon = float(_item[3])/R2D
                        gt_alt = float(_item[4])
                        gt_qw = float(_item[8])
                        gt_qx = float(_item[9])
                        gt_qy = float(_item[10])
                        gt_qz = float(_item[11])

                        algo_x = float(item[5])
                        algo_y = float(item[6])
                        algo_z = float(item[7])
                        algo_qx = float(item[8])
                        algo_qy = float(item[9])
                        algo_qz = float(item[10])
                        algo_qw = float(item[11])

                        (gt_yaw, gt_pitch, gt_roll) = attitude.quat2euler(np.array([gt_qw, gt_qx, gt_qy, gt_qz]))
                        (algo_yaw, algo_pitch, algo_roll) = attitude.quat2euler(np.array([algo_qw, algo_qx, algo_qy, algo_qz]))

                        if not b_align:
                            b_align = True
                            first_timestamp = timestamp

                            c_ne = attitude.ecef_to_ned(gt_lat,gt_lon) # DCM of efef to ned
                            ins1000_pos_ecef = geoparams.lla2ecef(np.array([gt_lat,gt_lon,gt_alt])).reshape(1,3)
                        else:
                            ins1000_pos_ecef = np.vstack((ins1000_pos_ecef,geoparams.lla2ecef(np.array([gt_lat,gt_lon,gt_alt]))))

                        ins1000_pos_ned = c_ne.dot(ins1000_pos_ecef[-1] - ins1000_pos_ecef[0])
                        ins1000_pos_v = tm_ned2vins.dot(np.append(ins1000_pos_ned,1))
                        ins1000_rpy_v = attitude.dcm2euler(attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll])).dot(dcm_vins2ned))

                        # ins1000_rpy_v = attitude.dcm2euler(attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll])).dot(c_vn.T))
                        # print("{0:0.3f},{1},{2},{3}".format(timestamp,ins1000_pos_ned[0],ins1000_pos_ned[1],ins1000_pos_ned[2]))
                        # print("{0},{1},{2}".format(ins1000_pos_ned[0],ins1000_pos_ned[1],ins1000_pos_ned[2]))
                        
                        algo_ecef = c_ne.T.dot(tm_vins2ned.dot(np.array([algo_x, algo_y, algo_z, 1]))) + ins1000_pos_ecef[0]
                        algo_lla = geoparams.ecef2lla(algo_ecef)

                        str1 = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19},{20},{21},{22},{23},{24}\n'.format(str(timestamp),\
                            gt_lat*R2D, gt_lon*R2D, gt_alt*R2D, gt_roll*R2D, gt_pitch*R2D, gt_yaw*R2D, \
                            ins1000_pos_ned[0], ins1000_pos_ned[1], ins1000_pos_ned[2], \
                            algo_x, algo_y, algo_z, algo_roll*R2D, algo_pitch*R2D, algo_yaw*R2D, \
                            ins1000_pos_v[0], ins1000_pos_v[1], ins1000_pos_v[2], ins1000_rpy_v[2]*R2D, ins1000_rpy_v[1]*R2D, ins1000_rpy_v[0]*R2D, \
                            algo_lla[0]*R2D, algo_lla[1]*R2D, algo_lla[2]*R2D)

                        output_file.write(str1)
                        break
    output_file.close()
    print("time length:{0:0.3f}s".format(timestamp - first_timestamp))

