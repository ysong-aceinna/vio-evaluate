import sys
import os
import datetime
import time
import math
import numpy as np
import csv
import getopt


def save_aligned_data_vins(output_align_file, ground_truth_file, algo_result_file):
    output_file = open(output_align_file, 'a+')
    output_file.write("\
        time,ref_pose_x,ref_pose_y,ref_pose_z,\
        ref_roll,ref_pitch,ref_yaw,\
        algo_pose_x,algo_pose_y,algo_pose_z,\
        algo_roll,algo_pitch,algo_yaw,\
        align_pose_x,align_pose_y,align_pose_z,\
        align_roll,align_pitch,align_yaw\n")

    cur_line = 1
    r2d = 180/math.pi
    d_x = d_y = d_z = 0
    d_yaw = d_pitch = d_roll = 0
    b_align = False
    first_timestamp = 0
    timestamp = 0

    with open(ground_truth_file,'r') as csv_file:
        ground_truth_reader = csv.reader(csv_file)
        row_count = sum(1 for row in ground_truth_reader)

    with open(algo_result_file,'r') as csv_file:
        algo_result_reader = csv.reader(csv_file)

        for idx,item in enumerate(algo_result_reader):
            if 0 == idx: continue
            # if 11 == idx: break

            timestamp = item[0][:-6]

            with open(ground_truth_file,'r') as csv_file:
                ground_truth_reader = csv.reader(csv_file)

                for _idx,_item in enumerate(ground_truth_reader):
                    if _idx < cur_line: continue

                    if _item[0][:-6] != timestamp: 
                        if _idx == row_count - 1:
                            print('[{0}]:can not find same timestamp {1} in ground-truth.'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), timestamp))
                        else: 
                            continue
                    else:
                        cur_line = _idx
                        gt_x = float(_item[1])
                        gt_y = float(_item[2])
                        gt_z = float(_item[3])
                        gt_qw = float(_item[4])
                        gt_qx = float(_item[5])
                        gt_qy = float(_item[6])
                        gt_qz = float(_item[7])

                        algo_x = float(item[5])
                        algo_y = float(item[6])
                        algo_z = float(item[7])
                        algo_qx = float(item[8])
                        algo_qy = float(item[9])
                        algo_qz = float(item[10])
                        algo_qw = float(item[11])

                        (gt_roll, gt_pitch, gt_yaw) = cal_attitude(gt_qw, gt_qx, gt_qy, gt_qz)
                        gt_roll = gt_roll*r2d
                        gt_pitch = gt_pitch*r2d
                        gt_yaw = gt_yaw*r2d

                        (algo_roll, algo_pitch, algo_yaw) = cal_attitude(algo_qw, algo_qx, algo_qy, algo_qz)
                        algo_roll = algo_roll*r2d
                        algo_pitch = algo_pitch*r2d
                        algo_yaw = algo_yaw*r2d

                        if not b_align:
                            b_align = True
                            first_timestamp = timestamp
                            d_x = gt_x - algo_x
                            d_y = gt_y - algo_y
                            d_z = gt_z - algo_z
                            d_roll = gt_roll - algo_roll
                            d_pitch = gt_pitch - algo_pitch
                            d_yaw = gt_yaw - algo_yaw

                        align_x = algo_x + d_x
                        align_y = algo_y + d_y
                        align_z = algo_z + d_z
                        align_roll = algo_roll + d_roll
                        align_pitch = algo_pitch + d_pitch
                        align_yaw = algo_yaw + d_yaw

                        str = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},\n'.format(\
                            timestamp, gt_x, gt_y, gt_z, gt_roll, gt_pitch, gt_yaw,  \
                            algo_x, algo_y, algo_z, algo_roll, algo_pitch, algo_yaw,\
                            align_x, align_y, align_z, align_roll, align_pitch, align_yaw)
                        output_file.write(str)
                        break
    output_file.close()
    first_timestamp = float(first_timestamp)
    timestamp = float(timestamp)
    print("time length:{0:0.3f}s".format((timestamp - first_timestamp)/1000))


def save_aligned_data_loop(output_align_file, ground_truth_file, algo_loop_file):
    # output_align_file = 'align.csv'
    output_file = open(output_align_file, 'a+')
    output_file.write("\
        time,ref_pose_x,ref_pose_y,ref_pose_z,\
        ref_roll,ref_pitch,ref_yaw,\
        algo_pose_x,algo_pose_y,algo_pose_z,\
        algo_roll,algo_pitch,algo_yaw,\
        align_pose_x,align_pose_y,align_pose_z,\
        align_roll,align_pitch,align_yaw\n")

    cur_line = 1
    r2d = 180/math.pi
    d_x = d_y = d_z = 0
    d_yaw = d_pitch = d_roll = 0
    b_align = False
    ground_truth_row_count = 0
    loop_file_row_count = 0
    first_timestamp = 0
    timestamp = 0

    #get row count of ground-truth file.
    with open(ground_truth_file,'r') as csv_file:
        ground_truth_reader = csv.reader(csv_file)
        ground_truth_row_count = sum(1 for row in ground_truth_reader)

    #get row count of result file.
    with open(algo_loop_file,'r') as csv_file:
        algo_result_reader = csv.reader(csv_file)
        loop_file_row_count = sum(1 for row in algo_result_reader)

    #get the last row data which contains all point pose/attitude data on whole trojactory. 
    path = []
    with open(algo_loop_file,'r') as csv_file:
        algo_result_reader = csv.reader(csv_file)
        for idx,item in enumerate(algo_result_reader):
            if idx == loop_file_row_count-1: 
                path = item

    path = path[4:]
    num = 10
    loop_times = int(len(path)/num)
    for i in range(loop_times):
        timestamp = path[i*num + 1][:-6]

        with open(ground_truth_file,'r') as csv_file:
            ground_truth_reader = csv.reader(csv_file)

            for _idx,_item in enumerate(ground_truth_reader):
                if _idx < cur_line: continue

                if _item[0][:-6] != timestamp:
                    if _idx == ground_truth_row_count - 1:
                        print('[{0}]:can not find same timestamp {1} in ground-truth.'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), timestamp))
                    else: 
                        continue
                else:
                    cur_line = _idx
                    gt_x = float(_item[1])
                    gt_y = float(_item[2])
                    gt_z = float(_item[3])
                    gt_qw = float(_item[4])
                    gt_qx = float(_item[5])
                    gt_qy = float(_item[6])
                    gt_qz = float(_item[7])

                    algo_x = float(path[i*num + 3])
                    algo_y = float(path[i*num + 4])
                    algo_z = float(path[i*num + 5])
                    algo_qx = float(path[i*num + 6])
                    algo_qy = float(path[i*num + 7])
                    algo_qz = float(path[i*num + 8])
                    algo_qw = float(path[i*num + 9])

                    (gt_roll, gt_pitch, gt_yaw) = cal_attitude(gt_qw, gt_qx, gt_qy, gt_qz)
                    gt_roll = gt_roll*r2d
                    gt_pitch = gt_pitch*r2d
                    gt_yaw = gt_yaw*r2d

                    (algo_roll, algo_pitch, algo_yaw) = cal_attitude(algo_qw, algo_qx, algo_qy, algo_qz)
                    algo_roll = algo_roll*r2d
                    algo_pitch = algo_pitch*r2d
                    algo_yaw = algo_yaw*r2d

                    if not b_align:
                        b_align = True
                        first_timestamp = timestamp
                        d_x = gt_x - algo_x
                        d_y = gt_y - algo_y
                        d_z = gt_z - algo_z
                        d_roll = gt_roll - algo_roll
                        d_pitch = gt_pitch - algo_pitch
                        d_yaw = gt_yaw - algo_yaw

                    align_x = algo_x + d_x
                    align_y = algo_y + d_y
                    align_z = algo_z + d_z
                    align_roll = algo_roll + d_roll
                    align_pitch = algo_pitch + d_pitch
                    align_yaw = algo_yaw + d_yaw

                    str = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},\n'.format(\
                        timestamp, gt_x, gt_y, gt_z, gt_roll, gt_pitch, gt_yaw,  \
                        algo_x, algo_y, algo_z, algo_roll, algo_pitch, algo_yaw,\
                        align_x, align_y, align_z, align_roll, align_pitch, align_yaw)
                    output_file.write(str)
                    output_file.flush()
                    break
    output_file.close()

    first_timestamp = float(first_timestamp)
    timestamp = float(timestamp)
    print("time length:{0:0.3f}s".format((timestamp - first_timestamp)/1000))

def eval_rmse(align_file):
    gt_x_list = []
    gt_y_list = []
    gt_z_list = []
    gt_roll_list= []
    gt_pitch_list= []
    gt_yaw_list = []

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

            gt_x_list.append(float(item[1]))
            gt_y_list.append(float(item[2]))
            gt_z_list.append(float(item[3]))
            gt_roll_list.append(float(item[4]))
            gt_pitch_list.append(float(item[5]))
            gt_yaw_list.append(float(item[6]))

            align_x_list.append(float(item[13]))
            align_y_list.append(float(item[14]))
            align_z_list.append(float(item[15]))
            align_roll_list.append(float(item[16]))
            align_pitch_list.append(float(item[17]))
            align_yaw_list.append(float(item[18]))

    gt_x_list = np.array(gt_x_list)
    gt_y_list = np.array(gt_y_list)
    gt_z_list = np.array(gt_z_list)
    gt_roll_list = np.array(gt_roll_list)
    gt_pitch_list = np.array(gt_pitch_list)
    gt_yaw_list = np.array(gt_yaw_list)
    align_x_list = np.array(align_x_list)
    align_y_list = np.array(align_y_list)
    align_z_list = np.array(align_z_list)
    align_roll_list = np.array(align_roll_list)
    align_pitch_list = np.array(align_pitch_list)
    align_yaw_list = np.array(align_yaw_list)

    rmse_x = cal_rmse(gt_x_list, align_x_list)
    rmse_y = cal_rmse(gt_y_list, align_y_list)
    rmse_z = cal_rmse(gt_z_list, align_z_list)
    rmse_roll = cal_rmse(gt_roll_list, align_roll_list)
    rmse_pitch = cal_rmse(gt_pitch_list, align_pitch_list)
    rmse_yaw = cal_rmse(gt_yaw_list, align_yaw_list)

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

def cal_rmse(x, y):
    if x.size != y.size:
        raise ValueError("Size of IMU Rotation Matrix is incorrect!")

    e = x - y
    e2 = np.multiply(e, e) #equal to: pow(e,2)
    rmse = math.sqrt(e2.sum()/e.size)
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

    try:
        opts, args = getopt.getopt(argv, "hvlo:r:", [])
        if 0 == len(opts):
            show_help_info()    
            sys.exit(0)

        for opt, arg in opts:
            if opt in ("-h",):
                show_help_info()
                sys.exit(0)
            elif opt in ("-o",):
                output_align_file = arg
            elif opt in ("-v",):
                ground_truth_file = args[0]
                algo_result_file = args[1]
                save_aligned_data_vins(output_align_file, ground_truth_file, algo_result_file)
                break
            elif opt in ("-l",):
                ground_truth_file = args[0]
                algo_result_file = args[1]
                save_aligned_data_loop(output_align_file, ground_truth_file, algo_result_file)
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

    main(sys.argv[1:])
    # ground_truth_file = '/Users/songyang/Downloads/datasets/EuRoC_ASL/V1_02_medium_mav0/state_groundtruth_estimate0/data.csv'
    # algo_result_file = '/Users/songyang/project/code/ros/catkin_ws/test/loop4.csv'
    # save_aligned_data_loop('align.csv', ground_truth_file, algo_result_file)
    time_end=time.time()
    print('Time cost:{0:0.2f}s'.format(time_end-time_start))
    print('end:{0}'.format(datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S')))

