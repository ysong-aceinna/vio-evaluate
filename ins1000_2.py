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
R2D = 180/math.pi

#algo_result_file: vins algo的数据结果文件，是 vins保存在 /home/song/vins-fusion下的vio.csv。
def save_aligned_data_vio_and_loop(output_align_file, ground_truth_file, algo_result_file, delta_yaw, loop_result_file=None):
    output_file = open(output_align_file, 'a+')
    loop_header = ''
    str_header = "time,\
        ins1000_lat, ins1000_lon, ins1000_alt, ins1000_roll, ins1000_pitch, ins1000_yaw, \
        ins1000_ned_x, ins1000_ned_y, ins1000_ned_z, \
        algo_pose_x, algo_pose_y, algo_pose_z, algo_roll, algo_pitch,algo_yaw, algo_vel_x, algo_vel_y, algo_vel_z,\
        align_pose_x, align_pose_y, align_pose_z, align_roll,align_pitch, align_yaw, align_vel_x, align_vel_y, align_vel_z,\
        algo_lat, algo_lon, algo_alt"
    if loop_result_file is not None:
        loop_header = ",loop_pose_x, loop_pose_y, loop_pose_z, loop_lat, loop_lon, loop_alt, loop_roll, loop_pitch, loop_yaw"
    str_header = (str_header + loop_header + '\n').replace(" ", "")
    output_file.write(str_header)

    gt_cur_line = 1 #跳过第一行的表头
    loop_cur_line = 0
    b_align = False
    first_timestamp = 0
    timestamp = 0
    time_delta = 0.015 #15ms
    ins1000_pos_ecef = np.array([])
    ins1000_pos_v = np.array([])
    gt_llas = np.array([])
    gt_headings = np.array([])

    algo_llas = np.array([])
    algo_headings = np.array([])

    loop_llas = np.array([])
    loop_headings = np.array([])
    find_loop = False
    #get row count of ground-truth file.
    with open(ground_truth_file,'r') as csv_file:
        ground_truth_reader = csv.reader(csv_file)
        row_count = sum(1 for row in ground_truth_reader)

    #打开vio.csv
    with open(algo_result_file,'r') as csv_file: 
        algo_result_reader = csv.reader(csv_file)

        #遍历vio.csv
        for idx,item in enumerate(algo_result_reader): 
            if idx % 100 == 0: print("{0}\n".format(idx))
            if 0 == idx: continue
            # if 300 == idx: break
            timestamp = float(item[0][:-6])/1000  # ms to sec

            loop_x = 0
            loop_y = 0
            loop_z = 0
            loop_qx = 0
            loop_qy = 0
            loop_qz = 0
            loop_qw = 0
            loop_yaw = 0
            loop_pitch = 0
            loop_roll = 0

            if loop_result_file is not None:
                #打开vio_loop.csv
                with open(loop_result_file,'r') as csv_file: 
                    loop_reader = csv.reader(csv_file)
                    #遍历vio_loop.csv
                    for idx_lp,item_lp in enumerate(loop_reader): 
                        find_loop = False
                        if idx_lp < loop_cur_line: continue
                        if float(item[0]) < float(item_lp[0]): break
                        if item[0] != item_lp[0]: continue

                        # when item[0] == item_lp[0]:
                        find_loop = True
                        loop_cur_line = idx_lp
                        loop_x = float(item_lp[1])
                        loop_y = float(item_lp[2])
                        loop_z = float(item_lp[3])
                        loop_qx = float(item_lp[5])
                        loop_qy = float(item_lp[6])
                        loop_qz = float(item_lp[7])
                        loop_qw = float(item_lp[4])
                        (loop_yaw, loop_pitch, loop_roll) = attitude.quat2euler(np.array([loop_qw, loop_qx, loop_qy, loop_qz]))
                        break

            #打开ins1000 ground_truth.csv
            with open(ground_truth_file,'r') as csv_file: 
                ground_truth_reader = csv.reader(csv_file)
                #遍历ins1000 ground_truth.csv
                for _idx,_item in enumerate(ground_truth_reader): 
                    if _idx < gt_cur_line: continue

                    if abs(float(_item[0]) - timestamp) > time_delta: 
                        if _idx == row_count - 1:
                            print('[{0}]:can not find same timestamp {1} in ground-truth.'.format(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'), timestamp))
                        else: 
                            continue
                    else:
                        gt_cur_line = _idx
                        gt_lat = float(_item[2])/R2D
                        gt_lon = float(_item[3])/R2D
                        gt_alt = float(_item[4])
                        gt_v_x = float(_item[5]) #都转换为body系下的表示的速度,主要是沿行驶方向上的速度.
                        gt_v_y = float(_item[6])
                        gt_v_z = float(_item[7])
                        gt_qw = float(_item[8])
                        gt_qx = float(_item[9])
                        gt_qy = float(_item[10])
                        gt_qz = float(_item[11])

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
                            c_bv1 = c_bn.dot(c_vn.T)
                            c_bv = c_ib_vb.dot(c_bn.dot(c_vn.T))
                            ins1000_euler = attitude.dcm2euler(c_bv)
                            # ins1000的yaw不准，需要增加一个delta_yaw项做人工对齐。
                            # c_vn = attitude.rot_z(algo_yaw - ins1000_euler[0] + delta_yaw).dot(c_vn)
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
                        ins1000_vel_v = attitude.rot_x(math.pi).dot(attitude.euler2dcm(np.array([gt_yaw, gt_pitch, gt_roll])).dot(np.array([gt_v_x, gt_v_y, gt_v_z])))

                        # print("{0:0.3f},{1},{2},{3}".format(timestamp,ins1000_pos_ned[0],ins1000_pos_ned[1],ins1000_pos_ned[2]))
                        # print("{0},{1},{2}".format(ins1000_pos_ned[0],ins1000_pos_ned[1],ins1000_pos_ned[2]))
                        
                        algo_ecef = c_ev.dot((np.array([algo_x, algo_y, algo_z]) - t_vn)) + ins1000_pos_ecef[0]
                        algo_lla = geoparams.ecef2lla(algo_ecef)
                        # algo的yaw转为ned系的heading。
                        algo_att = attitude.dcm2euler(attitude.rot_x(math.pi).dot(attitude.euler2dcm(np.array([algo_yaw, algo_pitch, algo_roll]))).dot(c_vn))
                        # algo在body下对vins导航系速度的表示
                        algo_vel = attitude.euler2dcm(np.array([algo_yaw, algo_pitch, algo_roll])).dot(np.array([algo_v_x, algo_v_y, algo_v_z]))
    
                        str_data = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19},{20},{21},{22},{23},{24},{25},{26},{27},{28},{29},{30}'.format(str(timestamp),\
                            gt_lat*R2D, gt_lon*R2D, gt_alt, gt_roll*R2D, gt_pitch*R2D, gt_yaw*R2D, \
                            ins1000_pos_ned[0], ins1000_pos_ned[1], ins1000_pos_ned[2], \
                            algo_x, algo_y, algo_z, algo_roll*R2D, algo_pitch*R2D, algo_yaw*R2D, algo_vel[0], algo_vel[1], algo_vel[2], \
                            ins1000_pos_v[0], ins1000_pos_v[1], ins1000_pos_v[2], ins1000_rpy_v[2]*R2D, ins1000_rpy_v[1]*R2D, ins1000_rpy_v[0]*R2D, \
                            ins1000_vel_v[0], ins1000_vel_v[1], ins1000_vel_v[2], algo_lla[0]*R2D, algo_lla[1]*R2D, algo_lla[2])
                        
                        str_loop = ""
                        if loop_result_file is not None and find_loop:
                            loop_ecef = c_ev.dot((np.array([loop_x, loop_y, loop_z]) - t_vn)) + ins1000_pos_ecef[0]
                            loop_lla = geoparams.ecef2lla(loop_ecef)
                            loop_att = attitude.dcm2euler(attitude.rot_x(math.pi).dot(attitude.euler2dcm(np.array([loop_yaw, loop_pitch, loop_roll]))).dot(c_vn))

                            str_loop = ',{0},{1},{2},{3},{4},{5},{6},{7},{8}'.format(\
                                loop_x, loop_y, loop_z, \
                                loop_lla[0]*R2D, loop_lla[1]*R2D, loop_lla[2], \
                                loop_roll*R2D, loop_pitch*R2D, loop_yaw*R2D)

                        output_file.write(str_data + str_loop + '\n')

                        if gt_llas.shape[0] == 0:
                            gt_llas = np.array([[gt_lat,gt_lon,gt_alt]])
                            gt_headings = np.array([gt_yaw*R2D])
                            algo_llas = np.array([algo_lla])
                            algo_headings = np.array([algo_att[0]*R2D])
                            if loop_result_file is not None and find_loop:
                                loop_llas = np.array([loop_lla])
                                loop_headings = np.array([loop_att[0]*R2D])
                        else:
                            gt_llas = np.append(gt_llas, np.array([[gt_lat,gt_lon,gt_alt]]),axis=0)
                            gt_headings = np.append(gt_headings, np.array([gt_yaw*R2D]))
                            algo_llas = np.append(algo_llas, np.array([algo_lla]),axis=0)
                            algo_headings = np.append(algo_headings, np.array([algo_att[0]*R2D]))
                            if loop_result_file is not None and find_loop:
                                if 0 == loop_llas.size:
                                    loop_llas = np.array([loop_lla])
                                    loop_headings = np.array([loop_att[0]*R2D])
                                else:
                                    loop_llas = np.append(loop_llas, np.array([loop_lla]),axis=0)
                                    loop_headings = np.append(loop_headings, np.array([loop_att[0]*R2D]))
                        break

    output_file.close()
    kml_path = os.path.abspath('./kmlpath')
    if not os.path.exists(kml_path):
        os.makedirs(kml_path)
    # kml_gen.kml_gen(kml_path, gt_llas, gt_headings, 'ground_truth', color='ff00ff00')
    kml_gen.kml_gen(kml_path, algo_llas, algo_headings, 'algo', color='ff0000ff', gap=10)
    kml_gen.kml_gen(kml_path, gt_llas, gt_headings, 'ground-truth', color='ff00ff00', gap=10)
    kml_gen.kml_gen(kml_path, loop_llas, loop_headings, 'loop', color='ffff0000', gap=10)
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


def cal_diff_yaw(align_flie):
    align_yaws = []
    algo_yaws = []

    with open(align_flie,'r') as csv_file:
        align_reader = csv.reader(csv_file)

        for idx,item in enumerate(align_reader):
            if 0 == idx: continue
            if item[24] == '' :continue
            align_yaws.append(float(item[24])) # ins1000转换到vins导航系下的heading, deg
            algo_yaws.append(float(item[15]))   # vins输出的heading, deg

    align_yaws = np.array(align_yaws) 
    algo_yaws = np.array(algo_yaws) 
    diff_yaws = (align_yaws - algo_yaws)/R2D
    for i in range(diff_yaws.size):
        diff_yaws[i] = attitude.angle_range_pi(diff_yaws[i])

    np.savetxt("aaa.txt", diff_yaws*R2D)


def main():
    # ground_truth_file = 'ins1000/2019-11-12/CNM-20191112_135317.csv' 
    # algo_result_file = './data_2019_11_12/company_15hz_left/vio.csv'
    # loop_result_file = './data_2019_11_12/company_15hz_left/vio_loop.csv'
    # delta_yaw = -0.03

    ground_truth_file = 'ins1000/2019-11-12/CNM-20191112_094710.csv' 
    algo_result_file = './data_2019_11_12/company_15hz_right/vio.csv'
    loop_result_file = './data_2019_11_12/company_15hz_right/vio_loop.csv'
    delta_yaw = 0.16

    # ground_truth_file = 'ins1000/2019-11-12/CNM-20191112_135723.csv' 
    # algo_result_file = './data_2019_11_12/company_20hz_left/vio.csv'
    # loop_result_file = './data_2019_11_12/company_20hz_left/vio_loop.csv'
    # delta_yaw = -0.06

    # ground_truth_file = 'ins1000/2019-11-12/CNM-20191112_095251.csv' 
    # algo_result_file = './data_2019_11_12/company_20hz_right/vio.csv'
    # loop_result_file = './data_2019_11_12/company_20hz_right/vio_loop.csv'
    # delta_yaw = -0.17

    # ground_truth_file = 'ins1000/2019-11-12/CNM-20191112_141454.csv' 
    # algo_result_file = './data_2019_11_12/road_15hz_left/vio.csv'
    # loop_result_file = './data_2019_11_12/road_15hz_left/vio_loop.csv'
    # delta_yaw = -0.04

    # ground_truth_file = 'ins1000/2019-11-12/CNM-20191112_102325.csv' 
    # algo_result_file = './data_2019_11_12/road_15hz_right/vio.csv'
    # loop_result_file = './data_2019_11_12/road_15hz_right/vio_loop.csv'
    # delta_yaw = -0.05

    # ground_truth_file = 'ins1000/2019-11-12/CNM-20191112_142413.csv' 
    # algo_result_file = './data_2019_11_12/road_20hz_left/vio.csv'
    # loop_result_file = './data_2019_11_12/road_20hz_left/vio_loop.csv'
    # delta_yaw = 0

    # ground_truth_file = 'ins1000/2019-11-12/CNM-20191112_101307.csv' 
    # algo_result_file = './data_2019_11_12/road_20hz_right/vio.csv'
    # loop_result_file = './data_2019_11_12/road_20hz_right/vio_loop.csv'
    # delta_yaw = -0.02

    # ground_truth_file = 'ins1000/2019-10-15/CNM-20191015_094301.csv' #0.14
    # algo_result_file = './data_2019_10_15/2019_10_15/company_15hz_left/vio.csv'
    # loop_result_file = './data_2019_10_15/2019_10_15/company_15hz_left/vio_loop.csv'
    # delta_yaw = 0.14

    # align_flie = './data_2019_11_12/company_15hz_left/align.csv'
    # align_flie = './data_2019_11_12/company_15hz_right/align.csv'
    # align_flie = './data_2019_11_12/company_15hz_right/align.csv'
    align_flie = 'align.csv'
    save_aligned_data_vio_and_loop(align_flie, ground_truth_file, algo_result_file, delta_yaw, loop_result_file)

    # cal_diff_yaw(align_flie)



if __name__=="__main__":
    print('start:{0}'.format(datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S')))
    time_start=time.time()
    main()
    time_end=time.time()
    print('Time cost:{0:0.2f}s'.format(time_end-time_start))
    print('end:{0}'.format(datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S')))
