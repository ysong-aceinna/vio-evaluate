
% EuRoC数据集的ground-truth和算法输出的pose没有对齐,需要用"absoluteOrientationQuaternion"求解转换矩阵。
% 如下，A为algo输出的pose,B为ground-truth, transition_matrix为转换矩阵

clc;clear;close all;

[FileName, PathName]= uigetfile('*.csv','Please Selet the data file');
 if isequal(FileName,0) || isequal(PathName,0)
       disp('User pressed cancel')
    else
       disp(['User selected ', fullfile(PathName, FileName)])
 end
FillNfull = fullfile(PathName,FileName);

data = importdata(FillNfull);
data = data.data; %取出csv中的数据部分。

ref_pose_x = data(:,2);
ref_pose_y = data(:,3);
ref_pose_z = data(:,4);
algo_pose_x = data(:,8);
algo_pose_y = data(:,9);
algo_pose_z = data(:,10);

% load('EuRoC_MH_02_easy.mat')
[row,clum] = size(ref_pose_x);

% for "machine_hall_02", n = 200;
% for "vicon_room_1_02", n = 100;
% for "vicon_room_1_03", n = 100;

% for "machine_hall_02", n = 120;
% for "vicon_room_1_02", n = 90;

n =120; %取ref_pose 和 algo_pose 的前n个坐标点，求转换矩阵。
rx = ref_pose_x(1:n,:);
ry = ref_pose_y(1:n,:);
rz = ref_pose_z(1:n,:);
ax = algo_pose_x(1:n,:);
ay = algo_pose_y(1:n,:);
az = algo_pose_z(1:n,:);

A = [ax, ay, az]';
B = [rx, ry, rz]';
[s, R, T, error] = absoluteOrientationQuaternion( A, B, 0)
transition_matrix = [R, T]  %transition_matrix就是A到B的转换矩阵。

i=min(1400, clum); %防止越界
a= transition_matrix * [algo_pose_x(i,:), algo_pose_y(i,:),algo_pose_z(i,:),1]'; %对齐algo输出的第i个pose
b= [ref_pose_x(i,:), ref_pose_y(i,:), ref_pose_z(i,:)]';%ground-truth的第i个pose
diff=a-b %看下二者的差距。







