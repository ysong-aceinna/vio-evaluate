
% INS1000��λ����Ϊground-truth������VINS�㷨�����poseû�ж���,��Ҫ��"absoluteOrientationQuaternion"���ת������
% ���£�AΪalgo�����pose,BΪground-truth, transition_matrixΪA��B��ת������

clc;clear;close all;

[FileName, PathName]= uigetfile('*.csv','Please Selet the data file');
 if isequal(FileName,0) || isequal(PathName,0)
       disp('User pressed cancel')
    else
       disp(['User selected ', fullfile(PathName, FileName)])
 end
FillNfull = fullfile(PathName,FileName);

data = importdata(FillNfull);
data = data.data; %ȡ��csv�е����ݲ��֡�

ref_pose_x = data(:,8);
ref_pose_y = data(:,9);
ref_pose_z = data(:,10);
algo_pose_x = data(:,11);
algo_pose_y = data(:,12);
algo_pose_z = data(:,13);

% load('EuRoC_MH_02_easy.mat')
[row,clum] = size(ref_pose_x);

% for "vio_1_[2019-10-8].csv", n = 255;

n =255; %ȡref_pose �� algo_pose ��ǰn������㣬��ת������
rx = ref_pose_x(1:n,:);
ry = ref_pose_y(1:n,:);
rz = ref_pose_z(1:n,:);
ax = algo_pose_x(1:n,:);
ay = algo_pose_y(1:n,:);
az = algo_pose_z(1:n,:);

A = [ax, ay, az]';
B = [rx, ry, rz]';
[s, R, T, error] = absoluteOrientationQuaternion( A, B, 0)
transition_matrix = [R, T]  %transition_matrix����B��A��ת������

i=min(1400, clum); %��ֹԽ��
a= transition_matrix * [algo_pose_x(i,:), algo_pose_y(i,:),algo_pose_z(i,:),1]'; %����algo����ĵ�i��pose
b= [ref_pose_x(i,:), ref_pose_y(i,:), ref_pose_z(i,:)]';%ground-truth�ĵ�i��pose
diff=a-b %���¶��ߵĲ�ࡣ







