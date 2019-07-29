
% EuRoC���ݼ���ground-truth���㷨�����poseû�ж���,��Ҫ����С�������ת������
% ���£�AΪalgo�����pose,BΪground-truth, XΪת������
% XA=B => X=B*A'*inv(AA')

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

ref_pose_x = data(:,2);
ref_pose_y = data(:,3);
ref_pose_z = data(:,4);
algo_pose_x = data(:,8);
algo_pose_y = data(:,9);
algo_pose_z = data(:,10);

% load('EuRoC_MH_02_easy.mat')
[row,clum] = size(ref_pose_x);

n =100; %ȡref_pose �� algo_pose ��ǰn������㣬��ת������
rx = ref_pose_x(1:n,:);
ry = ref_pose_y(1:n,:);
rz = ref_pose_z(1:n,:);
ax = algo_pose_x(1:n,:);
ay = algo_pose_y(1:n,:);
az = algo_pose_z(1:n,:);

A = [ax,ay,az,ones(n,1)]';
B = [rx, ry, rz]';
X = B * A' * inv((A*A')) %ת������

i=min(1600, clum); %��ֹԽ��
a= X * [algo_pose_x(i,:), algo_pose_y(i,:),algo_pose_z(i,:),1]'; %����algo����ĵ�i��pose
b= [ref_pose_x(i,:), ref_pose_y(i,:), ref_pose_z(i,:)]';%ground-truth�ĵ�i��pose
a-b %���¶��ߵĲ�ࡣ






