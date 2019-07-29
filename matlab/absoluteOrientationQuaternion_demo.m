
% offical demo of "absoluteOrientationQuaternion".
% ref: https://www.mathworks.com/matlabcentral/fileexchange/22422-absolute-orientation
% https://blog.csdn.net/smf0504/article/details/52677431

clc;clear;close all;
% 先构造两个坐标系的3D点X和Y,
s=1; %s=0.7  两坐标系之间的缩放尺度
R = [0.36 0.48 -0.8 ; -0.8 0.6 0 ; 0.48 0.64 0.6]; % X到Y的旋转矩阵
T= [45 -78 98]';  % 两个坐标系间的平移量
X = [ 0.272132 0.538001 0.755920 0.582317;   %至少需要4个点
0.728957 0.089360 0.507490 0.100513; 
0.578818 0.779569 0.136677 0.785203]; 
Y = s*R*X+repmat(T,1,4); %repmat(T,1,4)是旋转后再加上一个平移量T。

%Compute 
[s2,R2,T2,error] = absoluteOrientationQuaternion( X, Y, 1) % 如果两个坐标系不存在缩放的欢喜，则doScale应设置为0。

%Add noise
Noise = [ 
-0.23 -0.01 0.03 -0.06; 
0.07 -0.09 -0.037 -0.08; 
0.009 0.09 -0.056 0.012];

Y = Y+Noise; 
[s2 R2 T2 error] = absoluteOrientationQuaternion( X, Y, 1)

% 运行结果：
% s2 = 1.000000000000000
% 
% R2 =
%    0.360000000000002   0.480000000000004  -0.799999999999997
%   -0.799999999999996   0.600000000000006   0.000000000000006
%    0.480000000000005   0.639999999999992   0.600000000000005
% 
% T2 =
%   44.999999999999986
%  -78.000000000000000
%   98.000000000000014
% 
% error =  1.799270102596743e-14
% 
% ***********添加噪声后*********
% 
% s2 = 1.168803791772182
% 
% 
% R2 =
%    0.529970779476690   0.411527879296549  -0.741468662495290
%   -0.684614393956318   0.723608001500273  -0.087718822111357
%    0.500433916205419   0.554108531540948   0.665229006271405
% 
% 
% T2 =
%   44.830790511826365
%  -78.063338255874783
%   97.853478884350125
% 
% error = 0.082546858874616


