close all;
clear;
clc;

%% Load parameters
bendData = readmatrix("C:/Users/bsgx043/Desktop/Seokchang/Forward-and-Inverse-kinematics/test/bend_data.csv");
yawData = readmatrix("C:/Users/bsgx043/Desktop/Seokchang/Forward-and-Inverse-kinematics/test/yaw_data.csv");
baseData = readmatrix("C:/Users/bsgx043/Desktop/Seokchang/Forward-and-Inverse-kinematics/test/bend_base_data.csv");

Base = [0,0,0];

%% Init Syringe

%Forward Kinematic Test
[YawEnd,BendBase,BendEnd,EEPos] = FK2(0,1,baseData, bendData, yawData);


X00 = [0,0];
Y00 = [0,0];
Z00 = [0,138];

X01 = [0,YawEnd(1)];
Y01 = [0,YawEnd(2)];
Z01 = [138,YawEnd(3)];

X12 = [YawEnd(1),BendBase(1)];
Y12 = [YawEnd(2),BendBase(2)];
Z12 = [YawEnd(3),BendBase(3)];

X23 = [BendBase(1),BendEnd(1)];
Y23 = [BendBase(2),BendEnd(2)];
Z23 = [BendBase(3),BendEnd(3)];

X34 = [BendEnd(1),EEPos(1)];
Y34 = [BendEnd(2),EEPos(2)];
Z34 = [BendEnd(3),EEPos(3)];

plot3(X00,Y00,Z00,'k')
hold on
plot3(X01,Y01,Z01,'r')
plot3(X12,Y12,Z12,'k')
plot3(X23,Y23,Z23,'b')
plot3(X34,Y34,Z34,'k')
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis([-20 20 -20 20 0 250]);
grid on

