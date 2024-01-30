close all;
clear;
clc;

%% Load parameters
bendData = readmatrix("C:/Users/bsgx043/Desktop/Forward-and-Inverse-kinematics/KinematicsMATLAB/Robotics/FlexibleSyringe/bend_data.csv");
yawData = readmatrix("C:/Users/bsgx043/Desktop/Forward-and-Inverse-kinematics/KinematicsMATLAB/Robotics/FlexibleSyringe/yaw_data.csv");
baseData = readmatrix("C:/Users/bsgx043/Desktop/Forward-and-Inverse-kinematics/KinematicsMATLAB/Robotics/FlexibleSyringe/bend_base_data.csv");

BendStep = 30;

BasePos = [0,0,0];
TMatrix = [1 0 0 0
           0 1 0 0
           0 0 1 0
           0 0 0 1];

[Base, T01, T02, T03, T04, curve] = DHFinal(BendStep,bendData,yawData,baseData,BasePos,TMatrix);


Point0 = Base(1:3,4);
Point1 = T01(1:3,4);
Point2 = T02(1:3,4);
Point3 = T03(1:3,4);
Point4 = T04(1:3,4);

%% Plot
plot3([Point0(1),Point1(1)],[Point0(2),Point1(2)],[Point0(3),Point1(3)],'-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
hold on
plot3([Point1(1),Point2(1)],[Point1(2),Point2(2)],[Point1(3),Point2(3)],'-b', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
plot3([Point2(1),Point3(1)],[Point2(2),Point3(2)],[Point2(3),Point3(3)],'-b', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
plot3(Point4(1),Point4(2),Point4(3),'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
plot3(curve(:,1), curve(:,2), curve(:,3), 'r-', 'LineWidth', 2);
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis([-300 300 -300 300 00 600]);
grid on
view(73.7000, 70.8084);