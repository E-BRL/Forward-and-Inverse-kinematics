close all
clear;
clc;

close all;
clear;
clc;

%% Load parameters
bendData = readmatrix("C:/Users/bsgx043/Desktop/Forward-and-Inverse-kinematics/KinematicsMATLAB/Robotics/FlexibleSyringe/bend_data.csv");
yawData = readmatrix("C:/Users/bsgx043/Desktop/Forward-and-Inverse-kinematics/KinematicsMATLAB/Robotics/FlexibleSyringe/yaw_data.csv");
baseData = readmatrix("C:/Users/bsgx043/Desktop/Forward-and-Inverse-kinematics/KinematicsMATLAB/Robotics/FlexibleSyringe/bend_base_data.csv");

%% Define bend, yaw

bend = 0;
yaw = 0;

[yaw,bendbase,bend] = FKtest(yaw, bend, bendData, yawData, baseData);

T0 = [0,0,0]; %base
T1 = [0,0,138]; %yawStart

norm(bendbase-yaw)


%% Plot
plot3([T0(1),T1(1)],[T0(2),T1(2)],[T0(3),T1(3)],'-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
hold on
plot3([T1(1),yaw(1)],[T1(2),yaw(2)],[T1(3),yaw(3)],'-b', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
plot3([yaw(1),bendbase(1)],[yaw(2),bendbase(2)],[yaw(3),bendbase(3)],'-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
plot3([bendbase(1),bend(1)],[bendbase(2),bend(2)],[bendbase(3),bend(3)],'-r', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis([-300 300 -300 300 00 600]);
grid on
