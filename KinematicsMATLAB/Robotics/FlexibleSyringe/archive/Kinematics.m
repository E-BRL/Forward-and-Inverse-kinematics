close all;
clear;
clc;

% Kinematics for the flexible needle. Note that yaw refers to movement of
% the yaw joint in the x axis. Bend refers to the moevement of the bend
% joint in the y axis.


%% Load parameters
bendData = readmatrix("C:/Users/user/Desktop/Korea/Seokchang/Forward-and-Inverse-kinematics/test/bend_data.csv");
yawData = readmatrix("C:/Users/user/Desktop/Korea/Seokchang/Forward-and-Inverse-kinematics/test/yaw_data.csv");
baseData = readmatrix("C:/Users/user/Desktop/Korea/Seokchang/Forward-and-Inverse-kinematics/test/bend_base_data.csv");

%Forward Kinematic Test
ben_pos = FK3(0,40,bendData,yawData,baseData);

%% Frame Assigment

Base = [1 0 0 0
        0 1 0 0
        0 0 1 0
        0 0 0 1];
    
YawStart = [1 0 0 0
            0 1 0 0
            0 0 1 135
            0 0 0 1];
        
YawEnd = [1 0 0 0
          0 1 0 0
          0 0 1 163
          0 0 0 1];
      
BendStart = [1 0 0 0
             0 1 0 0
             0 0 1 169
             0 0 0 1];
         
BendEnd = [1 0 0 ben_pos(1)
           0 1 0 ben_pos(2)
           0 0 1 ben_pos(3)
           0 0 0 1];
 
Point0 = Base(1:3,4);
Point1 = YawStart(1:3,4);
Point2 = YawEnd(1:3,4);
Point3 = BendStart(1:3,4);
Point4 = BendEnd(1:3,4);
[Curve12,~] = calculateCurvePoints(Point2', Point3', Point4');

        
%% Plot
plot3([Point0(1),Point1(1)],[Point0(2),Point1(2)],[Point0(3),Point1(3)],'-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
hold on
plot3([Point1(1),Point2(1)],[Point1(2),Point2(2)],[Point1(3),Point2(3)],'-b', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
plot3([Point2(1),Point3(1)],[Point2(2),Point3(2)],[Point2(3),Point3(3)],'-b', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
plot3(Point4(1),Point4(2),Point4(3),'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
plot3(Curve12(:,1), Curve12(:,2), Curve12(:,3), 'r-', 'LineWidth', 2);
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis([-150 150 -150 150 00 300]);