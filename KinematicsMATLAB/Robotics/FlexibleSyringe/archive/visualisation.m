close all;
clear;
clc;

plotFrames = false;

%% Define Base Location
Base = [1,0,0,0
        0,1,0,0
        0,0,1,0
        0,0,0,1];

RotateY90 = [0 0 1 0
        0 1 0 0
        1 0 0 0
        0 0 0 1];

RotateZ90 = [0 -1 0 0
        1 0 0 0
        0 0 1 0
        0 0 0 1];

%rotate by 90 degrees in y axis and z axis
Base = RotateZ90*RotateY90*Base; 

Base(1,4)= 0; %Base xpos
Base(2,4) = 0; %Base ypos
base(3,4) = 0; %Base zpos


%% Define thetas
Theta2 = deg2rad(0);
Theta4 = deg2rad(90);

%% Define Constants
Theta1 = 0; Theta3 = 0; Theta5 = 0;
alpha1 = 0; alpha2 = 0; alpha3 = -pi/2; alpha4 = 0; alpha5 = 0;
a1  = 0.138; a2 = 0.025; a3 = 0.005; a4 = 0.075; a5 = 0.006;
d1 = 0; d2 = 0; d3 = 0; d4 = 0; d5 = 0;

%% Define Frames
T1 = DHFrame(alpha1,Theta1,d1,a1);
T12 = DHFrame(alpha2,Theta2,d2,a2);
T23 = DHFrame(alpha3,Theta3,d3,a3);
T34 = DHFrame(alpha4,Theta4,d4,a4);
T45 = DHFrame(alpha5,Theta5,d5,a5);

%% Test
T01 = Base*T1;
T02 = Base*T1*T12;
T03 = Base*T1*T12*T23;
T04 = Base*T1*T12*T23*T34;
T05 = Base*T1*T12*T23*T34*T45;

BasePosition = [Base(1,4),Base(2,4),Base(3,4)];
Position1 = [T01(1,4),T01(2,4),T01(3,4)];
Position2 = [T02(1,4),T02(2,4),T02(3,4)];
Position3 = [T03(1,4),T03(2,4),T03(3,4)];
Position4 = [T04(1,4),T04(2,4),T04(3,4)];
Position5 = [T05(1,4),T05(2,4),T05(3,4)];


X01 = [BasePosition(1),Position1(1)];
Y01 = [BasePosition(2),Position1(2)];
Z01 = [BasePosition(3),Position1(3)];

X12 = [Position1(1),Position2(1)];
Y12 = [Position1(2),Position2(2)];
Z12 = [Position1(3),Position2(3)];

[curve_points,~] = calculateCurvePoints(BasePosition, Position1, Position2);

X23 = [Position2(1),Position3(1)];
Y23 = [Position2(2),Position3(2)];
Z23 = [Position2(3),Position3(3)];

X34 = [Position3(1),Position4(1)];
Y34 = [Position3(2),Position4(2)];
Z34 = [Position3(3),Position4(3)];

[curve2, ~] = calculateCurvePoints(Position2,Position3,Position4);

X45 = [Position4(1),Position5(1)];
Y45 = [Position4(2),Position5(2)];
Z45 = [Position4(3),Position5(3)];

plot3(X01,Y01,Z01,'-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
hold on
plot3(X12,Y12,Z12, '-r', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
%plot3(curve_points(:,1), curve_points(:,2), curve_points(:,3), 'r-', 'LineWidth', 2);
hold on
plot3(X23,Y23,Z23, '-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
plot3(X34,Y34,Z34, '-b', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
%plot3(curve2(:,1), curve2(:,2), curve2(:,3), 'b-', 'LineWidth', 2);
plot3(X45,Y45,Z45, '-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
if plotFrames
    plotFrameAxes(T01, 0.01); 
    plotFrameAxes(T02, 0.01); 
    plotFrameAxes(T03, 0.01); 
    plotFrameAxes(T04, 0.01);
    plotFrameAxes(T05, 0.01);
end
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')

axis([-0.3 0.3 -0.3 0.3 0 0.3]);
grid on


