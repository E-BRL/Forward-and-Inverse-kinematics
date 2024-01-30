function [Base, T01, T02, T03, T04, T05, Curve12, Curve34] = DHSyringe(ThetaYaw,ThetaBend, BasePos, TMatrix)


%% Define Base Location
Base = [1,0,0,0
        0,1,0,0
        0,0,1,0
        0,0,0,1];

RotateY90 = [0 0 1 0
            0 1 0 0
            1 0 0 0
            0 0 0 1];

RotateYneg90 = [0 0 -1 0
                0 1 0 0
                1 0 0 0
                0 0 0 1];

RotateZ90 = [0 -1 0 0
            1 0 0 0
            0 0 1 0
            0 0 0 1];

RotateX90 = [1 0 0 0
            0 0 -1 0
            0 1 0 0
            0 0 0 1];

%rotate by 90 degrees in y axis and z axis
Base = TMatrix*RotateY90*Base; 

Base(1,4) = BasePos(1); %Base xpos
Base(2,4) = BasePos(2); %Base ypos
Base(3,4) = BasePos(3); %Base zpos

%% Define Constants
Theta1 = 0; Theta3 = 0; Theta5 = 0;
alpha1 = 0; alpha2 = 0; alpha3 = -pi/2; alpha4 = 0; alpha5 = 0;
a1  = 0.138; a2 = 0.025; a3 = 0.005; a4 = 0.075; a5 = 0.006;
d1 = 0; d2 = 0; d3 = 0; d4 = 0; d5 = 0;

%% Define Frames
T1 = DHFrame(alpha1,Theta1,d1,a1);
T12 = DHFrame(alpha2,ThetaYaw,d2,a2);
T23 = DHFrame(alpha3,Theta3,d3,a3);
T34 = DHFrame(alpha4,ThetaBend,d4,a4);
T45 = DHFrame(alpha5,Theta5,d5,a5);

%% Get transformation frames
T01 = Base*T1;
T02 = Base*T1*T12;
T03 = Base*T1*T12*T23;
T04 = Base*T1*T12*T23*T34;
T05 = Base*T1*T12*T23*T34*T45;

%% Get Bezier Curves
BasePosition = [Base(1,4),Base(2,4),Base(3,4)];
Position1 = [T01(1,4),T01(2,4),T01(3,4)];
Position2 = [T02(1,4),T02(2,4),T02(3,4)];
Position3 = [T03(1,4),T03(2,4),T03(3,4)];
Position4 = [T04(1,4),T04(2,4),T04(3,4)];

[Curve12,~] = calculateCurvePoints(BasePosition, Position1, Position2);
[Curve34, ~] = calculateCurvePoints(Position2,Position3,Position4);

end

