function [Base,T01, T02, T03, T04, curve] = DHFinal(BendStep,bendData,yawData,baseData,BasePos,TMatrix)


ben_pos = FK3(0,BendStep,bendData,yawData,baseData)/1000;

%% Frame Assigment

Base = [1 0 0 0
        0 1 0 0
        0 0 1 0
        0 0 0 1];
    
T1 = [1 0 0 0
      0 1 0 0
      0 0 1 0.138
      0 0 0 1];
        
T2 = [1 0 0 0
      0 1 0 0
      0 0 1 0.025
      0 0 0 1];

T3 = [1 0 0 0
      0 1 0 0
      0 0 1 0.005
      0 0 0 1];
         
T4 = [1 0 0 0
      0 1 0 0
      0 0 1 0
      0 0 0 1];  

T03 = Base*T1*T2*T3;

T4((1:3),4) = [ben_pos(1)-T03(1,4),ben_pos(2)-T03(2,4),ben_pos(3)-T03(3,4)];

RotateZ90 = [0 -1 0 0
            1 0 0 0
            0 0 1 0
            0 0 0 1];

RotateYneg90 = [0 0 -1 0
                0 1 0 0
                1 0 0 0
                0 0 0 1];
            
RotateXneg90 = [1 0 0 0
            0 0 1 0
            0 -1 0 0
            0 0 0 1];


Base = TMatrix*RotateZ90*Base;

Base(1,4) = BasePos(1); %Base xpos
Base(2,4) = BasePos(2); %Base ypos
Base(3,4) = BasePos(3); %Base zpos

%% Get transformation frames
T01 = Base*T1;
T02 = Base*T1*T2;
T03 = Base*T1*T2*T3;
T04 = Base*T1*T2*T3*T4;

%% Get Bezier Curve
BasePosition = [Base(1,4),Base(2,4),Base(3,4)];
Position3 = [T03(1,4),T03(2,4),T03(3,4)];
Position4 = [T04(1,4),T04(2,4),T04(3,4)];

[curve,~] = calculateCurvePoints(BasePosition, Position3, Position4);
