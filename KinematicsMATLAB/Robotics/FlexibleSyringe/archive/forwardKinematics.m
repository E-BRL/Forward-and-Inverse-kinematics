function [eePosition,RotPosBend,RotPosYaw,RotPosBase] = forwardKinematics(CurrentYaw, CurrentBend, baseData, bendData, yawData)

%% Evaluate contribution of the top section
BendPosition = [0, bendData(abs(CurrentBend)+1,2)*sign(CurrentBend), bendData(abs(CurrentBend)+1,3)]';
BendBasePosition = [0, baseData(abs(CurrentBend)+1,2)*sign(CurrentBend), baseData(abs(CurrentBend)+1,3)]';
xA = -1*sign(CurrentBend)*baseData(abs(CurrentBend)+1,1);

%% Evaluate the contribution of the lower section
YawTemp = [yawData(abs(CurrentYaw)+1,2)*sign(CurrentYaw), 0, yawData(abs(CurrentYaw)+1,3)];
yA = sign(CurrentYaw)*yawData(abs(CurrentYaw)+1,1);

Rigidpart = 4.5; %mm
YawPosition = [YawTemp(1)+(Rigidpart*sin(abs(yA))), 0, YawTemp(3)+(Rigidpart*sin(abs(yA)))]';

%% Rotation for deflections in x and y
xRot = [1, 0, 0
    0, cos(xA), -sin(xA)
    0, sin(xA), cos(xA)];

yRot = [cos(yA), 0, sin(yA)
    0, 1, 0
    -sin(yA), 0, cos(yA)];

RotPosYaw = xRot*YawPosition;
RotPosBend = yRot*BendPosition;
RotPosBase = yRot*BendBasePosition;

%% Translate the bend joint to the tip of the yaw joint
Pos = RotPosYaw - RotPosBase;
eePosition = RotPosBend + Pos;

end