function [RotPosYaw,RotPosBase,RotPosBend,ee] = FK2(CurrentYaw, CurrentBend, baseData, bendData, yawData)


if CurrentBend < 0
    CurrentBend = -CurrentBend;
    BendPos = [0, -1*bendData(CurrentBend+1,2), bendData(CurrentBend+1,3)]';
    BendBasePos = [0, -1*baseData(CurrentBend+1,2), baseData(CurrentBend+1,3)]';
    xA = baseData(CurrentBend+1,1);

else 
    BendPos = [0, bendData(CurrentBend+1,2), bendData(CurrentBend+1,3)]';
    BendBasePos = [0, baseData(CurrentBend+1,2), baseData(CurrentBend+1,3)]';
    xA = baseData(CurrentBend+1,1);
end

if CurrentYaw < 0
    CurrentYaw = -CurrentYaw;
    YawPos = [-1*yawData(CurrentYaw+1,2), 0, yawData(CurrentYaw+1,3)]';
    yA = -1*yawData(CurrentYaw+1,0);
    YawPos = [YawPos(1)-4.5*sin(abs(yA)), 0, YawPos(3) + 4.5*sin(abs(yA))]';

else
    YawPos = [yawData(CurrentYaw+1,2), 0, yawData(CurrentYaw+1,3)]';
    yA = yawData(CurrentYaw+1, 1);
    YawPos = [YawPos(1)+4.5*sin(abs(yA)), 0, YawPos(3)+4.5*sin(abs(yA))]';
end

%% Rotation for deflections in x and y
xRot = [1, 0, 0
    0, cos(xA), -sin(xA)
    0, sin(xA), cos(xA)];

yRot = [cos(yA), 0, sin(yA)
    0, 1, 0
    -sin(yA), 0, cos(yA)];

RotPosYaw = xRot*YawPos;
RotPosBend = yRot*BendPos;
RotPosBase = yRot*BendBasePos;

%% Translate the bend joint to the tip of the yaw joint
Pos = RotPosYaw - RotPosBase;
ee = RotPosBend + Pos;