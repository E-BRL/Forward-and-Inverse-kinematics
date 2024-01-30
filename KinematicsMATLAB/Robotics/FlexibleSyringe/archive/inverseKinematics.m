function [YawStep,BendStep] = inverseKinematics(CurrentYawStep,CurrentBendStep,baseData,bendData,yawData,Target)

%% Obtain current position
CurrentPos = forwardKinematics(CurrentYawStep, CurrentBendStep, baseData, bendData, yawData);

%% Vector between Current pos and target
Distance = CurrentPos - Target;

%% Direct Distance
errorDist = norm(Distance);


%% Define desired directions of movement
if Distance(1) < 0 %need to move to the right of current position (+x direction)
    yawCount = 1;
elseif Distance(1) > 0 %need to move to the left of current position (-x direction)
    yawCount = -1;
else
    yawCount = 0;
end


if Distance(2) < 0 % need to move forward of current position (+y direction)
    bendCount = 1;
elseif Distance(2) > 0 %need to move reverse from current position (-y direction)
    bendCount = -1;
else
    bendCount= 0;
end

%% While loop until at desired position
endFlag = false;

bendStep = CurrentBendStep;
yawStep = CurrentYawStep;


while ((abs(yawStep)+2) <= length(yawData))

    if endFlag
        break
    end
    
    bendStep = CurrentBendStep;
    yawStep = yawStep + yawCount;
    
    while ((abs(bendStep)+2) <= length(bendData))

            bendStep = bendStep + bendCount;

            ModifiedPosition = forwardKinematics(yawStep,bendStep,baseData,bendData,yawData);
            ModifiedError = norm(ModifiedPosition-Target);
            
            if errorDist <= ModifiedError
                if errorDist < 1 %Threshold for success
                    endFlag = true;
                    break
                end
            end
            errorDist = ModifiedError;
    end

end

%% Return usefual variables
YawStep = yawStep;
BendStep = bendStep+bendCount;






