function [Trajectory] = GenerateTrajectory(Start,Target,numPoints)


    % Linearly spaced vectors for x, y, and z
    xPoints = linspace(Start(1), Target(1), numPoints);
    yPoints = linspace(Start(2), Target(2), numPoints);
    zPoints = linspace(Start(3), Target(3), numPoints);

    % Combine them into a trajectory
    Trajectory = [xPoints; yPoints; zPoints]';


end

