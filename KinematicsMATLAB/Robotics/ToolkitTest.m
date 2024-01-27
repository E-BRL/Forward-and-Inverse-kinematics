%Plot 2 Kuka with desired pose, visulise magnetic field interaction of EPMs

%% Clear all previous
close all;
clear;
clc;

%% Activate/Deactivate Robots
Robot1Active = true;                %true or false
Robot2Active = false;
needleActive = true;
plotNeedleFrames = false;
phantomActive = false;

%% Load Phantom
%Load phantom
if phantomActive
    phantom = importrobot('urdf/phantom.urdf','DataFormat','row');
end
%% Adjustable Variables

%Define Trajectory Height
yHeight = 0.6;

%Define Target Point Robot 1
TargetPoint1 = [0.02,-0.4160,0.51];

%Define Target Point Robot 2
TargetPoint2 = [0,0.6,0.5];

%Define Magnet Orientation Robot 1
MagOrient1 =[0, 0, -pi/2];               %[Rx, Ry Rz]

%Define Magnet orientation Robot 2
MagOrient2 = [pi/2, 0, 0];

%% Constants

% Define unit vectors in local magnet frame (assuming the magnet's north pole points along the local Z-axis)
localMagnetDirection = [0; 0; 1];

%Magentic moment of the EPM (Magnitude)
mu_EPM = 970.1; %from some old code, where does it come from?
%mu_EPM = 0.00166; %N52 = 1.48T. Square cylinder, d=100mm,L=100mm mu = M*Volume = 1.48*pi*0.05^2*0.1

% Permeability of free space
mu0 = 4*pi*1e-7; 

% Grid
[x, y, z] = meshgrid(linspace(-0.4, 0.4, 25), linspace(-0.4, 0.4, 25), linspace(-0.42, 1, 25));

%Robot joint limits
jointLimits = [-170,170;
               -90,120; % Elbow Up
               -170,170;
               -120,120;
               -170,170;
               -120,120;
               -175,175];

%% Robot Inverse Kinematics 

%Robot 1
if Robot1Active

    %Load robot 1
    robot1 = importrobot('urdf/kuka_iiwa_1.urdf','DataFormat','row');

    %Create initial guess
    initialguess1 = robot1.homeConfiguration;

    %Create Inverse Kinematic solver
    IK1 = generalizedInverseKinematics('RigidBodyTree', robot1,'ConstraintInputs', {'position','aiming','joint','orientation'});

    %Define Position Constraints
    posConstraint1 = constraintPositionTarget('lbr_iiwa_link_7');
    posConstraint1.TargetPosition = TargetPoint1;

    %Define Aiming Constraint
    aimConstraint1 = constraintAiming('lbr_iiwa_link_7');
    aimConstraint1.TargetPoint = TargetPoint1;

    %Define Oirentation Constraints
    oirConstraint1 = constraintOrientationTarget('lbr_iiwa_link_7');
    oirConstraint1.OrientationTolerance = deg2rad(0.2);
    oirConstraint1.TargetOrientation = eul2quat(MagOrient1);

    %Define Constraints
    jointConstraint1 = constraintJointBounds(robot1);
    jointConstraint1.Bounds = deg2rad(jointLimits);

    %Inverse kinematics (optimisation routine)
    [Angles1,success1] = IK1(initialguess1, posConstraint1, aimConstraint1, jointConstraint1, oirConstraint1);
end

% Robot 2
if Robot2Active

    %Load robot 2
    robot2 = importrobot('urdf/kuka_iiwa_2.urdf','DataFormat','row');

    %Create initial guess
    initialguess2 = robot2.homeConfiguration;

    %Create Inverse Kinematic solver
    IK2 = generalizedInverseKinematics('RigidBodyTree', robot2,'ConstraintInputs', {'position','aiming','joint','orientation'});

    %Define Position Constraints
    posConstraint2 = constraintPositionTarget('magnet_center_link');
    posConstraint2.TargetPosition = TargetPoint2;

    %Define Aiming Constraint
    aimConstraint2 = constraintAiming('magnet_center_link');
    aimConstraint2.TargetPoint = TargetPoint2;

    %Define Oirentation Constraints
    oirConstraint2 = constraintOrientationTarget('magnet_center_link');
    oirConstraint2.OrientationTolerance = deg2rad(5);
    oirConstraint2.TargetOrientation = eul2quat(MagOrient2);

    %Define Constraints
    jointConstraint2 = constraintJointBounds(robot2);
    jointConstraint2.Bounds = deg2rad(jointLimits);

    %Inverse kinematics (optimisation routine)
    [Angles2,success2] = IK2(initialguess2, posConstraint2, aimConstraint2, jointConstraint2, oirConstraint2);
end

%% Robot End Effector Magnetic Fields

%initialise Bx, By, Bz

Bx2 = x.*0; By2 = x.*0; Bz2 = x.*0; 

%% Robot 1
if Robot1Active
    % Compute transformation matrix for robots from base to end effector
    transformMatrix1 = getTransform(robot1, Angles1, 'lbr_iiwa_link_7', 'base_link');

    %% Add points to visualise the flexible needle
    % Extract position and rotation matrix from transformation matrix
    position1 = transformMatrix1(1:3, 4);
    rotationMatrix1 = transformMatrix1(1:3, 1:3);

    if needleActive
    % Define the local offset in the end effector's frame
    % This is 0.145 units along the local Z-axis (ie start position of the
    % needle)
    localOffset = [0; 0; 0.145];
    
    % Transform the local offset to the global coordinate system
    globalOffset = rotationMatrix1 * localOffset;
    
    % Calculate the global position of the needle
    BasePos = position1 + globalOffset;
    
    ThetaYaw = deg2rad(0);
    ThetaBend = deg2rad(10);

    %Create Syringe
    [Base, T01, T02, T03, T04, T05, Curve12, Curve34] = DHSyringe(ThetaYaw,ThetaBend, BasePos, transformMatrix1);

    %Create plotable points
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
   
    X23 = [Position2(1),Position3(1)];
    Y23 = [Position2(2),Position3(2)];
    Z23 = [Position2(3),Position3(3)];
    
    X34 = [Position3(1),Position4(1)];
    Y34 = [Position3(2),Position4(2)];
    Z34 = [Position3(3),Position4(3)];
   
    X45 = [Position4(1),Position5(1)];
    Y45 = [Position4(2),Position5(2)];
    Z45 = [Position4(3),Position5(3)];

    end


end

%% Robot 2
if Robot2Active

    % Compute transformation matrix for robots from base to end effector
    transformMatrix2 = getTransform(robot2, Angles2, 'magnet_center_link', 'base_link');

    % Extract rotation matrices from transformation matrices
    R2 = transformMatrix2(1:3, 1:3);

    % Get x,y,z position Magnet of Robot 2
    Mag2_x = transformMatrix2(1,4);
    Mag2_y = transformMatrix2(2,4);
    Mag2_z = transformMatrix2(3,4);

    % Calculate magnetic moment vectors
    m2 = mu_EPM * R2 * localMagnetDirection;

    % Calculate field components for the second EPM (robot2)
    x2 = x - Mag2_x;
    y2 = y - Mag2_y;
    z2 = z - Mag2_z;
    r2 = sqrt(x2.^2 + y2.^2 + z2.^2);
    rx2 = x2./r2; ry2 = y2./r2; rz2 = z2./r2;
    
    Bx2 = mu0/(4*pi) * (3*(m2(1)*rx2 + m2(2)*ry2 + m2(3)*rz2).*rx2 - m2(1))./r2.^3;
    By2 = mu0/(4*pi) * (3*(m2(1)*rx2 + m2(2)*ry2 + m2(3)*rz2).*ry2 - m2(2))./r2.^3;
    Bz2 = mu0/(4*pi) * (3*(m2(1)*rx2 + m2(2)*ry2 + m2(3)*rz2).*rz2 - m2(3))./r2.^3;

    % Remove singularities
    threshold = 0.1;
    Bx2(r2<threshold) = NaN; By2(r2<threshold) = NaN; Bz2(r2<threshold) = NaN;
end


% Sum the magnetic fields from all dipoles
Bx_total = Bx2;
By_total = By2;
Bz_total = Bz2;

%% Needle FK, get 


%% Generate Trajectory
Trajectory1 = GenerateTrajectory([0.02,-0.5,yHeight],[0.02,-0.02,yHeight],100);
Trajectory2 = GenerateTrajectory([0.02,-0.02,yHeight],[0.02,0.00,yHeight],100);
[Trajectory3, ~] = calculateCurvePointsTraj([0.02,-0.35,yHeight], [0.02,0.00,yHeight], [0.00, 0.016,yHeight]);


Trajectory = [Trajectory1
              Trajectory2
              Trajectory3];

%% Visualise


%Display Robots
figure(1)
if Robot1Active
    show(robot1,Angles1,'Frames','off');
    hold on
    addOrientationArrows(transformMatrix1,0.1);
end
if Robot2Active
    show(robot2,Angles2,"Frames","off");
    hold on
    addOrientationArrows(transformMatrix2,0.2);
    plot3(Mag2_x, Mag2_y, Mag2_z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Second dipole position
end
if needleActive
    plot3(X01,Y01,Z01,'-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
    hold on
    plot3(Curve12(:,1), Curve12(:,2), Curve12(:,3), 'r-', 'LineWidth', 2);
    plot3(X23,Y23,Z23, '-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
    plot3(Curve34(:,1), Curve34(:,2), Curve34(:,3), 'b-', 'LineWidth', 2);
    plot3(X45,Y45,Z45, '-k', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize',2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
    if plotNeedleFrames 
        plotFrameAxes(Base, 0.1);
        plotFrameAxes(T01, 0.1); 
        plotFrameAxes(T02, 0.1); 
        plotFrameAxes(T03, 0.1); 
        plotFrameAxes(T04, 0.1);
        plotFrameAxes(T05, 0.1);
    end
end
if phantomActive
    show(phantom,"Frames","off");
end
quiver3(x, y, z, Bx_total, By_total, Bz_total);
hold on
plot3(Trajectory(:,1), Trajectory(:,2), Trajectory(:,3), '.r');
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trajectory');


axis([-1.2 1.2 -1.2 1.2 -0.42 1]);



% Start Trajectory1 = [0.02,-0.5,0.51];
% End Trajectory1 = [0.02,-0.414,0.51]
% End Trajectory2 = [0.02,-0.416,0.51]
