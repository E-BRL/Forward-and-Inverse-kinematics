function rot_pos_bend = FK3(yaw, bend, bend_data, yaw_data, bend_base_data)
    % MATLAB function to perform forward kinematics

    % Direction setup
    if bend < 0
        bend = -bend;
        ben_pos = [0, -bend_data(bend+1, 2), bend_data(bend+1, 3)];
        ben_base_pos = [0, -bend_base_data(bend+1, 2), bend_base_data(bend+1, 3)];
        xA = bend_base_data(bend+1, 1);
    else
        ben_pos = [0, bend_data(bend+1, 2), bend_data(bend+1, 3)];
        ben_base_pos = [0, bend_base_data(bend+1, 2), bend_base_data(bend+1, 3)];
        xA = -bend_base_data(bend+1, 1);
    end

    if yaw < 0
        yaw = -yaw;
        yaw_pos = [-yaw_data(yaw+1, 2), 0, yaw_data(yaw+1, 3)];
        yA = -yaw_data(yaw+1, 1);
        yaw_pos = [yaw_pos(1)-(4.5*sin(abs(yA))), 0, yaw_pos(3)+(4.5*sin(abs(yA)))]; % Rigid part (yaw-bend) is set to 4.5mm
    else
        yaw_pos = [yaw_data(yaw+1, 2), 0, yaw_data(yaw+1, 3)];
        yA = yaw_data(yaw+1, 1);
        yaw_pos = [yaw_pos(1)+(4.5*sin(abs(yA))), 0, yaw_pos(3)+(4.5*sin(abs(yA)))]; % Rigid part (yaw-bend) is set to 4.5mm
    end

    % Rotation for yawing and bending
    rot_mat_x = [1, 0, 0; 0, cos(xA), -sin(xA); 0, sin(xA), cos(xA)];
    rot_mat_y = [cos(yA), 0, sin(yA); 0, 1, 0; -sin(yA), 0, cos(yA)];
    rot_pos_yaw = rot_mat_x * yaw_pos.';
    rot_pos_bend = rot_mat_y * ben_pos.';
    rot_pos_bend_base = rot_mat_y * ben_base_pos.';

    % Translation the bending joint to tip of the yawing joint
    tran_pos = rot_pos_yaw - rot_pos_bend_base;
    ee_pos = rot_pos_bend + tran_pos;
    
end