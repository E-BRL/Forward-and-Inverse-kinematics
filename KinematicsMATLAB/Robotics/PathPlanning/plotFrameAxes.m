function plotFrameAxes(T, scale)
    % This function plots the frame axes given a transformation matrix T
    % and a scale factor for the length of the axes.

    % Extract the origin of the frame
    origin = T(1:3, 4);

    % Extract the direction of the axes
    x_axis = T(1:3, 1) * scale + origin;
    y_axis = T(1:3, 2) * scale + origin;
    z_axis = T(1:3, 3) * scale + origin;

    % Plot the axes
    plot3([origin(1), x_axis(1)], [origin(2), x_axis(2)], [origin(3), x_axis(3)], 'r', 'LineWidth', 2); % X-axis in red
    plot3([origin(1), y_axis(1)], [origin(2), y_axis(2)], [origin(3), y_axis(3)], 'g', 'LineWidth', 2); % Y-axis in green
    plot3([origin(1), z_axis(1)], [origin(2), z_axis(2)], [origin(3), z_axis(3)], 'b', 'LineWidth', 2); % Z-axis in blue

end