close all;
clear;
clc;

% Define the points A, B, and C
A = [0.163000000000000	0	0];
B = [0.168000000000000	0	0];
C = [0.238476946558943	1.57070202662887e-18	-0.0256515107494252];

% Vector from A to B
vector_AB = B - A;

% Normalize the vector AB to get a unit vector
unit_vector_AB = vector_AB / norm(vector_AB);

% Calculate the distance from B to C
distance_BC = 2*(norm(B - C))/4;

% Set the control point at a distance from B in the direction of the unit vector AB
control_point = B + distance_BC * unit_vector_AB;

% Generate points along the curve
t_values = linspace(0, 1, 100);
curve_points = arrayfun(@(t) quadraticBezier(t, B, control_point, C), t_values, 'UniformOutput', false);
curve_points = cell2mat(curve_points');

% Plotting
plot3(curve_points(:,1), curve_points(:,2), curve_points(:,3), 'b-', 'LineWidth', 2);
hold on;
scatter3(B(1), B(2), B(3), 'ro');
scatter3(C(1), C(2), C(3), 'go');
scatter3(control_point(1), control_point(2), control_point(3), 'bo');
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
title('Quadratic Bezier Curve from B to C with control point at distance BC');
grid on;
hold off;
axis equal


function point = quadraticBezier(t, p0, p1, p2)
    % Calculate a point on a quadratic Bezier curve
    point = (1 - t)^2 * p0 + 2 * (1 - t) * t * p1 + t^2 * p2;
end