function [curve_points] = formCurve(A,B,C)

%This function produces a Discretised Bezier Curve between 2 points (from B
%to C) using the vector between point A and B as a refernce direction for
%placing a control point.

% Vector from A to B
vector_AB = B - A;

% Normalize the vector AB to get a unit vector
unit_vector_AB = vector_AB / norm(vector_AB);

% Calculate the distance from B to C and multiply by 2/3
distance_BC = 2*(norm(B - C))/3;

% Set the control point at a distance from B in the direction of the unit vector AB
control_point = B + distance_BC * unit_vector_AB;

% Generate points along the curve
t_values = linspace(0, 1, 100);
curve_points = arrayfun(@(t) quadraticBezier(t, B, control_point, C), t_values, 'UniformOutput', false);
curve_points = cell2mat(curve_points');