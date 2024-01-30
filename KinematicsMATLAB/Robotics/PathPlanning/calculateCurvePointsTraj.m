function [curve_points, control_point] = calculateCurvePointsTraj(A, B, C, increments)
    % Vector from A to B
    vector_AB = B - A;
    
    % Normalize the vector AB to get a unit vector
    unit_vector_AB = vector_AB / norm(vector_AB);
    
    % Calculate the distance from B to C
    distance_BC = 2*(norm(B - C))/4;
    
    % Set the control point at a distance from B in the direction of the unit vector AB
    control_point = B + distance_BC * unit_vector_AB;

    % Set the control point at a distance from B in the direction of the unit vector AB
    control_point2 = B + (distance_BC * unit_vector_AB)*1.5;
    
    % Generate points along the curve
    t_values = linspace(0, 1, increments);
    curve_points = arrayfun(@(t) cubicBezier(t, B, control_point, control_point2, C), t_values, 'UniformOutput', false);
    curve_points = cell2mat(curve_points');
end