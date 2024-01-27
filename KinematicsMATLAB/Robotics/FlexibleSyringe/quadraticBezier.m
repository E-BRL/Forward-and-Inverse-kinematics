function point = quadraticBezier(t, p0, p1, p2)
    % Calculate a point on a quadratic Bezier curve
    point = (1 - t)^2 * p0 + 2 * (1 - t) * t * p1 + t^2 * p2;
end