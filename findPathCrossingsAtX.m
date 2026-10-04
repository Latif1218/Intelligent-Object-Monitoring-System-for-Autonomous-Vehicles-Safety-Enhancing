function sList = findPathCrossingsAtX(waypoints, cumDist, xVal)
%FINDPATHCROSSINGSATX Finds all arc-length distances s where the path's
% x-coordinate equals xVal, considering only roughly-horizontal segments
% (i.e. segments that are part of the main road, not a turn arc or a
% perpendicular crossroad segment).

sList = [];
n = size(waypoints, 1);
for i = 2:n
    p0 = waypoints(i-1, :);
    p1 = waypoints(i, :);
    dx = p1(1) - p0(1);
    dy = p1(2) - p0(2);
    if abs(dx) < 1e-6
        continue % vertical segment, not part of the main road here
    end
    if abs(dx) <= abs(dy)
        continue % more vertical than horizontal, skip (turn arc / crossroad)
    end
    xmin = min(p0(1), p1(1));
    xmax = max(p0(1), p1(1));
    if xVal >= xmin && xVal <= xmax
        frac = (xVal - p0(1)) / dx;
        s = cumDist(i-1) + frac * (cumDist(i) - cumDist(i-1));
        sList(end+1) = s; %#ok<AGROW>
    end
end
end
