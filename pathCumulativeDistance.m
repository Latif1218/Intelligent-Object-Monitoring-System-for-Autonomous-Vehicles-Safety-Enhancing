function cumDist = pathCumulativeDistance(waypoints)
%PATHCUMULATIVEDISTANCE Cumulative arc-length along a polyline.
% waypoints: Nx3 [x y z]. Returns Nx1 cumulative distance, cumDist(1)=0.

n = size(waypoints, 1);
cumDist = zeros(n, 1);
for i = 2:n
    cumDist(i) = cumDist(i-1) + norm(waypoints(i, 1:2) - waypoints(i-1, 1:2));
end
end
