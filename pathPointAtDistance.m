function [pos, yawDeg] = pathPointAtDistance(waypoints, cumDist, s)
%PATHPOINTATDISTANCE Interpolates [x y z] position and yaw (degrees) at
% arc-length distance s along a waypoint polyline.

totalLen = cumDist(end);
s = max(0, min(s, totalLen));

segIdx = find(cumDist >= s, 1, 'first');
if isempty(segIdx) || segIdx <= 1
    segIdx = 2;
end

p0 = waypoints(segIdx-1, :);
p1 = waypoints(segIdx, :);
segLen = cumDist(segIdx) - cumDist(segIdx-1);

if segLen < 1e-6
    frac = 0;
else
    frac = (s - cumDist(segIdx-1)) / segLen;
end

pos = p0 + frac * (p1 - p0);
dirVec = p1 - p0;
yawDeg = atan2d(dirVec(2), dirVec(1));
end
