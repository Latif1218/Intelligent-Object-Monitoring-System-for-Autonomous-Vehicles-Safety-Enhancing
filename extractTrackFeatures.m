function feat = extractTrackFeatures(t, x, y, speed, egoX, egoY)
%EXTRACTTRACKFEATURES Computes a fixed set of behavior features from a
% single track/actor's time-series data. The SAME formula must be used
% for both ground-truth and sensor-tracked data for a fair, apples-to-
% apples robustness comparison.
%
% t, x, y, speed: column vectors (or vectors), same length, sorted by time.
% egoX, egoY (optional): ego vehicle's position, already interpolated to
% the same timestamps t, used to compute distance-to-ego features.

t = t(:); x = x(:); y = y(:); speed = speed(:);
if nargin < 6
    egoX = [];
    egoY = [];
end

if numel(t) < 4
    feat = table(NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, 'VariableNames', ...
        {'avg_speed', 'max_speed', 'speed_std', 'max_accel', 'max_decel', ...
         'max_lateral_rate', 'yaw_rate_std', 'min_dist_to_ego', 'avg_dist_to_ego', 'duration'});
    return
end

dt = diff(t);
dt(dt <= 0) = median(dt(dt > 0));

accel = diff(speed) ./ dt;
lateralRate = abs(diff(y)) ./ dt;

% Heading (yaw) from consecutive positions, then its rate of change
yawRad = atan2(diff(y), diff(x));
yawUnwrapped = unwrap(yawRad);
yawRate = diff(yawUnwrapped) ./ dt(2:end);
yawRateStdVal = std(yawRate, 'omitnan');

% Distance to ego, if ego position was supplied
if ~isempty(egoX)
    egoX = egoX(:); egoY = egoY(:);
    distToEgo = sqrt((x - egoX).^2 + (y - egoY).^2);
    minDist = min(distToEgo, [], 'omitnan');
    avgDist = mean(distToEgo, 'omitnan');
else
    minDist = NaN;
    avgDist = NaN;
end

feat = table( ...
    mean(speed, 'omitnan'), ...
    max(speed, [], 'omitnan'), ...
    std(speed, 'omitnan'), ...
    max(accel, [], 'omitnan'), ...
    min(accel, [], 'omitnan'), ...
    max(lateralRate, [], 'omitnan'), ...
    yawRateStdVal, ...
    minDist, ...
    avgDist, ...
    t(end) - t(1), ...
    'VariableNames', {'avg_speed', 'max_speed', 'speed_std', 'max_accel', ...
        'max_decel', 'max_lateral_rate', 'yaw_rate_std', 'min_dist_to_ego', ...
        'avg_dist_to_ego', 'duration'});
end