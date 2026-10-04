function bakeEgoTrajectoryFromLog(ego, recordTable, sampleEverySec)
%BAKEEGOTRAJECTORYFROMLOG Converts a recorded ego position/speed log
% (produced by manually driving the actor in a custom simulation loop)
% into an actual trajectory() assigned to the actor. This is needed
% because manually setting Actor.Position each timestep does NOT save a
% replayable path -- when the scenario is reloaded and restarted (e.g.
% in Driving Scenario Designer), the actor would otherwise sit still.
%
% recordTable must contain columns: Time, EgoX, EgoSpeed, and optionally
% EgoY (if absent, motion is assumed to be along a straight x-axis road).
% sampleEverySec: how often (in seconds) to keep a waypoint (e.g. 1).

dt = recordTable.Time(2) - recordTable.Time(1);
xVec = recordTable.EgoX;
if ismember('EgoY', recordTable.Properties.VariableNames)
    yVec = recordTable.EgoY;
else
    yVec = zeros(size(xVec));
end
speedVec = recordTable.EgoSpeed;

idxStep = max(1, round(sampleEverySec / dt));
idx = 1:idxStep:height(recordTable);
if idx(end) ~= height(recordTable)
    idx(end+1) = height(recordTable);
end

wp = [xVec(idx), yVec(idx), zeros(numel(idx), 1)];
spd = speedVec(idx);

% Remove consecutive duplicate waypoints (can happen while stopped)
keep = true(size(spd));
for k = 2:numel(spd)
    if norm(wp(k, :) - wp(k-1, :)) < 1e-3
        keep(k) = false;
    end
end
wp = wp(keep, :);
spd = spd(keep);

% Merge consecutive zero-speed entries (trajectory() rejects repeated zeros)
keep2 = true(size(spd));
for k = 2:numel(spd)
    if spd(k) == 0 && spd(k-1) == 0
        keep2(k) = false;
    end
end
wp = wp(keep2, :);
spd = spd(keep2);

if size(wp, 1) < 2
    return % nothing meaningful to bake (e.g. ego never moved)
end

trajectory(ego, wp, spd);
end
