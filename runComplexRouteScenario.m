function recordTable = runComplexRouteScenario(scenario, ego, meta, outDir)
%RUNCOMPLEXROUTESCENARIO Advances the scenario while driving ego along
% its precomputed route (meta.egoWaypoints/egoCumDist). At every
% timestep, ego's forward speed is the MINIMUM of:
%   1) car-following speed based on whatever is directly ahead of it
%   2) the NEXT upcoming traffic signal along its route (stop on red)
%   3) pedestrian-yield speed (stop if a pedestrian is in the crosswalk ahead)
% and ego additionally shifts laterally (a simple lane change) to avoid
% any stalled obstacle vehicle blocking its path ahead.

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

dt = scenario.SampleTime;
maxSpeed  = 15;    % m/s
maxDecel  = 4;     % m/s^2
maxAccel  = 2;     % m/s^2
safeTimeGap = 2.0; % s
minGap    = 5;     % m

laneWidth   = 3.5;
maxLatRate  = 1.5; % m/s, how fast ego can shift laterally
lookaheadObs = 30; % m, distance at which ego starts avoiding an obstacle
passMargin   = 15; % m, distance after an obstacle before merging back

actorNames = arrayfun(@(a) a.Name, scenario.Actors, 'UniformOutput', false);
actorIDs   = arrayfun(@(a) a.ActorID, scenario.Actors);

s = 0;               % distance traveled along the route
egoSpeed = meta.egoInitialSpeed;
lateralOffset = 0;

logRows = {};
restart(scenario);

while advance(scenario)
    t = scenario.SimulationTime;
    poses = actorPoses(scenario);
    [basePos, yawDeg] = pathPointAtDistance(meta.egoWaypoints, meta.egoCumDist, s);
    yawRad = deg2rad(yawDeg);
    perp = [-sin(yawRad), cos(yawRad)]; % unit vector perpendicular to heading
    egoPos = basePos + [perp * lateralOffset, 0];

    % --- 1) Car-following: nearest actor ahead, roughly in the same lane ---
    gapToLead = inf;
    leadSpeed = maxSpeed;
    for ii = 1:numel(poses)
        idx = find(actorIDs == poses(ii).ActorID, 1);
        nm = actorNames{idx};
        if strcmp(nm, 'Ego')
            continue
        end
        rel = poses(ii).Position(1:2) - egoPos(1:2);
        along = rel(1)*cos(yawRad) + rel(2)*sin(yawRad); % distance ahead, along heading
        lateral = -rel(1)*sin(yawRad) + rel(2)*cos(yawRad); % offset to the side
        if along > 0 && abs(lateral) < 2.2 && along < gapToLead
            gapToLead = along;
            leadSpeed = norm(poses(ii).Velocity(1:2));
        end
    end
    desiredGap = minGap + safeTimeGap * egoSpeed;
    if gapToLead < desiredGap
        speedTarget_lead = max(0, leadSpeed - (desiredGap - gapToLead) * 0.5);
    else
        speedTarget_lead = maxSpeed;
    end

    % --- 2) Next traffic signal ahead along the route ---
    speedTarget_signal = maxSpeed;
    signalState = 'none';
    nextEventDist = inf;
    for e = 1:numel(meta.intersectionEvents)
        ev = meta.intersectionEvents(e);
        distToStop = (ev.s - ev.stopLineDist) - s;
        if distToStop > -2 && distToStop < nextEventDist
            nextEventDist = distToStop;
            phase = mod(t + ev.phaseOffset, ev.cycleLen);
            if phase < ev.greenDuration
                signalState = 'green';
            else
                signalState = 'red';
            end
            if strcmp(signalState, 'red') && distToStop < 40
                brakingDist = max(egoSpeed^2 / (2*maxDecel), 1);
                if distToStop < brakingDist
                    speedTarget_signal = max(0, egoSpeed - maxDecel*dt*3);
                end
                if distToStop <= 1
                    speedTarget_signal = 0;
                end
            end
        end
    end

    % --- 3) Pedestrians ahead in a crosswalk ---
    speedTarget_ped = maxSpeed;
    for ii = 1:numel(poses)
        idx = find(actorIDs == poses(ii).ActorID, 1);
        nm = actorNames{idx};
        if contains(nm, 'ped')
            rel = poses(ii).Position(1:2) - egoPos(1:2);
            along = rel(1)*cos(yawRad) + rel(2)*sin(yawRad);
            lateral = -rel(1)*sin(yawRad) + rel(2)*cos(yawRad);
            if along > 0 && along < 20 && abs(lateral) < 4
                speedTarget_ped = 0;
            end
        end
    end

    % --- Combine forward-speed requirements ---
    speedTarget = min([speedTarget_lead, speedTarget_signal, speedTarget_ped, maxSpeed]);
    if speedTarget > egoSpeed
        egoSpeed = min(speedTarget, egoSpeed + maxAccel*dt);
    else
        egoSpeed = max(speedTarget, egoSpeed - maxDecel*dt);
    end
    egoSpeed = max(egoSpeed, 0);

    % --- Lateral lane-change to avoid obstacles ahead ---
    targetLateral = 0;
    for o = 1:numel(meta.obstacleEvents)
        obsS = meta.obstacleEvents(o).s;
        if s + lookaheadObs > obsS && s < obsS + passMargin
            targetLateral = laneWidth;
        end
    end
    if lateralOffset < targetLateral
        lateralOffset = min(targetLateral, lateralOffset + maxLatRate*dt);
    else
        lateralOffset = max(targetLateral, lateralOffset - maxLatRate*dt);
    end

    % --- Advance ego along the path and push into the scenario ---
    s = s + egoSpeed * dt;
    [basePos, yawDeg] = pathPointAtDistance(meta.egoWaypoints, meta.egoCumDist, s);
    yawRad = deg2rad(yawDeg);
    perp = [-sin(yawRad), cos(yawRad)];
    finalPos = basePos + [perp * lateralOffset, 0];

    ego.Position = finalPos;
    ego.Yaw = yawDeg;
    ego.Velocity = egoSpeed * [cos(yawRad), sin(yawRad), 0];

    logRows(end+1, :) = {meta.scenarioID, t, s, finalPos(1), finalPos(2), egoSpeed, ...
        lateralOffset, gapToLead, leadSpeed, signalState, speedTarget}; %#ok<AGROW>
end

recordTable = cell2table(logRows, 'VariableNames', ...
    {'ScenarioID', 'Time', 'DistanceTraveled', 'EgoX', 'EgoY', 'EgoSpeed', ...
     'LateralOffset', 'GapToLead', 'LeadSpeed', 'SignalState', 'TargetSpeed'});

% Bake the recorded ego path into a real trajectory so this scenario can
% be reloaded and replayed later (e.g. in Driving Scenario Designer).
bakeEgoTrajectoryFromLog(ego, recordTable, 1);

scenarioName = sprintf('scenario_%03d', meta.scenarioID);
save(fullfile(outDir, [scenarioName '_complexroute.mat']), 'scenario', 'meta', 'recordTable');
writetable(recordTable, fullfile(outDir, [scenarioName '_egolog.csv']));

end