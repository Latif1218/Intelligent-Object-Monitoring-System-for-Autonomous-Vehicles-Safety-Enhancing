function recordTable = runEgoFollowingScenario(scenario, ego, meta, outDir)
%RUNEGOFOLLOWINGSCENARIO Advances the scenario while manually driving the
% ego vehicle. At every timestep, ego's target speed is the MINIMUM of:
%   1) a car-following speed based on the gap/speed of whatever vehicle
%      is directly ahead of it (reacts to the "front vehicle")
%   2) a traffic-signal speed (must slow to a stop before the stop line
%      on red, free to go on green)
%   3) a pedestrian-yield speed (stops if a pedestrian is in the
%      crosswalk ahead)
% All other actors (lead vehicle, background traffic, pedestrians) move
% on their own independent, predefined trajectories -- ego is the only
% one being actively controlled here.

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

dt = scenario.SampleTime;
maxSpeed  = 15;   % m/s, ego's free-flow cruising speed
maxDecel  = 4;    % m/s^2
maxAccel  = 2;    % m/s^2
safeTimeGap = 2.0; % seconds, desired headway to the vehicle ahead
minGap    = 5;    % meters, minimum bumper-to-bumper gap

% Map ActorID -> Name once (actorPoses does not return names directly)
actorNames = arrayfun(@(a) a.Name, scenario.Actors, 'UniformOutput', false);
actorIDs   = arrayfun(@(a) a.ActorID, scenario.Actors);

egoPos = [0 0 0];
egoSpeed = meta.egoInitialSpeed;

logRows = {};
restart(scenario);

while advance(scenario)
    t = scenario.SimulationTime;
    poses = actorPoses(scenario);

    % --- 1) Car-following: find the nearest actor ahead, in the same lane ---
    gapToLead = inf;
    leadSpeed = maxSpeed;
    for ii = 1:numel(poses)
        idx = find(actorIDs == poses(ii).ActorID, 1);
        nm = actorNames{idx};
        if strcmp(nm, 'Ego')
            continue
        end
        isVehicleLike = ~startsWith(nm, 'pedestrian');
        dx = poses(ii).Position(1) - egoPos(1);
        dy = abs(poses(ii).Position(2) - egoPos(2));
        if isVehicleLike && dx > 0 && dy < 2 && dx < gapToLead
            gapToLead = dx;
            leadSpeed = norm(poses(ii).Velocity(1:2));
        end
    end

    desiredGap = minGap + safeTimeGap * egoSpeed;
    if gapToLead < desiredGap
        speedTarget_lead = max(0, leadSpeed - (desiredGap - gapToLead) * 0.5);
    else
        speedTarget_lead = maxSpeed;
    end

    % --- 2) Traffic signal ---
    signalState = trafficSignalState(t, meta);
    distToStopLine = (meta.intersectionX - meta.stopLineDist) - egoPos(1);
    if strcmp(signalState, 'red') && distToStopLine > -2 && distToStopLine < 40
        brakingDist = max(egoSpeed^2 / (2 * maxDecel), 1);
        if distToStopLine < brakingDist
            speedTarget_signal = max(0, egoSpeed - maxDecel * dt * 3);
        else
            speedTarget_signal = maxSpeed;
        end
        if distToStopLine <= 1
            speedTarget_signal = 0;
        end
    else
        speedTarget_signal = maxSpeed;
    end

    % --- 3) Pedestrians in the crosswalk ahead ---
    speedTarget_ped = maxSpeed;
    for ii = 1:numel(poses)
        idx = find(actorIDs == poses(ii).ActorID, 1);
        nm = actorNames{idx};
        if startsWith(nm, 'pedestrian')
            dx = poses(ii).Position(1) - egoPos(1);
            dy = abs(poses(ii).Position(2));
            if dx > 0 && dx < 20 && dy < 4
                speedTarget_ped = 0;
            end
        end
    end

    % --- Combine: ego obeys whichever requirement is most restrictive ---
    speedTarget = min([speedTarget_lead, speedTarget_signal, speedTarget_ped, maxSpeed]);

    % --- Apply acceleration/deceleration limits ---
    if speedTarget > egoSpeed
        egoSpeed = min(speedTarget, egoSpeed + maxAccel * dt);
    else
        egoSpeed = max(speedTarget, egoSpeed - maxDecel * dt);
    end
    egoSpeed = max(egoSpeed, 0);

    % --- Integrate ego position and push it into the scenario ---
    egoPos = egoPos + [egoSpeed * dt, 0, 0];
    ego.Position = egoPos;
    ego.Velocity = [egoSpeed 0 0];

    % --- Log everything for the dataset ---
    logRows(end+1, :) = {meta.scenarioID, t, egoPos(1), egoSpeed, gapToLead, leadSpeed, ...
        signalState, speedTarget}; %#ok<AGROW>
end

recordTable = cell2table(logRows, 'VariableNames', ...
    {'ScenarioID', 'Time', 'EgoX', 'EgoSpeed', 'GapToLead', 'LeadSpeed', 'SignalState', 'TargetSpeed'});

% Bake the recorded ego path into a real trajectory so this scenario can
% be reloaded and replayed later (e.g. in Driving Scenario Designer).
bakeEgoTrajectoryFromLog(ego, recordTable, 1);

scenarioName = sprintf('egofollow_scenario_%03d', meta.scenarioID);
save(fullfile(outDir, [scenarioName '.mat']), 'scenario', 'meta', 'recordTable');
writetable(recordTable, fullfile(outDir, [scenarioName '_egolog.csv']));

end
