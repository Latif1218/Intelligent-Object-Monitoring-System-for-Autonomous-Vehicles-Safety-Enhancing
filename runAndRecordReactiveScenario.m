function recordTable = runAndRecordReactiveScenario(scenario, gtTable, meta, simInfo, outDir)
%RUNANDRECORDREACTIVESCENARIO Advances a reactive scenario to completion.
%
%   Unlike runAndRecordScenario.m (which just plays back actors that
%   already have a fixed trajectory()), this function actually DRIVES
%   the actors: at every timestep it
%     1) logs the current pose of every actor,
%     2) advances each background vehicle along its own independent,
%        randomly-varying speed profile,
%     3) advances each pedestrian (stationary, then crosses during its
%        event window, then stationary again),
%     4) computes the ego vehicle's acceleration in real time via IDM
%        car-following against whichever vehicle is currently ahead of
%        it, combined with the traffic signal and pedestrian crossings
%        (see reactiveCarFollowingLib.m), and moves the ego accordingly.
%
%   Saves the same file layout as runAndRecordScenario.m so it fits the
%   existing dataset_100_scenarios pipeline:
%       scenario_XXX.mat              (scenario, gtTable, meta, recordTable, simInfo)
%       scenario_XXX_trajectories.csv (ScenarioID,Time,ActorID,X,Y,Z,Vx,Vy,Vz,Yaw)
%       scenario_XXX_labels.csv       (gtTable)

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

lib = reactiveCarFollowingLib();
dt  = scenario.SampleTime;

road      = simInfo.road;
sig       = simInfo.signal;
pedEvents = simInfo.pedEvents;
p         = simInfo.idmParams;

ego       = simInfo.egoActor;
bgActors  = simInfo.backgroundActors;
bgState   = simInfo.backgroundState;    % struct array: targetV, nextChange
pedActors = simInfo.pedestrianActors;   % struct array: actorHandle, startT, endT, xCross, yStart, yEnd

logRows = {};
restart(scenario);

while advance(scenario)
    t = scenario.SimulationTime;

    % ---- 1) log current pose of every actor ----
    poses = actorPoses(scenario);
    for i = 1:numel(poses)
        pz = poses(i);
        logRows(end+1, :) = {meta.scenarioID, t, pz.ActorID, ...
            pz.Position(1), pz.Position(2), pz.Position(3), ...
            pz.Velocity(1), pz.Velocity(2), pz.Velocity(3), pz.Yaw}; %#ok<AGROW>
    end

    % ---- 2) advance background vehicles (independent random speed) ----
    for k = 1:numel(bgActors)
        st = bgState(k);
        st.nextChange = st.nextChange - dt;
        if st.nextChange <= 0
            st.targetV    = p.bgSpeedMin + (p.bgSpeedMax - p.bgSpeedMin) * rand();
            st.nextChange = 8 + 7 * rand();
        end
        curV = bgActors(k).Velocity(1);
        curX = bgActors(k).Position(1);
        accK = sign(st.targetV - curV) * min(1.5, abs(st.targetV - curV));
        newV = max(0, curV + accK * dt);
        newX = curX + curV * dt + 0.5 * accK * dt^2;
        bgActors(k).Velocity = [newV 0 0];
        bgActors(k).Position = [newX bgActors(k).Position(2) bgActors(k).Position(3)];
        bgState(k) = st;
    end

    % ---- 3) advance pedestrians (wait, cross during their window, then stay) ----
    for k = 1:numel(pedActors)
        pk = pedActors(k);
        if t >= pk.startT && t <= pk.endT
            frac = (t - pk.startT) / max(pk.endT - pk.startT, 0.01);
            newY = pk.yStart + frac * (pk.yEnd - pk.yStart);
            pk.actorHandle.Position = [pk.xCross newY 0];
            pk.actorHandle.Velocity = [0, (pk.yEnd - pk.yStart) / (pk.endT - pk.startT), 0];
        elseif t > pk.endT
            pk.actorHandle.Position = [pk.xCross pk.yEnd 0];
            pk.actorHandle.Velocity = [0 0 0];
        else
            pk.actorHandle.Velocity = [0 0 0];
        end
    end

    % ---- 4) ego: reactive IDM control based on whoever is ahead right now ----
    posEgo = ego.Position(1);
    vEgo   = ego.Velocity(1);

    if ~isempty(bgActors)
        bgPos = arrayfun(@(a) a.Position(1), bgActors);
        bgVel = arrayfun(@(a) a.Velocity(1), bgActors);
    else
        bgPos = []; bgVel = [];
    end
    aheadMask = bgPos > posEgo;
    hasLeader = any(aheadMask);
    if hasLeader
        aheadPos = bgPos(aheadMask);
        aheadVel = bgVel(aheadMask);
        [leaderPos, idxMin] = min(aheadPos);
        leaderVel = aheadVel(idxMin);
    else
        leaderPos = NaN; leaderVel = NaN;
    end

    [acc, ~] = lib.egoControlAcceleration(t, posEgo, vEgo, ...
        leaderPos, leaderVel, hasLeader, road, sig, pedEvents, p);
    acc = max(min(acc, p.a), -6); % clip to plausible accel/brake bounds

    newVEgo = max(0, vEgo + acc * dt);
    newXEgo = posEgo + vEgo * dt + 0.5 * acc * dt^2;
    ego.Velocity = [newVEgo 0 0];
    ego.Position = [newXEgo ego.Position(2) ego.Position(3)];
end

recordTable = cell2table(logRows, 'VariableNames', ...
    {'ScenarioID', 'Time', 'ActorID', 'X', 'Y', 'Z', 'Vx', 'Vy', 'Vz', 'Yaw'});

scenarioName = sprintf('scenario_%03d', meta.scenarioID);

save(fullfile(outDir, [scenarioName '.mat']), 'scenario', 'gtTable', 'meta', 'recordTable', 'simInfo');
writetable(recordTable, fullfile(outDir, [scenarioName '_trajectories.csv']));
writetable(gtTable, fullfile(outDir, [scenarioName '_labels.csv']));

end
