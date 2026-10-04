function recordTable = runAndRecordComplexReactiveScenario(scenario, gtTable, meta, simInfo, outDir)
%RUNANDRECORDCOMPLEXREACTIVESCENARIO Closed-loop simulation for a
%   complex, multi-crossroad, multi-lane reactive scenario: lane changes
%   for both ego and background traffic, mixed background vehicle types
%   with occasional sudden braking, mixed crossing actors (pedestrian/
%   child/cyclist, safe or risky), and an optional ego right turn / left
%   turn / U-turn at one crossroad.
%
%   Same output file layout as runAndRecordScenario.m / earlier reactive
%   scripts: scenario_XXX.mat, scenario_XXX_trajectories.csv,
%   scenario_XXX_labels.csv.

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

lib = complexScenarioLib();
dt  = scenario.SampleTime;

crossroads     = simInfo.crossroads;
crossingEvents = simInfo.crossingEvents;
p              = simInfo.idmParams;
numLanes       = simInfo.numLanes;
laneWidth      = simInfo.laneWidth;

ego      = simInfo.egoActor;
bgActors = simInfo.backgroundActors;
bgState  = simInfo.backgroundState;

egoManeuver       = simInfo.egoManeuver;
maneuverCrossroad = simInfo.maneuverCrossroad;

GAP_TRIGGER = 20;  % m: consider a lane change if the leader is closer than this
SAFETY_GAP  = 15;  % m: how much clearance an adjacent lane needs near ego/bg x

% ---- ego maneuver state machine ----
egoPhase             = 'approach';  % 'approach' | 'turning' | 'post_turn'
egoLane              = simInfo.egoLane0;
egoTargetLaneVal      = egoLane;
egoLaneChangeStartT   = -1;
egoLaneChangeDur      = 3;
turnStartT            = -1;
turnDur               = 4.5 + 1.5 * rand();
turnRadius            = 8;
turnStartPos          = [0 0];
turnEndPos            = [0 0];
postTurnDir           = [1 0];

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

    % ---- snapshot background positions/lanes/speeds for this step ----
    if ~isempty(bgActors)
        bgX      = arrayfun(@(a) a.Position(1), bgActors);
        bgVelArr = arrayfun(@(a) a.Velocity(1), bgActors);
    else
        bgX = []; bgVelArr = [];
    end
    bgLaneArr = [bgState.lane];

    % ---- 2) advance background vehicles (independent speed + lane changes) ----
    for k = 1:numel(bgActors)
        st = bgState(k);
        st.nextChange = st.nextChange - dt;
        if st.nextChange <= 0
            if rand() < 0.15 % ~15% chance of a sudden-braking pulse
                st.targetV = max(0, 0.5 * rand());
                st.brakeUntil = t + 2 + 2 * rand();
            else
                st.targetV = st.speedMin + (st.speedMax - st.speedMin) * rand();
            end
            st.nextChange = 6 + 8 * rand();
        end
        if st.brakeUntil > 0 && t > st.brakeUntil
            st.targetV = st.speedMin + (st.speedMax - st.speedMin) * rand();
            st.brakeUntil = -1;
        end

        curX = bgActors(k).Position(1);
        curV = bgActors(k).Velocity(1);

        if st.laneChangeStartT < 0
            otherIdx = setdiff(1:numel(bgActors), k);
            newTarget = lib.shouldChangeLane(curX, curV, st.lane, numLanes, ...
                [bgX(otherIdx), ego.Position(1)], ...
                [bgLaneArr(otherIdx), egoLane], ...
                [bgVelArr(otherIdx), ego.Velocity(1)], GAP_TRIGGER, SAFETY_GAP);
            if newTarget ~= st.lane
                st.targetLane = newTarget;
                st.laneChangeStartT = t;
            end
        end

        accK = sign(st.targetV - curV) * min(2.5, abs(st.targetV - curV));
        newV = max(0, curV + accK * dt);
        newX = curX + curV * dt + 0.5 * accK * dt^2;

        startY  = st.lane * laneWidth;
        targetY = st.targetLane * laneWidth;
        if st.laneChangeStartT >= 0
            frac = min(1, (t - st.laneChangeStartT) / st.laneChangeDur);
            newY = startY + frac * (targetY - startY);
            if frac >= 1
                st.lane = st.targetLane;
                st.laneChangeStartT = -1;
            end
        else
            newY = bgActors(k).Position(2);
        end

        bgActors(k).Velocity = [newV 0 0];
        bgActors(k).Position = [newX newY bgActors(k).Position(3)];
        bgState(k) = st;
    end

    % ---- 3) advance crossing actors (wait, cross during window, then stay) ----
    for e = 1:numel(crossingEvents)
        ev = crossingEvents(e);
        if t >= ev.startT && t <= ev.endT
            frac = (t - ev.startT) / max(ev.endT - ev.startT, 0.01);
            newY = ev.yStart + frac * (ev.yEnd - ev.yStart);
            ev.actorHandle.Position = [ev.xCross newY 0];
            ev.actorHandle.Velocity = [0, (ev.yEnd - ev.yStart) / (ev.endT - ev.startT), 0];
        elseif t > ev.endT
            ev.actorHandle.Position = [ev.xCross ev.yEnd 0];
            ev.actorHandle.Velocity = [0 0 0];
        else
            ev.actorHandle.Velocity = [0 0 0];
        end
    end

    % ---- 4) ego ----
    posEgoX = ego.Position(1);
    posEgoY = ego.Position(2);
    vEgo    = ego.Velocity(1);

    switch egoPhase
        case 'approach'
            if egoLaneChangeStartT < 0
                newTarget = lib.shouldChangeLane(posEgoX, vEgo, egoLane, numLanes, ...
                    bgX, bgLaneArr, bgVelArr, GAP_TRIGGER, SAFETY_GAP);
                if newTarget ~= egoLane
                    egoLaneChangeStartT = t;
                    egoTargetLaneVal = newTarget;
                end
            end

            [leaderPos, leaderVel, hasLeader] = lib.findLeaderInLane(posEgoX, egoLane, bgX, bgLaneArr, bgVelArr);

            [acc, ~] = lib.egoControlAccelerationMulti(t, posEgoX, vEgo, ...
                leaderPos, leaderVel, hasLeader, crossroads, crossingEvents, p);
            acc = max(min(acc, p.a), -6);

            newVEgo = max(0, vEgo + acc * dt);
            newXEgo = posEgoX + vEgo * dt + 0.5 * acc * dt^2;

            if egoLaneChangeStartT >= 0
                frac = min(1, (t - egoLaneChangeStartT) / egoLaneChangeDur);
                startY = egoLane * laneWidth;
                targY  = egoTargetLaneVal * laneWidth;
                newYEgo = startY + frac * (targY - startY);
                if frac >= 1
                    egoLane = egoTargetLaneVal;
                    egoLaneChangeStartT = -1;
                end
            else
                newYEgo = posEgoY;
            end

            ego.Velocity = [newVEgo 0 0];
            ego.Position = [newXEgo newYEgo ego.Position(3)];

            % check whether to begin the turn/U-turn at the chosen crossroad
            if ~strcmp(egoManeuver, 'straight') && maneuverCrossroad >= 1
                cr = crossroads(maneuverCrossroad);
                sigState = lib.trafficSignalState(t, cr.signal);
                if newXEgo >= cr.xSignal - 1 && strcmp(sigState, 'green')
                    egoPhase = 'turning';
                    turnStartT = t;
                    turnStartPos = [newXEgo, newYEgo];
                    switch egoManeuver
                        case 'right_turn'
                            turnEndPos = turnStartPos + [turnRadius, turnRadius];
                            postTurnDir = [0, 1];
                        case 'left_turn'
                            turnEndPos = turnStartPos + [turnRadius, -turnRadius];
                            postTurnDir = [0, -1];
                        case 'u_turn'
                            turnEndPos = turnStartPos + [-turnRadius * 0.5, turnRadius * 1.5];
                            postTurnDir = [-1, 0];
                    end
                end
            end

        case 'turning'
            s = min(1, (t - turnStartT) / turnDur);
            ease = (1 - cos(pi * s)) / 2; % smooth ease in/out
            newPos = turnStartPos + ease * (turnEndPos - turnStartPos);
            turnSpeed = 5;
            ego.Velocity = [turnSpeed * postTurnDir(1), turnSpeed * postTurnDir(2), 0];
            ego.Position = [newPos(1), newPos(2), ego.Position(3)];
            if s >= 1
                egoPhase = 'post_turn';
            end

        case 'post_turn'
            cruiseSpeed = 0.8 * p.v0;
            newPos = [ego.Position(1) + postTurnDir(1) * cruiseSpeed * dt, ...
                      ego.Position(2) + postTurnDir(2) * cruiseSpeed * dt];
            ego.Velocity = [cruiseSpeed * postTurnDir(1), cruiseSpeed * postTurnDir(2), 0];
            ego.Position = [newPos(1), newPos(2), ego.Position(3)];
    end
end

recordTable = cell2table(logRows, 'VariableNames', ...
    {'ScenarioID', 'Time', 'ActorID', 'X', 'Y', 'Z', 'Vx', 'Vy', 'Vz', 'Yaw'});

scenarioName = sprintf('scenario_%03d', meta.scenarioID);

save(fullfile(outDir, [scenarioName '.mat']), 'scenario', 'gtTable', 'meta', 'recordTable', 'simInfo');
writetable(recordTable, fullfile(outDir, [scenarioName '_trajectories.csv']));
writetable(gtTable, fullfile(outDir, [scenarioName '_labels.csv']));

end
