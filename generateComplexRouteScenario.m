function [scenario, ego, gtTable, meta] = generateComplexRouteScenario(scenarioID, rngSeed)
%GENERATECOMPLEXROUTESCENARIO Builds a long, multi-intersection route
% scenario where:
%   - Ego drives through 2-3 crossroads, each with its own traffic signal
%   - Ego performs one of: straight / right turn / left turn / U-turn
%   - Ego reacts to a vehicle ahead (car-following), signals, pedestrians,
%     AND changes lane to avoid a stalled obstacle vehicle blocking its path
%   - Background traffic mixes car / truck / bicycle, crossing at each
%     intersection, plus pedestrians (some safe, some risky)
%   - Ego has NO fixed trajectory; its motion is computed live in
%     runComplexRouteScenario.m

rng(rngSeed);

numCrossroads = randi([2 3]);
[scenario, roadInfo] = createMultiCrossroadNetwork(numCrossroads);

turnTypes = {'straight', 'right', 'left', 'uturn'};
turnType = turnTypes{randi(numel(turnTypes))};

if strcmp(turnType, 'straight')
    turnX = NaN;
elseif strcmp(turnType, 'uturn')
    turnX = roadInfo.crossroadXs(1) + rand() * (roadInfo.crossroadXs(end) - roadInfo.crossroadXs(1));
else
    turnAtIdx = randi(numCrossroads);
    turnX = roadInfo.crossroadXs(turnAtIdx);
end

farDistance = 3000;
egoWaypoints = buildEgoRoute(turnType, turnX, farDistance);
egoCumDist = pathCumulativeDistance(egoWaypoints);

% --- Ego vehicle: position only, no trajectory (driven live later) ---
ego = vehicle(scenario, 'ClassID', 1, 'Name', 'Ego', ...
    'Length', 4.7, 'Width', 1.8, 'Height', 1.4, 'Position', [0 0 0]);

actorLog = struct('name', {}, 'actorType', {}, 'label', {}, 'description', {}, 'startTime', {});

% --- One traffic signal per crossroad, own random timing, matched to
%     the arc-length position(s) where ego's route crosses that x ---
intersectionEvents = struct('s', {}, 'crossroadIdx', {}, 'redDuration', {}, ...
    'greenDuration', {}, 'cycleLen', {}, 'phaseOffset', {}, 'stopLineDist', {});

for k = 1:numCrossroads
    xk = roadInfo.crossroadXs(k);
    redDur = randi([15 30]);
    greenDur = randi([15 30]);
    cyc = redDur + greenDur;
    offs = randi([0 cyc-1]);

    sCrossings = findPathCrossingsAtX(egoWaypoints, egoCumDist, xk);
    for c = 1:numel(sCrossings)
        intersectionEvents(end+1) = struct('s', sCrossings(c), 'crossroadIdx', k, ... %#ok<AGROW>
            'redDuration', redDur, 'greenDuration', greenDur, 'cycleLen', cyc, ...
            'phaseOffset', offs, 'stopLineDist', 15);
    end

    % --- Background traffic crossing this intersection: mix of car/truck/bicycle ---
    numBg = randi([1 3]);
    for b = 1:numBg
        types = {'car', 'truck', 'bicycle'};
        actType = types{randi(numel(types))};
        switch actType
            case 'car'
                classID = 1; len = 4.5; wid = 1.8; ht = 1.4;
            case 'truck'
                classID = 2; len = 8.0; wid = 2.4; ht = 3.0;
            case 'bicycle'
                classID = 3; len = 1.8; wid = 0.6; ht = 1.6;
        end
        name = sprintf('cross%d_bg_%d', k, b);
        if classID == 3
            act = actor(scenario, 'ClassID', classID, 'Name', name, ...
                'Length', len, 'Width', wid, 'Height', ht);
        else
            act = vehicle(scenario, 'ClassID', classID, 'Name', name, ...
                'Length', len, 'Width', wid, 'Height', ht);
        end
        yStart = -140 - (b-1)*25; % always clear of the other waypoints (-20, 20, 100)
        wp = [xk yStart 0; xk -20 0; xk 20 0; xk 100 0];
        spd = randi([4 15], 1, size(wp,1));
        trajectory(act, wp, spd);
        actorLog(end+1) = struct('name', name, 'actorType', actType, 'label', 0, ... %#ok<AGROW>
            'description', 'Background traffic with independently varying speed.', 'startTime', 0);
    end

    % --- Pedestrians at this intersection: mix of safe and risky ---
    numPed = randi([0 2]);
    for p = 1:numPed
        pedName = sprintf('cross%d_ped_%d', k, p);
        ped = actor(scenario, 'ClassID', 4, 'Name', pedName, ...
            'Length', 0.5, 'Width', 0.5, 'Height', 1.7);
        xOffset = xk - 10 + rand()*20;
        wp = [xOffset -4 0; xOffset 0 0; xOffset 4 0];
        isSafe = rand() > 0.4;
        if isSafe
            spd = [1.2 1.2 1.2]; label = 0;
            desc = 'Pedestrian waits and crosses only when it is safe to do so.';
        else
            spd = [3 3.5 3]; label = 1;
            desc = 'Pedestrian crosses without properly checking traffic or the signal.';
        end
        trajectory(ped, wp, spd);
        actorLog(end+1) = struct('name', pedName, 'actorType', 'pedestrian', 'label', label, ... %#ok<AGROW>
            'description', desc, 'startTime', 0);
    end
end

% --- Lead vehicle ahead of ego on the main road, own varying speed ---
leadVeh = vehicle(scenario, 'ClassID', 1, 'Name', 'LeadVehicle', ...
    'Length', 4.5, 'Width', 1.8, 'Height', 1.4);
leadWp = [30 0 0; 300 0 0; 700 0 0; 1500 0 0; 2500 0 0];
leadSpd = randi([5 18], 1, size(leadWp,1));
trajectory(leadVeh, leadWp, leadSpd);
actorLog(end+1) = struct('name', 'LeadVehicle', 'actorType', 'car', 'label', 0, ...
    'description', 'Lead vehicle with its own independently varying speed profile.', 'startTime', 0);

% --- Stalled obstacle vehicle(s) blocking ego's lane, forcing a lane change ---
totalLen = egoCumDist(end);
numObstacles = randi([1 2]);
obstacleEvents = struct('s', {}, 'position', {});
candidateS = sort(200 + rand(1, numObstacles) * (min(totalLen, 2600) - 400));
for o = 1:numel(candidateS)
    [obsPos, ~] = pathPointAtDistance(egoWaypoints, egoCumDist, candidateS(o));
    obsName = sprintf('obstacle_%d', o);
    obsAct = vehicle(scenario, 'ClassID', randi([1 2]), 'Name', obsName, ...
        'Length', 4.5, 'Width', 1.9, 'Height', 1.5);
    trajectory(obsAct, [obsPos; obsPos + [3 0 0]], 0.3); % nearly stationary, blocking the lane
    obstacleEvents(end+1) = struct('s', candidateS(o), 'position', obsPos); %#ok<AGROW>
    actorLog(end+1) = struct('name', obsName, 'actorType', 'car', 'label', 1, ... %#ok<AGROW>
        'description', 'Stalled/parked vehicle blocking the lane, forcing ego to change lanes.', 'startTime', 0);
end

gtTable = struct2table(actorLog);

meta.scenarioID = scenarioID;
meta.roadType = 'complex_multi_crossroad';
meta.duration = scenario.StopTime;
meta.rngSeed = rngSeed;
meta.turnType = turnType;
meta.turnX = turnX;
meta.numCrossroads = numCrossroads;
meta.crossroadXs = roadInfo.crossroadXs;
meta.egoWaypoints = egoWaypoints;
meta.egoCumDist = egoCumDist;
meta.intersectionEvents = intersectionEvents;
meta.obstacleEvents = obstacleEvents;
meta.egoInitialSpeed = 12; % m/s

end