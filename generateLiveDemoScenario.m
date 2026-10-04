function [scenario, ego, gtTable, meta] = generateLiveDemoScenario(rngSeed)
%GENERATELIVEDEMOSCENARIO Builds one long (~10-12 min) demo scenario with
% several left/right turns and a U-turn, multiple crossroads (mixed
% car/truck/bicycle/pedestrian traffic + one rail crossing with a train),
% and lane-blocking obstacles. Ego has NO fixed trajectory -- it is
% driven live and reactively in runLiveClassifiedDemo.m.

rng(rngSeed);

% --- Route definition: a long chain of straights + turns ---
legs = struct( ...
    'straightLength', {1500, 1800, 1500, 2000, 500, 2500}, ...
    'turnAngleDeg',   {90,   -90,  90,   0,    180, 0});

egoWaypoints = buildChainedEgoRoute(legs);
egoCumDist = pathCumulativeDistance(egoWaypoints);
totalLen = egoCumDist(end);

scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 650); % ~10.8 minutes

% --- Physical main road follows the exact curved/turning route ---
road(scenario, egoWaypoints, 'Lanes', lanespec(2, 'Width', 3.5), 'Name', 'MainRoute');

% --- Ego vehicle: position only, no trajectory (driven live later) ---
ego = vehicle(scenario, 'ClassID', 1, 'Name', 'Ego', ...
    'Length', 4.7, 'Width', 1.8, 'Height', 1.4, 'Position', egoWaypoints(1, :));

actorLog = struct('name', {}, 'actorType', {}, 'label', {}, 'description', {}, 'startTime', {});

% --- Crossroads spread along the whole route, each with its own signal ---
numCrossroads = 6;
crossFractions = linspace(0.12, 0.90, numCrossroads);
crossS = crossFractions * totalLen;

intersectionEvents = struct('s', {}, 'redDuration', {}, 'greenDuration', {}, ...
    'cycleLen', {}, 'phaseOffset', {}, 'stopLineDist', {});

trainCrossroadIdx = numCrossroads - 1; % one crossroad becomes a rail crossing

for k = 1:numCrossroads
    [posK, yawK] = pathPointAtDistance(egoWaypoints, egoCumDist, crossS(k));
    yawRad = deg2rad(yawK);
    dirVec = [cos(yawRad), sin(yawRad)];
    perpVec = [-sin(yawRad), cos(yawRad)];

    crossHalfLen = 150;
    crossStart = posK(1:2) - crossHalfLen * perpVec;
    crossEnd = posK(1:2) + crossHalfLen * perpVec;
    road(scenario, [crossStart 0; posK; crossEnd 0], 'Lanes', lanespec(2, 'Width', 3.3), ...
        'Name', sprintf('Cross%d', k));

    redDur = randi([15 30]);
    greenDur = randi([15 30]);
    cyc = redDur + greenDur;
    offs = randi([0 cyc-1]);
    intersectionEvents(end+1) = struct('s', crossS(k), 'redDuration', redDur, ... %#ok<AGROW>
        'greenDuration', greenDur, 'cycleLen', cyc, 'phaseOffset', offs, 'stopLineDist', 15);

    if k == trainCrossroadIdx
        % --- Rail crossing: a fast "Train" actor sweeps across the road ---
        trainWp = [posK - [dirVec*0 , 0] + [perpVec*-160, 0]; posK; posK + [perpVec*160, 0]];
        trainAct = vehicle(scenario, 'ClassID', 2, 'Name', sprintf('Train_%d', k), ...
            'Length', 60, 'Width', 3, 'Height', 4.5);
        trajectory(trainAct, trainWp, [28 28 28]);
        actorLog(end+1) = struct('name', sprintf('Train_%d', k), 'actorType', 'train', 'label', 1, ... %#ok<AGROW>
            'description', 'Fast train crossing the road at a level crossing.', 'startTime', 0);
    else
        % --- Normal background traffic crossing this intersection ---
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
                act = actor(scenario, 'ClassID', classID, 'Name', name, 'Length', len, 'Width', wid, 'Height', ht);
            else
                act = vehicle(scenario, 'ClassID', classID, 'Name', name, 'Length', len, 'Width', wid, 'Height', ht);
            end
            offsetStart = -140 - (b-1) * 25;
            p2 = posK(1:2);
            wp = [p2 + perpVec*offsetStart, 0; p2 + perpVec*-20, 0; p2 + perpVec*20, 0; p2 + perpVec*100, 0];
            spd = randi([4 15], 1, size(wp, 1));
            trajectory(act, wp, spd);
            actorLog(end+1) = struct('name', name, 'actorType', actType, 'label', 0, ... %#ok<AGROW>
                'description', 'Background traffic with independently varying speed.', 'startTime', 0);
        end
    end

    % --- Pedestrians at this intersection: mix of safe and risky ---
    numPed = randi([0 2]);
    for p = 1:numPed
        pedName = sprintf('cross%d_ped_%d', k, p);
        ped = actor(scenario, 'ClassID', 4, 'Name', pedName, 'Length', 0.5, 'Width', 0.5, 'Height', 1.7);
        offX = -10 + rand()*20;
        pt2 = posK(1:2) + dirVec*offX;
        wp = [pt2 + perpVec*-4, 0; pt2, 0; pt2 + perpVec*4, 0];
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

% --- Stalled obstacle vehicles blocking ego's lane, forcing lane changes ---
numObstacles = 3;
obstacleEvents = struct('s', {}, 'position', {});
candidateS = sort(300 + rand(1, numObstacles) * (totalLen - 600));
for o = 1:numel(candidateS)
    [obsPos, ~] = pathPointAtDistance(egoWaypoints, egoCumDist, candidateS(o));
    obsName = sprintf('obstacle_%d', o);
    obsAct = vehicle(scenario, 'ClassID', randi([1 2]), 'Name', obsName, ...
        'Length', 4.5, 'Width', 1.9, 'Height', 1.5);
    trajectory(obsAct, [obsPos; obsPos + [3 0 0]], 0.3);
    obstacleEvents(end+1) = struct('s', candidateS(o), 'position', obsPos); %#ok<AGROW>
    actorLog(end+1) = struct('name', obsName, 'actorType', 'car', 'label', 1, ... %#ok<AGROW>
        'description', 'Stalled/parked vehicle blocking the lane, forcing ego to change lanes.', 'startTime', 0);
end

gtTable = struct2table(actorLog);

meta.roadType = 'live_demo_long_route';
meta.duration = scenario.StopTime;
meta.rngSeed = rngSeed;
meta.egoWaypoints = egoWaypoints;
meta.egoCumDist = egoCumDist;
meta.intersectionEvents = intersectionEvents;
meta.obstacleEvents = obstacleEvents;
meta.egoInitialSpeed = 12; % m/s
end
