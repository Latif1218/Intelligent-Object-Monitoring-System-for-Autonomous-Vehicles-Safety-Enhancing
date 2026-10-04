function [scenario, gtTable, meta, simInfo] = generateComplexReactiveScenario(scenarioID, rngSeed)
%GENERATECOMPLEXREACTIVESCENARIO Build one complex, multi-crossroad,
%   multi-lane reactive scenario:
%
%     - 2-3 crossroads along one long road, each with its own
%       randomized traffic signal
%     - 2 lanes: ego and background vehicles change lanes when the
%       vehicle ahead in their lane is close and slow, and the other
%       lane is clear
%     - mixed background traffic: car / truck / bus / bicycle, each
%       independently speed-varying, occasionally braking hard
%     - mixed crossing actors at each crossroad: pedestrian / child /
%       cyclist, ~65% crossing safely on red (label 0), ~35% jaywalking
%       on green (label 1, risky)
%     - at ONE randomly chosen crossroad, the ego optionally performs a
%       maneuver instead of continuing straight: right turn, left turn,
%       or a U-turn (each ~15% chance, else straight ~55%)
%     - road length is derived from the scenario duration and a
%       generous top-speed bound, so the ego vehicle can NEVER reach the
%       end of the road before the simulation ends
%
%   NOTE ON TURNS: turning/U-turn motion is modeled with a smooth
%   (cosine-eased) interpolation between the approach point and an exit
%   point near the crossroad, rather than a physically exact steering
%   model. This keeps the geometry simple and robust while still giving
%   visually sensible turning behavior for the dataset.

rng(rngSeed);

duration = 90 + 60 * rand();  % 90-150 s
dt = 0.1;

numLanes  = 2;
laneWidth = 3.5;

% ---------------- IDM parameters for the ego controller ----------------
p.v0            = 10 + 8 * rand();
p.T             = 1.2 + 0.6 * rand();
p.a             = 1.2 + 0.8 * rand();
p.b             = 1.5 + 1.0 * rand();
p.delta         = 4;
p.s0            = 2 + 1.5 * rand();
p.vehicleLength = 4.5;

% ---------------- road, sized so ego can never reach the end ----------------
maxPossibleSpeed = 26; % m/s, generous upper bound across ego + all background actors
roadLength = duration * maxPossibleSpeed * 1.15;

scenario = drivingScenario('SampleTime', dt, 'StopTime', duration);
centers = [0 0 0; roadLength 0 0];
lspec = lanespec(numLanes, 'Width', laneWidth);
road(scenario, centers, 'Lanes', lspec, 'Name', 'ComplexReactiveRoad');

% ---------------- 2-3 crossroads spread across the road ----------------
numCrossroads = randi([2, 3]);
expectedTravel = duration * (p.v0 * 0.5); % conservative, accounts for stops/turns
spanEnd = min(0.85 * expectedTravel, 0.9 * roadLength);
basePositions = linspace(0.18, 0.82, numCrossroads) * spanEnd;
jitter = (rand(1, numCrossroads) - 0.5) * 0.05 * spanEnd;
crossPositions = sort(max(basePositions + jitter, 40));

crossroads = struct('xSignal', {}, 'stopLinePos', {}, 'crosswalkStart', {}, 'crosswalkEnd', {}, 'signal', {});
for c = 1:numCrossroads
    xSignal = crossPositions(c);
    sig.greenDur  = 15 + 10 * rand();
    sig.yellowDur = 2.5 + 1.5 * rand();
    sig.redDur    = 15 + 10 * rand();
    sig.offset    = (sig.greenDur + sig.yellowDur + sig.redDur) * rand();

    crossroads(c).xSignal        = xSignal;
    crossroads(c).stopLinePos     = xSignal - 5;
    crossroads(c).crosswalkStart  = xSignal - 3;
    crossroads(c).crosswalkEnd    = xSignal + 3;
    crossroads(c).signal          = sig;

    nsCenters = [xSignal -60 0; xSignal 0 0; xSignal 60 0];
    road(scenario, nsCenters, 'Lanes', lanespec(2, 'Width', 3.2), 'Name', sprintf('CrossStreet%d', c));
end

% ---------------- ego maneuver: straight / right turn / left turn / U-turn ----------------
maneuverRoll = rand();
if maneuverRoll < 0.55
    egoManeuver = 'straight';
    maneuverCrossroad = 0;
else
    maneuverCrossroad = randi(numCrossroads);
    if maneuverRoll < 0.70
        egoManeuver = 'right_turn';
    elseif maneuverRoll < 0.85
        egoManeuver = 'left_turn';
    else
        egoManeuver = 'u_turn';
    end
end

% ---------------- ego vehicle ----------------
egoV0 = 6 + 4 * rand();
egoLane0 = randi([0, numLanes - 1]);
egoActor = vehicle(scenario, 'ClassID', 1, 'Name', 'ego_reactive_car_following', ...
    'Length', 4.7, 'Width', 1.8, 'Height', 1.4, ...
    'Position', [0, egoLane0 * laneWidth, 0], 'Velocity', [egoV0 0 0]);

% ---------------- background vehicles (mixed types, own lane, may lane-change) ----------------
numVehicles = randi([5, 9]);
typeOptions = {'car', 'car', 'car', 'truck', 'bus', 'bicycle'}; % weighted toward cars
minStartX = 20;
maxStartX = 0.85 * spanEnd + 30;
startXs = sort(minStartX + (maxStartX - minStartX) * rand(numVehicles, 1));

bgActors = [];
bgState = struct('targetV', {}, 'nextChange', {}, 'vType', {}, 'speedMin', {}, 'speedMax', {}, ...
    'brakeUntil', {}, 'lane', {}, 'targetLane', {}, 'laneChangeStartT', {}, 'laneChangeDur', {});
for k = 1:numVehicles
    vType = typeOptions{randi(numel(typeOptions))};
    switch vType
        case 'car'
            len = 4.5; wid = 1.8; ht = 1.4; classID = 1; sMin = 8;  sMax = 22;
        case 'truck'
            len = 8.0; wid = 2.4; ht = 3.0; classID = 2; sMin = 6;  sMax = 16;
        case 'bus'
            len = 10.0; wid = 2.5; ht = 3.2; classID = 2; sMin = 6;  sMax = 14;
        case 'bicycle'
            len = 1.8; wid = 0.6; ht = 1.6; classID = 1; sMin = 3;  sMax = 8;
    end
    v0 = sMin + (sMax - sMin) * rand();
    lane0 = randi([0, numLanes - 1]);
    act = vehicle(scenario, 'ClassID', classID, 'Name', sprintf('actor_%d_independent_%s', k, vType), ...
        'Length', len, 'Width', wid, 'Height', ht, ...
        'Position', [startXs(k), lane0 * laneWidth, 0], 'Velocity', [v0 0 0]);
    bgActors = [bgActors, act]; %#ok<AGROW>
    bgState(k).targetV          = v0;
    bgState(k).nextChange       = 6 + 8 * rand();
    bgState(k).vType            = vType;
    bgState(k).speedMin         = sMin;
    bgState(k).speedMax         = sMax;
    bgState(k).brakeUntil       = -1;
    bgState(k).lane             = lane0;
    bgState(k).targetLane       = lane0;
    bgState(k).laneChangeStartT = -1;
    bgState(k).laneChangeDur    = 3;
end

% ---------------- crossing actors (pedestrian/child/cyclist) at each crossroad ----------------
crossTypeOptions = {'pedestrian', 'pedestrian', 'child', 'cyclist'};
crossingEvents = struct('actorHandle', {}, 'actorType', {}, 'startT', {}, 'endT', {}, ...
    'xCross', {}, 'yStart', {}, 'yEnd', {}, 'label', {});

sampleT = 2:1:(duration - 5);
roadSpanY = numLanes * laneWidth;

evIdx = 0;
for c = 1:numCrossroads
    sig = crossroads(c).signal;
    xSignal = crossroads(c).xSignal;

    states = strings(numel(sampleT), 1);
    for i = 1:numel(sampleT)
        states(i) = string(trafficSignalStateLocal(sampleT(i), sig));
    end
    redT   = sampleT(states == "red");
    greenT = sampleT(states == "green");

    numEventsHere = randi([1, 2]);
    for e = 1:numEventsHere
        evIdx = evIdx + 1;
        actorType = crossTypeOptions{randi(numel(crossTypeOptions))};

        isSafe = rand() < 0.65; % 65% safe (cross on red), 35% risky jaywalk on green
        if isSafe && ~isempty(redT)
            startT = redT(randi(numel(redT))); label = 0;
        elseif ~isempty(greenT)
            startT = greenT(randi(numel(greenT))); label = 1;
        elseif ~isempty(redT)
            startT = redT(randi(numel(redT))); label = 0;
        else
            startT = sampleT(randi(numel(sampleT))); label = 0;
        end

        switch actorType
            case 'pedestrian'
                durP = 3 + 3 * rand(); len = 0.5; wid = 0.5; ht = 1.7;
            case 'child'
                durP = 2 + 2 * rand(); len = 0.4; wid = 0.4; ht = 1.1;
            case 'cyclist'
                durP = 1.5 + 1.5 * rand(); len = 1.8; wid = 0.6; ht = 1.6;
        end
        yStart = -4; yEnd = roadSpanY + 4;

        labelWord = 'safe';
        if label == 1
            labelWord = 'risky';
        end
        act = actor(scenario, 'ClassID', 4, ...
            'Name', sprintf('actor_%d_crossroad%d_%s_%s', evIdx, c, actorType, labelWord), ...
            'Length', len, 'Width', wid, 'Height', ht, ...
            'Position', [xSignal, yStart, 0]);

        crossingEvents(evIdx).actorHandle = act;
        crossingEvents(evIdx).actorType   = actorType;
        crossingEvents(evIdx).startT      = startT;
        crossingEvents(evIdx).endT        = startT + durP;
        crossingEvents(evIdx).xCross      = xSignal;
        crossingEvents(evIdx).yStart      = yStart;
        crossingEvents(evIdx).yEnd        = yEnd;
        crossingEvents(evIdx).label       = label;
    end
end

% ---------------- ground-truth table ----------------
actorLog = struct('name', {}, 'actorType', {}, 'label', {}, 'description', {}, 'startTime', {});

maneuverDesc = 'drives straight through every crossroad';
switch egoManeuver
    case 'right_turn'
        maneuverDesc = sprintf('turns right onto the cross street at crossroad %d', maneuverCrossroad);
    case 'left_turn'
        maneuverDesc = sprintf('turns left onto the cross street at crossroad %d', maneuverCrossroad);
    case 'u_turn'
        maneuverDesc = sprintf('makes a U-turn at crossroad %d', maneuverCrossroad);
end

actorLog(end+1) = struct('name', 'ego_reactive_car_following', 'actorType', 'car', 'label', 0, ...
    'description', sprintf(['Ego vehicle adapts speed to whichever vehicle is ahead of it in its lane ' ...
    '(IDM car-following), changes lanes around slow/close traffic, reacts to %d traffic-signal crossroads ' ...
    'and crossing pedestrians/children/cyclists, and %s.'], numCrossroads, maneuverDesc), 'startTime', 0);

for k = 1:numVehicles
    actorLog(end+1) = struct('name', sprintf('actor_%d_independent_%s', k, bgState(k).vType), ... %#ok<AGROW>
        'actorType', bgState(k).vType, 'label', 0, ...
        'description', sprintf(['Background %s drives at its own independently varying speed, ' ...
        'occasionally brakes suddenly, and changes lanes around slower traffic ahead of it.'], bgState(k).vType), ...
        'startTime', 0);
end

for e = 1:numel(crossingEvents)
    ev = crossingEvents(e);
    if ev.label == 0
        desc = sprintf('%s waits for the signal and crosses safely at the marked crosswalk.', capitalizeFirst(ev.actorType));
    else
        desc = sprintf('%s jaywalks across the crossing while the ego has a green light (risky).', capitalizeFirst(ev.actorType));
    end
    actorLog(end+1) = struct('name', ev.actorHandle.Name, 'actorType', ev.actorType, ... %#ok<AGROW>
        'label', ev.label, 'description', desc, 'startTime', ev.startT);
end

gtTable = struct2table(actorLog);

meta.scenarioID      = scenarioID;
meta.roadType         = 'multi_crossroad_multilane';
meta.duration          = duration;
meta.numEvents          = numVehicles + numel(crossingEvents);
meta.rngSeed             = rngSeed;
meta.egoManeuver         = egoManeuver;
meta.maneuverCrossroad   = maneuverCrossroad;

simInfo.crossroads        = crossroads;
simInfo.crossingEvents    = crossingEvents;
simInfo.idmParams         = p;
simInfo.egoActor          = egoActor;
simInfo.backgroundActors  = bgActors;
simInfo.backgroundState   = bgState;
simInfo.roadLength        = roadLength;
simInfo.numLanes          = numLanes;
simInfo.laneWidth         = laneWidth;
simInfo.egoManeuver       = egoManeuver;
simInfo.maneuverCrossroad = maneuverCrossroad;
simInfo.egoLane0          = egoLane0;

end

% ======================= local helper functions =======================
function state = trafficSignalStateLocal(t, sig)
    cycle = sig.greenDur + sig.yellowDur + sig.redDur;
    tc = mod(t + sig.offset, cycle);
    if tc < sig.greenDur
        state = 'green';
    elseif tc < sig.greenDur + sig.yellowDur
        state = 'yellow';
    else
        state = 'red';
    end
end

function s = capitalizeFirst(s)
    if ~isempty(s)
        s(1) = upper(s(1));
    end
end
