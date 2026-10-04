function [scenario, gtTable, meta, simInfo] = generateReactiveScenario(scenarioID, rngSeed)
%GENERATEREACTIVESCENARIO Build one reactive car-following scenario.
%
%   Unlike generateScenario.m (fixed complexity-library waypoints/speed
%   profiles), this scenario is CLOSED-LOOP / reactive:
%
%     - The ego vehicle's speed is NOT pre-scripted. It reacts, step by
%       step, to whichever vehicle is currently closest ahead of it
%       (Intelligent Driver Model car-following): it speeds up/slows
%       down based on the gap and relative speed to that vehicle.
%     - There is a crossroad with a traffic signal (green/yellow/red,
%       randomized timing per scenario). Ego stops at the stop line on
%       red/yellow and proceeds on green.
%     - Pedestrians cross near the crossroad at 1-3 random time windows.
%       Ego stops before the crosswalk while a pedestrian is crossing.
%     - Background vehicles move independently, each at its own
%       randomly varying speed. They do NOT react to ego, the signal,
%       or the pedestrians.
%
%   The actual per-timestep control loop lives in
%   runAndRecordReactiveScenario.m (mirrors runAndRecordScenario.m's
%   job, but also drives the ego/background/pedestrian motion since
%   none of them use a pre-set trajectory() here).
%
%   Reuses createRoadNetwork('crossroad') as-is, so this plugs straight
%   into your existing dataset_100_scenarios pipeline.
%
%   Returns:
%     scenario - drivingScenario object (actors created, no trajectories)
%     gtTable  - same shape as generateScenario.m's gtTable
%                (name, actorType, label, description, startTime)
%     meta     - same shape as generateScenario.m's meta struct
%     simInfo  - extra state runAndRecordReactiveScenario.m needs to
%                drive the closed-loop simulation

rng(rngSeed);

[scenario, roadInfo] = createRoadNetwork('crossroad');

% Tune the run length to fit this road comfortably (crossroad road is
% ~400 m long; 60-90 s keeps vehicles within a sensible range at
% intersection speeds).
duration = 60 + 30 * rand();
scenario.StopTime = duration;

roadStart = roadInfo.centers(1, :);
roadEnd   = roadInfo.centers(end, :);
roadLen   = roadEnd(1) - roadStart(1);
xSignal   = roadInfo.intersectionPoint(1);

road.length          = roadLen;
road.signalPos        = xSignal;
road.stopLinePos       = xSignal - 5;
road.crosswalkStart    = xSignal - 3;
road.crosswalkEnd      = xSignal + 3;

% ---------------- traffic signal (randomized per scenario) ----------------
sig.greenDur  = 15 + 10 * rand();
sig.yellowDur = 2.5 + 1.5 * rand();
sig.redDur    = 15 + 10 * rand();
sig.offset    = (sig.greenDur + sig.yellowDur + sig.redDur) * rand();

% ---------------- pedestrian crossing events ----------------
numPed = randi([1, 3]);
pedEvents = zeros(numPed, 2);
for k = 1:numPed
    startT = 5 + max(duration - 15, 1) * rand();
    durP   = 3 + 3 * rand();
    pedEvents(k, :) = [startT, startT + durP];
end
pedEvents = sortrows(pedEvents, 1);

% ---------------- IDM parameters for the ego controller ----------------
p.v0            = 10 + 6 * rand();     % ego desired free-flow speed (m/s)
p.T             = 1.2 + 0.6 * rand();  % desired time headway (s)
p.a             = 1.2 + 0.8 * rand();  % max acceleration (m/s^2)
p.b             = 1.5 + 1.0 * rand();  % comfortable braking (m/s^2)
p.delta         = 4;
p.s0            = 2 + 1.5 * rand();    % minimum gap (m)
p.vehicleLength = 4.5;
p.bgSpeedMin    = 5;                            % background vehicles' own speed range
p.bgSpeedMax    = roadInfo.speedLimit + 4;

% ---------------- ego vehicle (manually/reactively controlled) ----------------
egoV0 = 6 + 4 * rand();
egoActor = vehicle(scenario, 'ClassID', 1, 'Name', 'ego_reactive_car_following', ...
    'Length', 4.7, 'Width', 1.8, 'Height', 1.4, ...
    'Position', roadStart, 'Velocity', [egoV0 0 0]);

% ---------------- background vehicles (independent, manually controlled) ----------------
numVehicles = randi([3, 6]);
minStartX = roadStart(1) + 20;
maxStartX = roadStart(1) + 0.9 * roadLen;
startXs = sort(minStartX + (maxStartX - minStartX) * rand(numVehicles, 1));

bgActors = [];
bgState = struct('targetV', {}, 'nextChange', {});
for k = 1:numVehicles
    v0 = p.bgSpeedMin + (p.bgSpeedMax - p.bgSpeedMin) * rand();
    act = vehicle(scenario, 'ClassID', 1, 'Name', sprintf('actor_%d_independent_traffic', k), ...
        'Length', 4.5, 'Width', 1.8, 'Height', 1.4, ...
        'Position', [startXs(k) 0 0], 'Velocity', [v0 0 0]);
    bgActors = [bgActors, act]; %#ok<AGROW>
    bgState(k).targetV    = v0;
    bgState(k).nextChange = 8 + 7 * rand();
end

% ---------------- pedestrian actors, one per crossing event ----------------
pedActors = struct('actorHandle', {}, 'startT', {}, 'endT', {}, ...
    'xCross', {}, 'yStart', {}, 'yEnd', {});
for k = 1:numPed
    yStart = -4; yEnd = 4;
    act = actor(scenario, 'ClassID', 4, 'Name', sprintf('actor_%d_signal_crosswalk_pedestrian', k), ...
        'Length', 0.5, 'Width', 0.5, 'Height', 1.7, ...
        'Position', [xSignal yStart 0]);
    pedActors(k).actorHandle = act; %#ok<AGROW>
    pedActors(k).startT = pedEvents(k, 1);
    pedActors(k).endT   = pedEvents(k, 2);
    pedActors(k).xCross = xSignal;
    pedActors(k).yStart = yStart;
    pedActors(k).yEnd   = yEnd;
end

% ---------------- ground-truth table (same shape as generateScenario.m) ----------------
actorLog = struct('name', {}, 'actorType', {}, 'label', {}, 'description', {}, 'startTime', {});

actorLog(end+1) = struct('name', 'ego_reactive_car_following', 'actorType', 'car', 'label', 0, ...
    'description', ['Ego vehicle adapts speed in real time to whichever vehicle is ' ...
    'currently ahead of it (IDM car-following), stops at red/yellow traffic signals, ' ...
    'and yields to crossing pedestrians.'], 'startTime', 0);

for k = 1:numVehicles
    actorLog(end+1) = struct('name', sprintf('actor_%d_independent_traffic', k), ... %#ok<AGROW>
        'actorType', 'car', 'label', 0, ...
        'description', 'Background vehicle drives at its own independently varying speed.', ...
        'startTime', 0);
end

for k = 1:numPed
    actorLog(end+1) = struct('name', sprintf('actor_%d_signal_crosswalk_pedestrian', k), ... %#ok<AGROW>
        'actorType', 'pedestrian', 'label', 0, ...
        'description', 'Pedestrian crosses at the marked crosswalk near the signal.', ...
        'startTime', pedEvents(k, 1));
end

gtTable = struct2table(actorLog);

meta.scenarioID = scenarioID;
meta.roadType   = 'crossroad';
meta.duration   = duration;
meta.numEvents  = numVehicles + numPed;
meta.rngSeed    = rngSeed;

simInfo.road             = road;
simInfo.signal           = sig;
simInfo.pedEvents        = pedEvents;
simInfo.idmParams        = p;
simInfo.egoActor         = egoActor;
simInfo.backgroundActors = bgActors;
simInfo.backgroundState  = bgState;
simInfo.pedestrianActors = pedActors;

end
