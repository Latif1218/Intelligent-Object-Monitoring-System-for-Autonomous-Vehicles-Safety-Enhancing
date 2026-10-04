function [scenario, gtTable, meta] = generateScenario(scenarioID, rngSeed)
%GENERATESCENARIO Build one realistic ~120s driving scenario containing
% 5-6 mixed real-life complexity events (safe + risky) with multiple
% actor types: car, truck, bus, bicycle, pedestrian, child.

rng(rngSeed);

roadTypes = {'highway', 'rural', 'village', 'mixed', 'crossroad', 'rail_crossing'};
roadType = roadTypes{randi(numel(roadTypes))};

[scenario, roadInfo] = createRoadNetwork(roadType);

lib = complexityLibrary();

% Keep only events that make sense on this road type
validIdx = find(cellfun(@(rt) any(strcmp(roadType, rt)), {lib.roadTypes}));
if any(strcmp(roadType, {'mixed', 'crossroad', 'rail_crossing'})) || isempty(validIdx)
    validIdx = 1:numel(lib); % these road types allow any event (plus their own special ones)
end

numEvents = randi([5 6]);
numEvents = min(numEvents, numel(validIdx));
chosenIdx = validIdx(randperm(numel(validIdx), numEvents));
chosenEvents = lib(chosenIdx);

% --- Ego vehicle: the autonomous vehicle driving through the whole route ---
ego = vehicle(scenario, 'ClassID', 1, 'Name', 'Ego', ...
    'Length', 4.7, 'Width', 1.8, 'Height', 1.4);
trajectory(ego, roadInfo.centers, 15); % 15 m/s baseline cruising speed

% --- Train actor, only for rail_crossing scenarios ---
if strcmp(roadType, 'rail_crossing')
    cp = roadInfo.crossingPoint;
    trainWaypoints = [cp + [0 -150 0]; cp; cp + [0 150 0]];
    trainSpeed = [25 25 25]; % m/s, fast-moving train
    trainAct = vehicle(scenario, 'ClassID', 2, 'Name', 'Train', ...
        'Length', 60, 'Width', 3, 'Height', 4.5);
    trajectory(trainAct, trainWaypoints, trainSpeed);
end

% --- Background actors, one per chosen complexity event ---
actorLog = struct('name', {}, 'actorType', {}, 'label', {}, ...
    'description', {}, 'startTime', {});

eventTimes = sort(randperm(100, numEvents)); % spread events across the 120s run

for k = 1:numEvents
    ev = chosenEvents(k);
    actorName = sprintf('actor_%d_%s', k, ev.name);

    switch ev.actorType
        case 'car'
            classID = 1; len = 4.5; wid = 1.8; ht = 1.4;
        case 'truck'
            classID = 2; len = 8.0; wid = 2.4; ht = 3.0;
        case 'bus'
            classID = 2; len = 10.0; wid = 2.5; ht = 3.2;
        case 'bicycle'
            classID = 3; len = 1.8; wid = 0.6; ht = 1.6;
        case 'child'
            classID = 4; len = 0.4; wid = 0.4; ht = 1.1;
        case 'pedestrian'
            classID = 4; len = 0.5; wid = 0.5; ht = 1.7;
    end

    if classID == 1 || classID == 2
        % Car or Truck/Bus -> vehicle() only accepts ClassID 1 or 2
        act = vehicle(scenario, 'ClassID', classID, 'Name', actorName, ...
            'Length', len, 'Width', wid, 'Height', ht);
    else
        % Bicycle (3) or Pedestrian/Child (4) -> use actor()
        act = actor(scenario, 'ClassID', classID, 'Name', actorName, ...
            'Length', len, 'Width', wid, 'Height', ht);
    end

    % Place the maneuver: near the intersection/crossing point for those
    % special event types, otherwise at a random point along the road.
    intersectionEvents = {'running_red_light', 'failure_to_yield', 'safe_intersection_stop', ...
        'pedestrian_jaywalk_intersection', 'safe_pedestrian_crosswalk_intersection'};
    railEvents = {'railway_dash_crossing', 'safe_railway_wait', 'pedestrian_railway_dash'};

    if any(strcmp(ev.name, intersectionEvents)) && isfield(roadInfo, 'intersectionPoint')
        basePos = roadInfo.intersectionPoint + [(rand()-0.5)*10, (rand()-0.5)*10, 0];
    elseif any(strcmp(ev.name, railEvents)) && isfield(roadInfo, 'crossingPoint')
        basePos = roadInfo.crossingPoint + [(rand()-0.5)*10, 0, 0];
    else
        segFrac = rand() * 0.8 + 0.1;
        basePos = roadInfo.centers(1,:) + segFrac * (roadInfo.centers(end,:) - roadInfo.centers(1,:));
    end
    lateralOffset = (rand() - 0.5) * 6; % meters

    wp = generateManeuverWaypoints(basePos, lateralOffset, ev.name);
    spd = generateSpeedProfile(ev.name, ev.label);

    trajectory(act, wp, spd);

    actorLog(end+1) = struct('name', actorName, 'actorType', ev.actorType, ... %#ok<AGROW>
        'label', ev.label, 'description', ev.description, 'startTime', eventTimes(k));
end

gtTable = struct2table(actorLog);

meta.scenarioID = scenarioID;
meta.roadType = roadType;
meta.duration = scenario.StopTime;
meta.numEvents = numEvents;
meta.rngSeed = rngSeed;

end

% ---------------------------------------------------------------------
function wp = generateManeuverWaypoints(basePos, lateralOffset, eventName)
% Short set of waypoints describing the maneuver, relative to basePos.
switch eventName
    case 'aggressive_lane_change'
        wp = [basePos; basePos+[10 0 0]; basePos+[20 lateralOffset 0]; basePos+[40 lateralOffset 0]];
    case 'sudden_braking'
        wp = [basePos; basePos+[15 0 0]; basePos+[20 0 0]; basePos+[22 0 0]];
    case 'tailgating'
        wp = [basePos; basePos+[10 0 0]; basePos+[30 0 0]; basePos+[50 0 0]];
    case 'unsafe_overtake'
        wp = [basePos; basePos+[10 lateralOffset 0]; basePos+[30 lateralOffset 0]; basePos+[45 0 0]];
    case 'speeding_zigzag'
        wp = [basePos; basePos+[5 1.5 0]; basePos+[10 -1.5 0]; basePos+[15 1.5 0]; basePos+[20 0 0]];
    case {'child_chasing_ball', 'jaywalking_pedestrian', 'animal_or_obstacle_crossing', 'safe_crosswalk_pedestrian'}
        wp = [basePos+[0 -3 0]; basePos+[0 0 0]; basePos+[0 3 0]];
    case 'bus_sudden_stop'
        wp = [basePos; basePos+[8 0 0]; basePos+[9 0 0]];
    case 'safe_bus_stop'
        wp = [basePos; basePos+[15 0 0]; basePos+[25 0 0]; basePos+[26 0 0]];
    case 'safe_lane_change'
        wp = [basePos; basePos+[15 0 0]; basePos+[30 lateralOffset*0.5 0]; basePos+[50 lateralOffset*0.5 0]];
    case 'safe_following'
        wp = [basePos; basePos+[20 0 0]; basePos+[40 0 0]; basePos+[60 0 0]];
    case 'safe_cyclist'
        wp = [basePos; basePos+[15 0.2 0]; basePos+[30 0 0]; basePos+[45 0.2 0]];
    case 'truck_merging'
        wp = [basePos; basePos+[10 lateralOffset 0]; basePos+[30 lateralOffset 0]];
    case 'running_red_light'
        wp = [basePos; basePos+[15 0 0]; basePos+[30 0 0]];
    case 'failure_to_yield'
        wp = [basePos; basePos+[10 lateralOffset 0]; basePos+[20 lateralOffset*1.5 0]];
    case 'safe_intersection_stop'
        wp = [basePos; basePos+[8 0 0]; basePos+[9 0 0]; basePos+[20 0 0]];
    case {'pedestrian_jaywalk_intersection', 'safe_pedestrian_crosswalk_intersection'}
        wp = [basePos+[0 -4 0]; basePos+[0 0 0]; basePos+[0 4 0]];
    case 'railway_dash_crossing'
        wp = [basePos; basePos+[6 0 0]; basePos+[12 0 0]];
    case 'safe_railway_wait'
        wp = [basePos; basePos+[5 0 0]; basePos+[5.5 0 0]];
    case 'pedestrian_railway_dash'
        wp = [basePos+[0 -3 0]; basePos+[0 0 0]; basePos+[0 3 0]];
    otherwise
        wp = [basePos; basePos+[20 0 0]];
end
end

% ---------------------------------------------------------------------
function spd = generateSpeedProfile(eventName, label)
% Returns a speed vector (m/s) matching the number of waypoints above.
switch eventName
    case 'aggressive_lane_change'
        spd = [25 28 30 30];
    case 'sudden_braking'
        spd = [20 20 5 2];
    case 'tailgating'
        spd = [18 18 18 18];
    case 'unsafe_overtake'
        spd = [15 22 22 15];
    case 'speeding_zigzag'
        spd = [6 7 7 6 5];
    case {'child_chasing_ball', 'jaywalking_pedestrian', 'animal_or_obstacle_crossing'}
        spd = [2.5 3 2.5];
    case 'safe_crosswalk_pedestrian'
        spd = [1.2 1.2 1.2];
    case 'bus_sudden_stop'
        spd = [10 8 0];
    case 'safe_bus_stop'
        spd = [10 8 4 0];
    case 'safe_lane_change'
        spd = [15 15 15 15];
    case 'safe_following'
        spd = [15 15 15 15];
    case 'safe_cyclist'
        spd = [5 5 5 5];
    case 'truck_merging'
        spd = [12 14 15];
    case 'running_red_light'
        spd = [18 20 20];
    case 'failure_to_yield'
        spd = [10 12 12];
    case 'safe_intersection_stop'
        spd = [10 3 0 8];
    case 'pedestrian_jaywalk_intersection'
        spd = [3 3.5 3];
    case 'safe_pedestrian_crosswalk_intersection'
        spd = [1.2 1.2 1.2];
    case 'railway_dash_crossing'
        spd = [15 20 20];
    case 'safe_railway_wait'
        spd = [10 4 0];
    case 'pedestrian_railway_dash'
        spd = [3 4 3];
    otherwise
        if label == 1
            spd = 20;
        else
            spd = 12;
        end
end
end