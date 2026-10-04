function trackLog = demoSensorTracking(scenarioMatFile)
%DEMOSENSORTRACKING Loads one saved scenario, simulates radar detections
% from the ego vehicle's point of view, tracks the surrounding actors
% with a multiObjectTracker, and logs the tracked (noisy, estimated)
% position/velocity of every track over time.
%
% Usage:
%   trackLog = demoSensorTracking('dataset_100_scenarios/scenario_005.mat');
%
% This is Step 4.1-4.3 of the project: simulate sensor data, then track.
% Later, features will be extracted from trackLog (Step 4.4) and run
% through trainedRiskClassifier (Step 4.5) to measure robustness.

data = load(scenarioMatFile);
scenario = data.scenario;
restart(scenario);

allNames = cell(1, numel(scenario.Actors));
for i = 1:numel(scenario.Actors)
    nm = scenario.Actors(i).Name;
    if isempty(nm)
        nm = sprintf('(unnamed_actor_%d)', i);
    elseif isstring(nm)
        nm = char(nm);
    elseif ~ischar(nm)
        nm = char(string(nm));
    end
    allNames{i} = nm;
end
fprintf('Actors found in this scenario: %s\n', strjoin(allNames, ', '));

egoIdx = find(strcmpi(strtrim(allNames), 'Ego'), 1);
if isempty(egoIdx)
    warning('No actor named "Ego" found by exact match. Using Actors(1) ("%s") as ego instead.', allNames{1});
    egoIdx = 1;
end
egoVehicle = scenario.Actors(egoIdx);

% --- Radar sensor mounted on the front of the ego vehicle ---
radarSensor = drivingRadarDataGenerator( ...
    'SensorIndex', 1, ...
    'UpdateRate', 10, ...
    'MountingLocation', [egoVehicle.Length/2, 0, 0], ...
    'RangeLimits', [0 150], ...
    'FieldOfView', [140 5], ...
    'HasNoise', true);

% --- Multi-object tracker using a standard constant-velocity EKF ---
tracker = multiObjectTracker( ...
    'FilterInitializationFcn', @initcvekf, ...
    'AssignmentThreshold', 35, ...
    'ConfirmationThreshold', [2 3], ...
    'DeletionThreshold', 5);

logRows = {};

while advance(scenario)
    time = scenario.SimulationTime;
    targets = targetPoses(egoVehicle); % other actors' poses, relative to ego

    [detections, numDets, isValidTime] = radarSensor(targets, time);

    if isValidTime && (numDets > 0 || isLocked(tracker))
        confirmedTracks = tracker(detections, time);
        for ii = 1:numel(confirmedTracks)
            trk = confirmedTracks(ii);
            % constvel/initcvekf state layout: [x vx y vy z vz]
            pos = trk.State([1 3 5])';
            vel = trk.State([2 4 6])';
            logRows(end+1, :) = {time, trk.TrackID, pos(1), pos(2), pos(3), ...
                vel(1), vel(2), vel(3)}; %#ok<AGROW>
        end
    end
end

trackLog = cell2table(logRows, 'VariableNames', ...
    {'Time', 'TrackID', 'X', 'Y', 'Z', 'Vx', 'Vy', 'Vz'});

fprintf('Tracking complete. %d track updates logged across %d unique tracks.\n', ...
    height(trackLog), numel(unique(trackLog.TrackID)));
end