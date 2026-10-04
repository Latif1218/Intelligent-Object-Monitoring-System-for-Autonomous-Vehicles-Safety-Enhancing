%% diagnose_radar_detections.m
%
% Prints the exact size of every detection's Measurement vector for the
% first ~15 simulation steps, plus a couple of other fields, so we can
% see EXACTLY what's inconsistent instead of guessing.

clear; clc;

scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 5);
road(scenario, [0 0 0; 200 0 0], 'Lanes', lanespec(3));

egoVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [10 0 0], ...
    'Length', 4.7, 'Width', 1.8, 'Height', 1.4, 'Name', 'Ego');
smoothTrajectory(egoVehicle, [10 0 0; 190 0 0], 15);

safeCar = vehicle(scenario, 'ClassID', 1, 'Position', [40 3.6 0], ...
    'Length', 4.5, 'Width', 1.8, 'Height', 1.4, 'Name', 'SafeCar');
smoothTrajectory(safeCar, [40 3.6 0; 190 3.6 0], 14);

safePed = actor(scenario, 'ClassID', 4, 'Length', 0.5, 'Width', 0.5, ...
    'Height', 1.7, 'Position', [60 -6 0], 'Name', 'SafePedestrian');
smoothTrajectory(safePed, [60 -6 0; 60 6 0], 1.2);

radarSensor = drivingRadarDataGenerator('SensorIndex', 1, ...
    'UpdateRate', 10, ...
    'MountingLocation', [egoVehicle.Length/2, 0, 0.2], ...
    'RangeLimits', [0 150], ...
    'FieldOfView', [140 5], ...
    'HasElevation', false, ...
    'HasRangeRate', false, ...
    'HasNoise', true, ...
    'HasOcclusion', true);

fprintf('MeasurementParameters/format info (from sensor object):\n');
disp(radarSensor);

step = 0;
while advance(scenario) && step < 15
    step = step + 1;
    time = scenario.SimulationTime;
    tgtPoses = targetPoses(egoVehicle);
    [detections, numDets, isValidTime] = radarSensor(tgtPoses, time);

    fprintf('\n--- Step %d, time=%.2f, numDets=%d, isValidTime=%d ---\n', step, time, numDets, isValidTime);
    for k = 1:numDets
        d = detections{k};
        fprintf('  det %d: size(Measurement)=[%s]  Measurement=[%s]  ObjectClassID=%d  SensorIndex=%d\n', ...
            k, num2str(size(d.Measurement)), num2str(d.Measurement(:)'), ...
            d.ObjectClassID, d.SensorIndex);
    end
end

fprintf('\nDone. Please copy everything above and paste it back.\n');
