%% live_risk_visualization.m
%
% Purpose:
%   Run a driving scenario (ego vehicle + surrounding car/pedestrian/
%   child/cyclist actors), simulate a radar sensor on the ego vehicle,
%   track the surrounding actors, and in REAL TIME classify each tracked
%   actor as SAFE (green box) or RISKY (red box) using the trained
%   classifier from Classification Learner.
%
% Visualization:
%   Figure 1: chasePlot(egoVehicle) - a 3-D chase-camera view of the
%             whole scenario (realistic vehicle/pedestrian meshes).
%   Figure 2: birdsEyePlot - top-down view showing:
%               - green boxes  = actors currently classified SAFE
%               - red boxes    = actors currently classified RISKY
%               - gray boxes   = not enough history yet to classify
%               - yellow dots  = raw radar detections
%               - track markers = confirmed tracker output
%
% REQUIREMENTS:
%   - Automated Driving Toolbox
%   - Sensor Fusion and Tracking Toolbox
%   - trainedRiskClassifier.mat in the current folder, containing a
%     classifier trained WITHOUT the ActorType predictor (radar-only
%     tracking cannot tell a car from a pedestrian by itself), with
%     predictors in this exact order:
%       avg_speed, max_speed, speed_std, max_accel, max_decel,
%       max_lateral_rate, yaw_rate_std, min_dist_to_ego, avg_dist_to_ego,
%       duration
%
% NOTE: MATLAB toolbox function signatures can vary slightly by version.
% If you hit an error, paste the exact error message back and I will
% patch this script precisely rather than guessing.

clear; clc; close all;

%% ---- CONFIG ----
MODEL_FILE = fullfile(pwd, 'trainedRiskClassifier.mat');
MIN_HISTORY_SEC = 1.5;     % need at least this much track history before classifying
HISTORY_WINDOW_SEC = 5;    % how much recent history to keep per track (rolling window)
SIM_STOP_TIME = 20;        % seconds
%% -----------------

if ~isfile(MODEL_FILE)
    error(['Could not find %s.\nExport your ActorType-free model from ' ...
           'Classification Learner and save it as trainedRiskClassifier.mat first.'], MODEL_FILE);
end
modelData = load(MODEL_FILE);
modelFields = fieldnames(modelData);
riskModel = modelData.(modelFields{1});   % grabs whatever variable name you saved it as
fprintf('Loaded classifier: %s\n', modelFields{1});

%% ---- BUILD DEMO SCENARIO ----
scenario = drivingScenario('SampleTime', 0.1, 'StopTime', SIM_STOP_TIME);

road(scenario, [0 0 0; 200 0 0], 'Lanes', lanespec(3));

% Ego vehicle
egoVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [10 0 0], ...
    'Length', 4.7, 'Width', 1.8, 'Height', 1.4, 'Name', 'Ego');
egoWP = [10 0 0; 190 0 0];
egoSpeed = 15;   % m/s
smoothTrajectory(egoVehicle, egoWP, egoSpeed);

% Actor 1: SAFE - car ahead maintaining steady speed/lane
safeCar = vehicle(scenario, 'ClassID', 1, 'Position', [40 3.6 0], ...
    'Length', 4.5, 'Width', 1.8, 'Height', 1.4, 'Name', 'SafeCar');
smoothTrajectory(safeCar, [40 3.6 0; 190 3.6 0], 14);

% Actor 2: RISKY - aggressive car doing a sudden lane change at high speed
riskyCar = vehicle(scenario, 'ClassID', 1, 'Position', [30 -3.6 0], ...
    'Length', 4.5, 'Width', 1.8, 'Height', 1.4, 'Name', 'RiskyCar');
smoothTrajectory(riskyCar, [30 -3.6 0; 70 -3.6 0; 100 0 0; 150 0 0], 22);

% Actor 3: SAFE - pedestrian crossing only at a marked crossing, moving slowly
safePed = actor(scenario, 'ClassID', 4, 'Length', 0.5, 'Width', 0.5, ...
    'Height', 1.7, 'Position', [60 -6 0], 'Name', 'SafePedestrian');
smoothTrajectory(safePed, [60 -6 0; 60 6 0], 1.2);

% Actor 4: RISKY - child suddenly running into the road chasing a ball
childActor = actor(scenario, 'ClassID', 4, 'Length', 0.4, 'Width', 0.4, ...
    'Height', 1.1, 'Position', [100 -8 0], 'Name', 'ChildChasingBall');
smoothTrajectory(childActor, [100 -8 0; 100 -1 0; 100 3 0], 3);

%% ---- SENSOR: RADAR ON EGO VEHICLE ----
radarSensor = drivingRadarDataGenerator('SensorIndex', 1, ...
    'UpdateRate', 10, ...
    'MountingLocation', [egoVehicle.Length/2, 0, 0.2], ...
    'RangeLimits', [0 150], ...
    'FieldOfView', [140 5], ...
    'HasElevation', false, ...
    'HasRangeRate', false, ...
    'HasNoise', true, ...
    'HasOcclusion', true);

%% ---- TRACKER ----
tracker = multiObjectTracker( ...
    'FilterInitializationFcn', @initRiskTrackFilter, ...
    'AssignmentThreshold', 30, ...
    'ConfirmationThreshold', [2 3], ...
    'DeletionThreshold', [5 5]);

%% ---- VISUALIZATION SETUP ----
% Figure 1: 3-D chase camera view
f1 = figure('Name', '3D Chase View', 'Position', [50 400 700 500]);
chasePlot(egoVehicle, 'Centerline', 'on', 'Parent', gca);

% Figure 2: Bird's-eye plot with risk-colored boxes
f2 = figure('Name', 'Risk Classification - Bird''s Eye View', 'Position', [800 400 700 500]);
bep = birdsEyePlot('XLimits', [-10 160], 'YLimits', [-20 20]);

safePlotter    = outlinePlotter(bep, 'Tag', 'SafeActors');
riskyPlotter   = outlinePlotter(bep, 'Tag', 'RiskyActors');
pendingPlotter = outlinePlotter(bep, 'Tag', 'PendingActors');
detPlotter     = detectionPlotter(bep, 'DisplayName', 'Radar detections', ...
                     'MarkerEdgeColor', [1 0.85 0], 'Marker', 'o');
egoOutline     = outlinePlotter(bep, 'Tag', 'Ego');

% simple manual legend as text (outlinePlotter doesn't support DisplayName)
axBep = gca;
text(axBep, 0.02, 0.98, '\color{green}Safe   \color{red}Risky   \color[rgb]{0.5,0.5,0.5}Classifying...   \color[rgb]{1,0.85,0}Radar dets   \color{blue}Ego', ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'FontSize', 9);

%% ---- PER-TRACK HISTORY BUFFER ----
% trackHistory(trackID) = struct with arrays: Time, X, Y, Vx, Vy
trackHistory = containers.Map('KeyType', 'double', 'ValueType', 'any');
trackLabel   = containers.Map('KeyType', 'double', 'ValueType', 'double'); % 0=safe,1=risky,-1=pending

%% ---- MAIN SIMULATION LOOP ----
restart(scenario);
warnState = warning('off', 'all');   % suppress harmless sensor/geometry warnings during simulation
cleanupWarn = onCleanup(@() warning(warnState));  % restore warnings when script ends/errors

while advance(scenario)
    time = scenario.SimulationTime;

    % --- Sensor: get radar detections of all OTHER actors, in ego frame ---
    tgtPoses = targetPoses(egoVehicle);
    [detections, numDets, isValidTime] = radarSensor(tgtPoses, time);

    % The radar always reports 3-element measurements [x,y,z] (z is the
    % fixed mounting height). Our tracker/filter only tracks x,y, so
    % strip z here to keep every detection a consistent 2-element size.
    for di = 1:numDets
        detections{di}.Measurement = detections{di}.Measurement(1:2);
        detections{di}.MeasurementNoise = detections{di}.MeasurementNoise(1:2, 1:2);
    end

    % --- Tracker update ---
    confirmedTracks = objectTrack.empty;
    if isValidTime && numDets > 0
        confirmedTracks = updateTracks(tracker, detections, time);
    elseif isLocked(tracker)
        confirmedTracks = updateTracks(tracker, {}, time);
    end

    % --- Update history buffer + classify each confirmed track ---
    activeIDs = [];
    for k = 1:numel(confirmedTracks)
        trk = confirmedTracks(k);
        tid = trk.TrackID;
        activeIDs(end+1) = tid; %#ok<AGROW>

        pos = trk.State([1 3]);   % [x; y] for a constant-velocity [x vx y vy] state
        vel = trk.State([2 4]);   % [vx; vy]

        if ~isKey(trackHistory, tid)
            trackHistory(tid) = struct('Time', [], 'X', [], 'Y', [], 'Vx', [], 'Vy', []);
            trackLabel(tid) = -1; % pending
        end
        h = trackHistory(tid);
        h.Time(end+1) = time;
        h.X(end+1) = pos(1);
        h.Y(end+1) = pos(2);
        h.Vx(end+1) = vel(1);
        h.Vy(end+1) = vel(2);

        % keep only the last HISTORY_WINDOW_SEC seconds
        keep = h.Time >= (time - HISTORY_WINDOW_SEC);
        h.Time = h.Time(keep); h.X = h.X(keep); h.Y = h.Y(keep);
        h.Vx = h.Vx(keep); h.Vy = h.Vy(keep);
        trackHistory(tid) = h;

        duration = h.Time(end) - h.Time(1);
        if duration >= MIN_HISTORY_SEC && numel(h.Time) >= 4
            feats = computeLiveFeatures(h);
            try
                predLabel = predict(riskModel, feats);
                trackLabel(tid) = predLabel;
            catch
                trackLabel(tid) = -1; % keep pending if predict() fails
            end
        end
    end

    % --- Split active tracks into safe / risky / pending for plotting ---
    safePos = zeros(0,2); safeDim = zeros(0,2); safeYaw = zeros(0,1);
    riskyPos = zeros(0,2); riskyDim = zeros(0,2); riskyYaw = zeros(0,1);
    pendPos = zeros(0,2); pendDim = zeros(0,2); pendYaw = zeros(0,1);

    for k = 1:numel(confirmedTracks)
        trk = confirmedTracks(k);
        tid = trk.TrackID;
        pos = [trk.State(1), trk.State(3)];
        vel = [trk.State(2), trk.State(4)];
        yaw = atan2d(vel(2), vel(1));
        dims = [2.0 1.8];   % generic [length width] box size for visualization

        lbl = trackLabel(tid);
        if lbl == 0
            safePos = [safePos; pos]; safeDim = [safeDim; dims]; safeYaw = [safeYaw; yaw]; %#ok<AGROW>
        elseif lbl == 1
            riskyPos = [riskyPos; pos]; riskyDim = [riskyDim; dims]; riskyYaw = [riskyYaw; yaw]; %#ok<AGROW>
        else
            pendPos = [pendPos; pos]; pendDim = [pendDim; dims]; pendYaw = [pendYaw; yaw]; %#ok<AGROW>
        end
    end

    plotOutline(safePlotter, safePos, safeYaw, safeDim(:,1), safeDim(:,2), 'Color', repmat([0 1 0], size(safePos,1), 1));
    plotOutline(riskyPlotter, riskyPos, riskyYaw, riskyDim(:,1), riskyDim(:,2), 'Color', repmat([1 0 0], size(riskyPos,1), 1));
    plotOutline(pendingPlotter, pendPos, pendYaw, pendDim(:,1), pendDim(:,2), 'Color', repmat([0.5 0.5 0.5], size(pendPos,1), 1));

    % raw radar detections
    if numDets > 0
        detPosCell = cellfun(@(d) d.Measurement(1:2)', detections, 'UniformOutput', false);
        detPos = vertcat(detPosCell{:});   % always Nx2 regardless of cell array orientation
        plotDetection(detPlotter, detPos);
    else
        plotDetection(detPlotter, zeros(0,2));
    end

    % ego box (blue-ish, always shown)
    plotOutline(egoOutline, [0 0], 0, egoVehicle.Length, egoVehicle.Width, 'Color', [0 0 1]);

    % refresh both figures
    figure(f1); chasePlot(egoVehicle, 'Centerline', 'on', 'Parent', gca);
    drawnow limitrate;
end

fprintf('\nSimulation finished.\n');

%% ---- LOCAL FUNCTIONS ----

function feats = computeLiveFeatures(h)
    % Mirrors the feature definitions used in build_feature_dataset.m,
    % computed from a rolling history buffer instead of a full logged
    % trajectory. Order MUST match the predictors the model was trained
    % on (ActorType excluded).
    speed = sqrt(h.Vx.^2 + h.Vy.^2);
    dt = diff(h.Time); dt(dt == 0) = NaN;

    dSpeed = diff(speed);
    accelSeries = dSpeed ./ dt;
    max_accel = max(accelSeries, [], 'omitnan');
    max_decel = min(accelSeries, [], 'omitnan');
    if isempty(max_accel) || isnan(max_accel), max_accel = 0; end
    if isempty(max_decel) || isnan(max_decel), max_decel = 0; end

    dY = diff(h.Y);
    lateralRateSeries = dY ./ dt;
    max_lateral_rate = max(abs(lateralRateSeries), [], 'omitnan');
    if isempty(max_lateral_rate) || isnan(max_lateral_rate), max_lateral_rate = 0; end

    heading = atan2(h.Vy, h.Vx);
    headingUnwrapped = unwrap(heading);
    dHeading = diff(headingUnwrapped);
    yawRateSeries = dHeading ./ dt;
    yaw_rate_std = std(yawRateSeries, 'omitnan');
    if isnan(yaw_rate_std), yaw_rate_std = 0; end

    distToEgo = sqrt(h.X.^2 + h.Y.^2);  % track positions are ego-relative

    feats = table( ...
        mean(speed, 'omitnan'), max(speed, [], 'omitnan'), std(speed, 'omitnan'), ...
        max_accel, max_decel, max_lateral_rate, yaw_rate_std, ...
        min(distToEgo, [], 'omitnan'), mean(distToEgo, 'omitnan'), ...
        h.Time(end) - h.Time(1), ...
        'VariableNames', {'avg_speed','max_speed','speed_std','max_accel','max_decel', ...
                           'max_lateral_rate','yaw_rate_std','min_dist_to_ego', ...
                           'avg_dist_to_ego','duration'});
end

function filter = initRiskTrackFilter(detection)
    % Constant-velocity Kalman filter: state = [x vx y vy]
    % measurement = [x; y] (2 elements - HasRangeRate is disabled on the
    % radar so every detection has this same fixed size)
    meas = detection.Measurement;
    initState = [meas(1); 0; meas(2); 0];
    initCov = diag([10 10 10 10]);
    measModel = [1 0 0 0; 0 0 1 0];
    filter = trackingKF('MotionModel', '2D Constant Velocity', ...
        'State', initState, 'StateCovariance', initCov, ...
        'MeasurementModel', measModel, ...
        'ProcessNoise', diag([1 1]));
end