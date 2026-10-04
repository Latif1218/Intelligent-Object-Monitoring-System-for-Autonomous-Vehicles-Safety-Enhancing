function [trackedFeatures, gtFeatures] = buildTrackedFeatureDataset(scenarioMatFile, labelsCsvFile)
%BUILDTRACKEDFEATUREDATASET Runs sensor+tracking on one scenario, matches
% tracks to ground-truth actors, extracts the same feature set from both
% the tracked (noisy) and ground-truth trajectories, and joins the
% ground-truth safe/risky label for each matched actor.
%
% Usage:
%   [trackedFeatures, gtFeatures] = buildTrackedFeatureDataset( ...
%       'dataset_100_scenarios/scenario_005.mat', ...
%       'dataset_100_scenarios/scenario_005_labels.csv');

% --- Sensor + tracking (noisy) ---
trackLog = demoSensorTracking(scenarioMatFile);

% --- Fresh ground-truth replay (same file, separate copy) ---
data2 = load(scenarioMatFile);
gtLog = getGroundTruthActorLog(data2.scenario);

if isfield(data2, 'meta') && isfield(data2.meta, 'scenarioID')
    scenarioID = data2.meta.scenarioID;
else
    scenarioID = NaN;
end

% --- Find ego's own (precisely known) trajectory, used for distance-to-ego ---
actorNamesAll = unique(gtLog.ActorName, 'stable');
egoNameIdx = find(strcmpi(actorNamesAll, 'Ego'), 1);
if isempty(egoNameIdx)
    egoNameIdx = find(contains(lower(actorNamesAll), 'ego'), 1);
end
if isempty(egoNameIdx)
    egoNameIdx = 1;
end
egoName = actorNamesAll{egoNameIdx};
egoTraj = gtLog(strcmp(gtLog.ActorName, egoName), :);
egoTraj.YawUnwrapped = unwrap(deg2rad(egoTraj.Yaw));

% --- Convert trackLog from ego-relative (sensor/tracker) coordinates into
%     world coordinates, using ego's own known position+heading. Without
%     this, tracked positions cannot be meaningfully compared to
%     ground-truth (world-frame) actor positions. ---
egoX_at_track = interp1(egoTraj.Time, egoTraj.X, trackLog.Time, 'linear', 'extrap');
egoY_at_track = interp1(egoTraj.Time, egoTraj.Y, trackLog.Time, 'linear', 'extrap');
egoYaw_at_track = interp1(egoTraj.Time, egoTraj.YawUnwrapped, trackLog.Time, 'linear', 'extrap');

relX = trackLog.X;
relY = trackLog.Y;
trackLog.X = egoX_at_track + relX .* cos(egoYaw_at_track) - relY .* sin(egoYaw_at_track);
trackLog.Y = egoY_at_track + relX .* sin(egoYaw_at_track) + relY .* cos(egoYaw_at_track);
% Vx/Vy magnitude (speed) is rotation-invariant, so no change needed there.

% --- Match each track to the actor it's actually tracking ---
matchTable = matchTracksToActors(trackLog, gtLog);

labelsTable = table();
if nargin >= 2 && isfile(labelsCsvFile)
    labelsTable = readtable(labelsCsvFile);
end

trackedRows = {};
gtRows = {};

uniqueActors = unique(matchTable.MatchedActor);

for a = 1:numel(uniqueActors)
    aName = uniqueActors{a};
    if isempty(aName)
        continue
    end

    % --- Combine ALL track fragments matched to this actor into one
    %     trajectory (simulates a track-management/re-association layer
    %     that stitches fragmented tracks of the same real-world object). ---
    tids = matchTable.TrackID(strcmp(matchTable.MatchedActor, aName));
    tk = trackLog(ismember(trackLog.TrackID, tids), :);
    tk = sortrows(tk, 'Time');
    tk = tk(~[false; diff(tk.Time) == 0], :); % drop exact-duplicate timestamps

    tkSpeed = sqrt(tk.Vx.^2 + tk.Vy.^2);
    egoX_tk = interp1(egoTraj.Time, egoTraj.X, tk.Time, 'linear', 'extrap');
    egoY_tk = interp1(egoTraj.Time, egoTraj.Y, tk.Time, 'linear', 'extrap');
    fT = extractTrackFeatures(tk.Time, tk.X, tk.Y, tkSpeed, egoX_tk, egoY_tk);
    fT.ActorName = {aName};
    fT.NumFragments = numel(tids);

    % --- Feature from the ground-truth path (same actor) ---
    gt = gtLog(strcmp(gtLog.ActorName, aName), :);
    egoX_gt = interp1(egoTraj.Time, egoTraj.X, gt.Time, 'linear', 'extrap');
    egoY_gt = interp1(egoTraj.Time, egoTraj.Y, gt.Time, 'linear', 'extrap');
    fG = extractTrackFeatures(gt.Time, gt.X, gt.Y, gt.Speed, egoX_gt, egoY_gt);
    fG.ActorName = {aName};

    % --- Join the ground-truth safe/risky label + ActorType/Description, if available ---
    label = NaN;
    actorTypeVal = {''};
    descVal = {''};
    if ~isempty(labelsTable) && ismember('name', labelsTable.Properties.VariableNames)
        li = find(strcmp(labelsTable.name, aName), 1);
        if ~isempty(li)
            label = labelsTable.label(li);
            if ismember('actorType', labelsTable.Properties.VariableNames)
                actorTypeVal = labelsTable.actorType(li);
            end
            if ismember('description', labelsTable.Properties.VariableNames)
                descVal = labelsTable.description(li);
            end
        end
    end
    fT.label = label; fT.ScenarioID = scenarioID; fT.ActorType = actorTypeVal; fT.Description = descVal;
    fG.label = label; fG.ScenarioID = scenarioID; fG.ActorType = actorTypeVal; fG.Description = descVal;

    trackedRows{end+1} = fT; %#ok<AGROW>
    gtRows{end+1} = fG; %#ok<AGROW>
end

if ~isempty(trackedRows)
    trackedFeatures = vertcat(trackedRows{:});
    trackedFeatures = trackedFeatures(~isnan(trackedFeatures.label), :); % keep only labeled actors
else
    trackedFeatures = table();
end

if ~isempty(gtRows)
    gtFeatures = vertcat(gtRows{:});
    gtFeatures = gtFeatures(~isnan(gtFeatures.label), :);
else
    gtFeatures = table();
end
end