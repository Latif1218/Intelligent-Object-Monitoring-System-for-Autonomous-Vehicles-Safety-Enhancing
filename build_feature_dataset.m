%% build_feature_dataset.m
%
% Purpose:
%   Combine ALL 450 scenarios (1-350 original + 351-450 newly exported by
%   export_batch2_trajectories.m) into ONE feature table, one row per
%   labeled actor, ready to import into Classification Learner.
%
% Assumes the following files exist in DATA_DIR for EVERY scenario ID
% (either originally, or produced by export_batch2_trajectories.m):
%   scenario_XXX_trajectories.csv : ScenarioID, Time, ActorID, X, Y, Z,
%                                    Vx, Vy, Vz, Yaw
%   scenario_XXX_labels.csv       : name, actorType, label, description,
%                                    startTime
%   scenario_XXX_actormap.csv     : ActorID, ActorName   <- only needed for
%                                    351-450 (produced by the export script);
%                                    for 1-350 we derive the mapping below.
%
% Features computed per actor (over the actor's full logged trajectory):
%   avg_speed, max_speed, speed_std           (from sqrt(Vx^2+Vy^2+Vz^2))
%   max_accel, max_decel                      (from diff(speed)/diff(time))
%   max_lateral_rate                          (from diff(Y or lateral pos)/diff(time))
%   min_dist_to_ego, avg_dist_to_ego          (distance to ego actor at each frame)
%   yaw_rate_std                              (stability of heading)
%   duration                                  (time actor was tracked)
%
% Output:
%   features_dataset.csv  -> one row per (ScenarioID, ActorName), with
%                             all features + Label (0=safe, 1=risky)

clear; clc;

%% ---- CONFIG ----
DATA_DIR = fullfile(pwd, 'dataset_100_scenarios');
OUT_FILE = fullfile(pwd, 'features_dataset.csv');
ALL_SCENARIO_IDS = 1:450;
ID_DIGITS = 3;       % filenames are zero-padded, e.g. scenario_001_...
EGO_ACTOR_ID = 1;    % ActorID 1 = ego vehicle, per batch 1-350 convention.
%% -----------------

allRows = {};
skipped = {};

% ---- helper: find a column by name, tolerant of BOM/renaming issues ----
% (readtable sometimes mangles the FIRST header if the CSV has a UTF-8
% BOM, e.g. 'name' becomes something like 'xEF_xBB_xBF_name'. This finds
% the intended column regardless.)
findCol = @(varNames, target) ...
    localFindCol(varNames, target);

for sid = ALL_SCENARIO_IDS
    sidStr = sprintf('%0*d', ID_DIGITS, sid);   % zero-padded, e.g. '001', '351'
    trajFile  = fullfile(DATA_DIR, sprintf('scenario_%s_trajectories.csv', sidStr));
    labelFile = fullfile(DATA_DIR, sprintf('scenario_%s_labels.csv', sidStr));
    mapFile   = fullfile(DATA_DIR, sprintf('scenario_%s_actormap.csv', sidStr));

    if ~isfile(trajFile) || ~isfile(labelFile)
        skipped{end+1} = sprintf('%d: missing trajectories or labels csv (looked for scenario_%s_*)', sid, sidStr); %#ok<AGROW>
        continue;
    end

    try
        % IMPORTANT: force comma delimiter explicitly. readtable's
        % automatic delimiter detection was unreliable on some of these
        % files (misdetecting space as the delimiter), which shredded
        % the description text into many bogus columns. Forcing ',' and
        % 'preserve' fixes that and still correctly handles quoted
        % descriptions that contain commas (e.g. "...mid-block, outside...").
        rtOpts = {'Delimiter', ',', 'VariableNamingRule', 'preserve'};
        traj = readtable(trajFile, rtOpts{:});
        labels = readtable(labelFile, rtOpts{:});

        % ---- Resolve label column names robustly (handles BOM issue) ----
        lblVars = labels.Properties.VariableNames;
        nameCol = findCol(lblVars, 'name');
        typeCol = findCol(lblVars, 'actorType');
        labelCol = findCol(lblVars, 'label');
        descCol = findCol(lblVars, 'description');
        if isempty(nameCol) || isempty(labelCol)
            error('labels.csv missing name/label column. Found columns: %s', strjoin(lblVars, ', '));
        end
        labelNames = labels.(nameCol);

        % ---- Resolve trajectory column names robustly ----
        trajVars = traj.Properties.VariableNames;
        actorIdCol = findCol(trajVars, 'ActorID');
        timeCol    = findCol(trajVars, 'Time');
        xCol = findCol(trajVars, 'X'); yCol = findCol(trajVars, 'Y');
        vxCol = findCol(trajVars, 'Vx'); vyCol = findCol(trajVars, 'Vy'); vzCol = findCol(trajVars, 'Vz');
        yawCol = findCol(trajVars, 'Yaw');
        if isempty(actorIdCol) || isempty(timeCol)
            error(['trajectories.csv missing ActorID/Time column. Found columns: %s. ' ...
                   'This usually means export_batch2_trajectories.m has not been (successfully) ' ...
                   're-run for this scenario yet.'], strjoin(trajVars, ', '));
        end

        % ---- Build ActorID -> ActorName map ----
        if isfile(mapFile)
            % batch 351-450: explicit map exported alongside trajectories
            amap = readtable(mapFile, rtOpts{:});
            amapVars = amap.Properties.VariableNames;
            mapIdCol = findCol(amapVars, 'ActorID');
            mapNameCol = findCol(amapVars, 'ActorName');
            if isempty(mapIdCol) || isempty(mapNameCol)
                error('actormap.csv missing ActorID/ActorName column. Found columns: %s', strjoin(amapVars, ', '));
            end
            id2name = containers.Map(amap.(mapIdCol), cellstr(amap.(mapNameCol)));
        else
            % batch 1-350: infer mapping. Convention observed: labels.csv
            % rows are named actor_1_..., actor_2_... etc and ActorID in
            % trajectories.csv = (label index + 1), with ActorID 1 = ego.
            n = height(labels);
            ids = (2:(n+1))';  % ActorID 2..n+1 map to labels row 1..n
            id2name = containers.Map(num2cell(ids), cellstr(labelNames));
        end

        % ---- Determine ego trajectory for distance-to-ego feature ----
        egoRows = traj(traj.(actorIdCol) == EGO_ACTOR_ID, :);
        hasEgo = height(egoRows) > 0;
        if hasEgo
            egoRows = sortrows(egoRows, timeCol);
        end

        % ---- Loop over labeled actors ----
        actorIDsInMap = cell2mat(keys(id2name));
        for ai = 1:numel(actorIDsInMap)
            aid = actorIDsInMap(ai);
            aname = id2name(aid);

            labelRow = labels(strcmp(labelNames, aname), :);
            if height(labelRow) ~= 1
                continue;  % no matching label, skip (don't guess)
            end

            actorTraj = traj(traj.(actorIdCol) == aid, :);
            if height(actorTraj) < 3
                continue;  % too few points to compute derivatives meaningfully
            end
            actorTraj = sortrows(actorTraj, timeCol);

            t  = actorTraj.(timeCol);
            vx = actorTraj.(vxCol); vy = actorTraj.(vyCol); vz = actorTraj.(vzCol);
            speed = sqrt(vx.^2 + vy.^2 + vz.^2);

            dt = diff(t);
            dt(dt == 0) = NaN;  % avoid div-by-zero

            % Longitudinal accel/decel from speed changes
            dSpeed = diff(speed);
            accelSeries = dSpeed ./ dt;
            max_accel = max(accelSeries, [], 'omitnan');
            max_decel = min(accelSeries, [], 'omitnan');  % most negative

            % Lateral movement rate (use Y as lateral proxy; adjust if your
            % road convention differs)
            dY = diff(actorTraj.(yCol));
            lateralRateSeries = dY ./ dt;
            max_lateral_rate = max(abs(lateralRateSeries), [], 'omitnan');

            % Yaw rate stability
            if ~isempty(yawCol)
                yawUnwrapped = unwrap(deg2rad(actorTraj.(yawCol)));
                dYaw = diff(yawUnwrapped);
                yawRateSeries = dYaw ./ dt;
                yaw_rate_std = std(yawRateSeries, 'omitnan');
            else
                yaw_rate_std = NaN;
            end

            % Distance to ego
            if hasEgo && aid ~= EGO_ACTOR_ID
                % interpolate ego position onto actor's timestamps
                egoX = interp1(egoRows.(timeCol), egoRows.(xCol), t, 'linear', 'extrap');
                egoY = interp1(egoRows.(timeCol), egoRows.(yCol), t, 'linear', 'extrap');
                distToEgo = sqrt((actorTraj.(xCol) - egoX).^2 + (actorTraj.(yCol) - egoY).^2);
                min_dist_to_ego = min(distToEgo, [], 'omitnan');
                avg_dist_to_ego = mean(distToEgo, 'omitnan');
            else
                min_dist_to_ego = NaN;
                avg_dist_to_ego = NaN;
            end

            row = table();
            row.ScenarioID       = sid;
            row.ActorName        = string(aname);
            row.ActorType        = string(labelRow.(typeCol)(1));
            row.avg_speed        = mean(speed, 'omitnan');
            row.max_speed        = max(speed, [], 'omitnan');
            row.speed_std        = std(speed, 'omitnan');
            row.max_accel        = max_accel;
            row.max_decel        = max_decel;
            row.max_lateral_rate = max_lateral_rate;
            row.yaw_rate_std     = yaw_rate_std;
            row.min_dist_to_ego  = min_dist_to_ego;
            row.avg_dist_to_ego  = avg_dist_to_ego;
            row.duration         = t(end) - t(1);
            row.Label             = labelRow.(labelCol)(1);   % 0 = safe, 1 = risky
            row.Description        = string(labelRow.(descCol)(1));

            allRows{end+1} = row; %#ok<AGROW>
        end

    catch ME
        skipped{end+1} = sprintf('%d: %s', sid, ME.message); %#ok<AGROW>
    end
end

if isempty(allRows)
    error('No rows built at all - check DATA_DIR and file naming.');
end

featuresTable = vertcat(allRows{:});
writetable(featuresTable, OUT_FILE);

fprintf('Wrote %d actor rows from %d scenarios to %s\n', ...
    height(featuresTable), numel(ALL_SCENARIO_IDS) - numel(skipped), OUT_FILE);

if ~isempty(skipped)
    fprintf('\n%d scenario(s) skipped or had errors:\n', numel(skipped));
    for i = 1:numel(skipped)
        fprintf('  - %s\n', skipped{i});
    end
end

fprintf('\nNext step: open Classification Learner, import %s,\n', OUT_FILE);
fprintf('set "Label" as the response variable, and train/compare classifiers.\n');

%% ---- local helper function ----
function colName = localFindCol(varNames, target)
    % Case-insensitive exact match first
    idx = find(strcmpi(varNames, target), 1);
    if isempty(idx)
        % Fall back: strip any leading non-letter junk (e.g. BOM artifacts
        % like 'xEF_xBB_xBF_name') and compare again
        cleaned = regexprep(varNames, '^[^A-Za-z]*', '');
        idx = find(strcmpi(cleaned, target), 1);
    end
    if isempty(idx)
        colName = '';
    else
        colName = varNames{idx};
    end
end
