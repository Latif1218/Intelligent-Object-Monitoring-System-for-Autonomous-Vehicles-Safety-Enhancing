%% export_batch2_trajectories.m
%
% Purpose:
%   Scenarios 351-450 were saved as *_complexroute.mat files containing the
%   full drivingScenario object, but no per-actor trajectory CSV in the
%   same format as scenarios 1-350 (*_trajectories.csv with columns:
%   ScenarioID, Time, ActorID, X, Y, Z, Vx, Vy, Vz, Yaw).
%
%   This script loads each saved drivingScenario, replays it frame-by-frame
%   with restart()/advance(), reads actorPoses() at every step, and writes
%   out a trajectories CSV in EXACTLY the same schema as batch 1-350, plus
%   an ActorID <-> Name lookup CSV so labels.csv can be joined correctly.
%
% Usage:
%   1. Put this script in the same folder as dataset_100_scenarios (or
%      point DATA_DIR / OUT_DIR below to the right paths).
%   2. Run in MATLAB. Requires Automated Driving Toolbox.
%
% Output (per scenario XXX in 351-450):
%   scenario_XXX_trajectories.csv   -> same columns as batch 1-350
%   scenario_XXX_actormap.csv       -> ActorID, ActorName  (for label join)

clear; clc;

%% ---- CONFIG: EDIT THESE PATHS IF NEEDED ----
DATA_DIR = fullfile(pwd, 'dataset_100_scenarios');   % where scenario_351_complexroute.mat etc live
OUT_DIR  = DATA_DIR;                                  % where new CSVs will be written
SCENARIO_IDS = 351:450;
%% ---------------------------------------------

if ~isfolder(DATA_DIR)
    error('DATA_DIR not found: %s. Edit DATA_DIR at the top of this script.', DATA_DIR);
end

fprintf('Starting export for %d scenarios (351-450)...\n', numel(SCENARIO_IDS));

failLog = {};

for sid = SCENARIO_IDS
    matFile = fullfile(DATA_DIR, sprintf('scenario_%d_complexroute.mat', sid));
    if ~isfile(matFile)
        warning('Missing file, skipping: %s', matFile);
        failLog{end+1} = sprintf('%d: file not found', sid); %#ok<AGROW>
        continue;
    end

    try
        S = load(matFile);

        % Find the drivingScenario object inside the loaded struct
        % (variable name may differ - search for the right class)
        scenario = [];
        fn = fieldnames(S);
        for k = 1:numel(fn)
            if isa(S.(fn{k}), 'drivingScenario')
                scenario = S.(fn{k});
                break;
            end
        end
        if isempty(scenario)
            warning('No drivingScenario object found in %s, skipping.', matFile);
            failLog{end+1} = sprintf('%d: no drivingScenario object in file', sid); %#ok<AGROW>
            continue;
        end

        actorList = scenario.Actors;   % array of Actor objects
        numActors = numel(actorList);

        % Build ActorID <-> Name map (ActorID = actor.ActorID property)
        actorIDs   = arrayfun(@(a) a.ActorID, actorList);
        actorNames = arrayfun(@(a) string(a.Name), actorList);
        actorMapTable = table(actorIDs(:), actorNames(:), ...
            'VariableNames', {'ActorID','ActorName'});
        writetable(actorMapTable, fullfile(OUT_DIR, sprintf('scenario_%d_actormap.csv', sid)));

        % Restart scenario to time 0
        restart(scenario);

        rows = struct('ScenarioID', {}, 'Time', {}, 'ActorID', {}, ...
                       'X', {}, 'Y', {}, 'Z', {}, ...
                       'Vx', {}, 'Vy', {}, 'Vz', {}, 'Yaw', {});
        rowIdx = 0;

        % Step through the scenario and log every actor's pose each frame
        while advance(scenario)
            t = scenario.SimulationTime;
            poses = actorPoses(scenario);  % struct array: ActorID, Position, Velocity, Yaw, ...

            for p = 1:numel(poses)
                rowIdx = rowIdx + 1;
                rows(rowIdx).ScenarioID = sid;
                rows(rowIdx).Time       = t;
                rows(rowIdx).ActorID    = poses(p).ActorID;
                rows(rowIdx).X  = poses(p).Position(1);
                rows(rowIdx).Y  = poses(p).Position(2);
                rows(rowIdx).Z  = poses(p).Position(3);
                rows(rowIdx).Vx = poses(p).Velocity(1);
                rows(rowIdx).Vy = poses(p).Velocity(2);
                rows(rowIdx).Vz = poses(p).Velocity(3);
                rows(rowIdx).Yaw = poses(p).Yaw;
            end
        end

        if rowIdx == 0
            warning('No frames captured for scenario %d (advance() returned false immediately).', sid);
            failLog{end+1} = sprintf('%d: zero frames captured', sid); %#ok<AGROW>
            continue;
        end

        trajTable = struct2table(rows);
        outFile = fullfile(OUT_DIR, sprintf('scenario_%d_trajectories.csv', sid));
        writetable(trajTable, outFile);

        fprintf('  scenario %d: %d actors, %d rows -> %s\n', ...
            sid, numActors, rowIdx, outFile);

    catch ME
        warning('Error on scenario %d: %s', sid, ME.message);
        failLog{end+1} = sprintf('%d: %s', sid, ME.message); %#ok<AGROW>
    end
end

fprintf('\nDone.\n');
if ~isempty(failLog)
    fprintf('%d scenario(s) had problems:\n', numel(failLog));
    for i = 1:numel(failLog)
        fprintf('  - %s\n', failLog{i});
    end
    fprintf('Fix these before running build_feature_dataset.m, or they will be skipped there too.\n');
else
    fprintf('All 100 scenarios (351-450) exported cleanly.\n');
end
