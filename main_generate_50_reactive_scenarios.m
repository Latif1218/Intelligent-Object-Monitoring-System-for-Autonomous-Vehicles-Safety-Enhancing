%% MAIN SCRIPT: Generate 50 REACTIVE car-following scenarios (crossroad + signal + pedestrians)
% Adds scenarios 201-250 into the SAME dataset folder used before
% (dataset_100_scenarios), continuing the ScenarioID numbering so
% nothing from your first 200 scenarios gets overwritten.
%
% Unlike generateScenario.m (fixed complexity-library waypoints), these
% scenarios are CLOSED-LOOP / reactive:
%   - ego vehicle speeds up/slows down based on whichever vehicle is
%     currently ahead of it (IDM car-following, re-evaluated every
%     timestep — the leader can change as vehicles overtake each other)
%   - background vehicles move independently, each at its own randomly
%     varying speed (they do not react to ego, the signal, or pedestrians)
%   - a crossroad with a traffic signal: ego stops on red/yellow, goes
%     on green
%   - pedestrians cross the crossroad at 1-3 random time windows per
%     scenario; ego stops for them before the crosswalk
%
% Requires (same folder / on path):
%   reactiveCarFollowingLib.m
%   generateReactiveScenario.m
%   runAndRecordReactiveScenario.m
%   createRoadNetwork.m   (your existing file, reused as-is)
%
% Output (same folder, same format as before):
%   dataset_100_scenarios/scenario_201.mat ... scenario_250.mat
%   dataset_100_scenarios/scenario_2XX_trajectories.csv
%   dataset_100_scenarios/scenario_2XX_labels.csv
%   dataset_100_scenarios/dataset_summary.csv   (merged with existing rows)

clear; clc;

startID = 201;   % continue after scenarios 1-200 you already generated
numNew  = 50;
outDir  = fullfile(pwd, 'dataset_100_scenarios'); % SAME folder as before
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

newSummaryRows = {};

for i = startID:(startID + numNew - 1)
    fprintf('Generating reactive scenario %d ...\n', i);
    seed = 1000 + i; % reproducible but different every scenario

    [scenario, gtTable, meta, simInfo] = generateReactiveScenario(i, seed);
    runAndRecordReactiveScenario(scenario, gtTable, meta, simInfo, outDir);

    newSummaryRows(end+1, :) = {meta.scenarioID, meta.roadType, meta.duration, ...
        meta.numEvents, sum(gtTable.label == 1), sum(gtTable.label == 0)}; %#ok<AGROW>
end

newSummaryTable = cell2table(newSummaryRows, 'VariableNames', ...
    {'ScenarioID', 'RoadType', 'DurationSec', 'NumEvents', 'NumRiskyActors', 'NumSafeActors'});

% Merge with the existing summary file instead of overwriting it
summaryFile = fullfile(outDir, 'dataset_summary.csv');
if isfile(summaryFile)
    oldSummary = readtable(summaryFile);
    combinedSummary = [oldSummary; newSummaryTable];
else
    combinedSummary = newSummaryTable;
end
writetable(combinedSummary, summaryFile);

fprintf('\nDone. %d reactive scenarios (IDs %d-%d) added to:\n%s\n', ...
    numNew, startID, startID + numNew - 1, outDir);
fprintf('Total scenarios in dataset_summary.csv: %d\n', height(combinedSummary));

%% To visually inspect a reactive scenario afterwards:
% visualizeReactiveScenario(201)                       % replays from the saved .mat
% (visualizeScenario.m will NOT animate these — see the note in
%  visualizeReactiveScenario.m for why.)
