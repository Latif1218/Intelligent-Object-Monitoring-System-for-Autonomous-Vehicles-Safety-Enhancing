%% MAIN SCRIPT: Generate 100 MORE scenarios (now including intersections and railway crossings)
% This appends new scenarios into the SAME dataset folder used before
% (dataset_100_scenarios), continuing the ScenarioID numbering (101-200)
% so nothing from your first 100 scenarios gets overwritten.
%
% Make sure complexityLibrary.m, createRoadNetwork.m, and generateScenario.m
% have been updated (they now include 'crossroad' and 'rail_crossing').

clear; clc;

startID = 101;   % continue after the first 100 you already generated
numNew  = 100;
outDir  = fullfile(pwd, 'dataset_100_scenarios'); % SAME folder as before
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

newSummaryRows = {};

for i = startID:(startID + numNew - 1)
    fprintf('Generating scenario %d ...\n', i);
    seed = 1000 + i; % reproducible but different every scenario

    [scenario, gtTable, meta] = generateScenario(i, seed);
    runAndRecordScenario(scenario, gtTable, meta, outDir);

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

fprintf('\nDone. %d new scenarios (IDs %d-%d) added to:\n%s\n', ...
    numNew, startID, startID + numNew - 1, outDir);
fprintf('Total scenarios in dataset_summary.csv: %d\n', height(combinedSummary));
