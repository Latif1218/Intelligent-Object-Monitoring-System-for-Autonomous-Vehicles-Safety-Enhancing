%% MAIN SCRIPT: Generate 100 MORE complex scenarios (multi-crossroad, turns, obstacles)
% Each scenario:
%   - is ~150 seconds long, on a very long road so ego never runs out of road
%   - has 2-3 crossroads (intersections), each with its own traffic signal
%   - has ego perform one of: straight / right turn / left turn / U-turn
%   - has ego reactively follow the vehicle ahead, obey each signal,
%     yield to pedestrians, and change lanes around a stalled obstacle
%   - has mixed background traffic: car, truck, bicycle, pedestrians
%
% Scenario IDs continue at 351 (you already have 350 scenarios saved) so
% they are added alongside your existing ones in the SAME
% dataset_100_scenarios folder (nothing is overwritten). Output naming
% for these scenarios:
%   scenario_XXX_complexroute.mat   - full scenario + metadata + ego log
%   scenario_XXX_egolog.csv         - ego's frame-by-frame reactive driving log
%   scenario_XXX_labels.csv         - ground-truth safe/risky background actors

clear; clc;

startID = 351;
numNew  = 100;
outDir  = fullfile(pwd, 'dataset_100_scenarios'); % SAME folder as before
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

summaryRows = {};

for i = startID:(startID + numNew - 1)
    fprintf('Generating complex route scenario %d ...\n', i);
    seed = 9000 + i;

    [scenario, ego, gtTable, meta] = generateComplexRouteScenario(i, seed);
    runComplexRouteScenario(scenario, ego, meta, outDir);

    writetable(gtTable, fullfile(outDir, sprintf('scenario_%03d_labels.csv', i)));

    summaryRows(end+1, :) = {meta.scenarioID, meta.duration, meta.turnType, ...
        meta.numCrossroads, height(gtTable)}; %#ok<AGROW>
end

summaryTable = cell2table(summaryRows, 'VariableNames', ...
    {'ScenarioID', 'DurationSec', 'TurnType', 'NumCrossroads', 'NumBackgroundActors'});

% Merge with the existing summary file instead of overwriting it
summaryFile = fullfile(outDir, 'dataset_summary.csv');
if isfile(summaryFile)
    oldSummary = readtable(summaryFile);
    % Align columns: older summary has different columns (RoadType, NumEvents, etc.)
    % so keep the new complex-route summary as its own file to avoid mismatches.
    writetable(summaryTable, fullfile(outDir, 'dataset_summary_complexroute_351_450.csv'));
else
    writetable(summaryTable, fullfile(outDir, 'dataset_summary_complexroute_351_450.csv'));
end

fprintf('\nDone. %d new complex-route scenarios (IDs %d-%d) added to:\n%s\n', ...
    numNew, startID, startID + numNew - 1, outDir);