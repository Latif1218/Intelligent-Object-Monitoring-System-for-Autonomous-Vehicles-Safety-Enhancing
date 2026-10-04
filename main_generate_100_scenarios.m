%% MAIN SCRIPT: Generate 100 realistic driving scenarios for behavior classification
% Requires: Automated Driving Toolbox
%
% Each scenario:
%   - is ~120 seconds long
%   - is set on a highway, rural, village, or mixed road
%   - contains 5-6 real-life driving complexity events (safe + risky)
%   - includes a mix of actor types: car, truck, bus, bicycle, pedestrian, child
%
% Output: dataset_100_scenarios/
%   scenario_XXX.mat              - full scenario object + tables (reloadable)
%   scenario_XXX_trajectories.csv - actor pose log at every timestep
%   scenario_XXX_labels.csv       - ground-truth safe/risky label per actor/event
%   dataset_summary.csv           - one row per scenario, overview stats

clear; clc;

numScenarios = 100;
outDir = fullfile(pwd, 'dataset_100_scenarios');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

summaryRows = {};

for i = 1:numScenarios
    fprintf('Generating scenario %d / %d ...\n', i, numScenarios);
    seed = 1000 + i; % reproducible but different every scenario

    [scenario, gtTable, meta] = generateScenario(i, seed);
    recordTable = runAndRecordScenario(scenario, gtTable, meta, outDir); %#ok<NASGU>

    summaryRows(end+1, :) = {meta.scenarioID, meta.roadType, meta.duration, ...
        meta.numEvents, sum(gtTable.label == 1), sum(gtTable.label == 0)}; %#ok<AGROW>
end

summaryTable = cell2table(summaryRows, 'VariableNames', ...
    {'ScenarioID', 'RoadType', 'DurationSec', 'NumEvents', 'NumRiskyActors', 'NumSafeActors'});
writetable(summaryTable, fullfile(outDir, 'dataset_summary.csv'));

fprintf('\nDone. %d scenarios saved to:\n%s\n', numScenarios, outDir);

%% To visually inspect any single scenario afterwards, run separately:
% [scenario, gtTable, meta] = generateScenario(5, 1005);
% visualizeScenario(scenario);
%
% Or open it interactively in the app:
% drivingScenarioDesigner(scenario);