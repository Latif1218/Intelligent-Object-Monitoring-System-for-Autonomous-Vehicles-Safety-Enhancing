%% MAIN SCRIPT: Generate 100 COMPLEX reactive scenarios
% (2-3 crossroads, mixed traffic, lane changes, turns/U-turns)
%
% Adds scenarios 251-350 into the SAME dataset folder (dataset_100_scenarios).
%
% Each scenario has:
%   - 2-3 crossroads along one long road, each with its own randomized
%     traffic signal
%   - 2 lanes: ego AND background vehicles change lanes when the
%     vehicle ahead in their lane is close and slow, and the other lane
%     is clear
%   - ego reacts to whichever vehicle is currently ahead of it in its
%     lane (IDM), AND to every upcoming crossroad's signal, AND to every
%     upcoming pedestrian/child/cyclist crossing
%   - mixed background traffic: car / truck / bus / bicycle, each with
%     its own independently varying speed, occasionally braking hard,
%     and changing lanes around slower traffic
%   - mixed crossing actors at each crossroad: pedestrian / child /
%     cyclist, ~65% crossing safely on a red signal (label 0) and ~35%
%     jaywalking on green (label 1, risky)
%   - at one randomly chosen crossroad per scenario, ego may go straight
%     (~55%), turn right (~15%), turn left (~15%), or make a U-turn (~15%)
%   - road length is derived from the scenario duration and a generous
%     top-speed bound, so the ego vehicle can NEVER run off the end of
%     the road before the simulation finishes
%
% Requires (same folder / on path):
%   complexScenarioLib.m
%   generateComplexReactiveScenario.m
%   runAndRecordComplexReactiveScenario.m

clear; clc;

startID = 251;   % continue after scenarios 1-250 you already generated
numNew  = 100;
outDir  = fullfile(pwd, 'dataset_100_scenarios'); % SAME folder as before
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

newSummaryRows = {};

for i = startID:(startID + numNew - 1)
    fprintf('Generating complex reactive scenario %d ...\n', i);
    seed = 1000 + i;

    [scenario, gtTable, meta, simInfo] = generateComplexReactiveScenario(i, seed);
    runAndRecordComplexReactiveScenario(scenario, gtTable, meta, simInfo, outDir);

    newSummaryRows(end+1, :) = {meta.scenarioID, meta.roadType, meta.duration, ...
        meta.numEvents, sum(gtTable.label == 1), sum(gtTable.label == 0)}; %#ok<AGROW>
end

newSummaryTable = cell2table(newSummaryRows, 'VariableNames', ...
    {'ScenarioID', 'RoadType', 'DurationSec', 'NumEvents', 'NumRiskyActors', 'NumSafeActors'});

summaryFile = fullfile(outDir, 'dataset_summary.csv');
if isfile(summaryFile)
    oldSummary = readtable(summaryFile);
    combinedSummary = [oldSummary; newSummaryTable];
else
    combinedSummary = newSummaryTable;
end
writetable(combinedSummary, summaryFile);

fprintf('\nDone. %d complex reactive scenarios (IDs %d-%d) added to:\n%s\n', ...
    numNew, startID, startID + numNew - 1, outDir);
fprintf('Total scenarios in dataset_summary.csv: %d\n', height(combinedSummary));

%% To visually inspect a complex scenario afterwards:
% visualizeComplexReactiveScenario(251)
