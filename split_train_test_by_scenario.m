%% split_train_test_by_scenario.m
%
% Purpose:
%   Split features_dataset.csv into train/test sets at the SCENARIO
%   level, not the row level. This avoids data leakage: actors from the
%   same scenario share road/traffic context, so if they end up split
%   across train and test, the model's reported accuracy will be
%   artificially optimistic (it partly "cheats" by having seen a very
%   similar scenario during training).
%
% How it works:
%   1. Get the unique list of ScenarioIDs (not rows).
%   2. Randomly assign ~80% of SCENARIOS to train, ~20% to test.
%   3. Every row (actor) from a given scenario goes entirely to
%      whichever set that scenario was assigned to.
%
% Output:
%   features_train.csv
%   features_test.csv

clear; clc;

%% ---- CONFIG ----
IN_FILE    = fullfile(pwd, 'features_dataset.csv');
TRAIN_FILE = fullfile(pwd, 'features_train.csv');
TEST_FILE  = fullfile(pwd, 'features_test.csv');
TRAIN_FRACTION = 0.8;   % 80% of scenarios -> train, 20% -> test
RNG_SEED = 42;           % fixed seed so the split is reproducible
%% -----------------

if ~isfile(IN_FILE)
    error('Could not find %s. Run build_feature_dataset.m first.', IN_FILE);
end

T = readtable(IN_FILE, 'Delimiter', ',', 'VariableNamingRule', 'preserve');

if ~ismember('ScenarioID', T.Properties.VariableNames)
    error('features_dataset.csv has no ScenarioID column - cannot do a scenario-level split.');
end

uniqueScenarios = unique(T.ScenarioID);
numScenarios = numel(uniqueScenarios);

rng(RNG_SEED);
shuffled = uniqueScenarios(randperm(numScenarios));

numTrain = round(TRAIN_FRACTION * numScenarios);
trainScenarios = shuffled(1:numTrain);
testScenarios  = shuffled(numTrain+1:end);

isTrainRow = ismember(T.ScenarioID, trainScenarios);
isTestRow  = ismember(T.ScenarioID, testScenarios);

trainTable = T(isTrainRow, :);
testTable  = T(isTestRow, :);

writetable(trainTable, TRAIN_FILE);
writetable(testTable, TEST_FILE);

fprintf('Total scenarios: %d  ->  train: %d scenarios, test: %d scenarios\n', ...
    numScenarios, numel(trainScenarios), numel(testScenarios));
fprintf('Total rows: %d  ->  train: %d rows, test: %d rows\n', ...
    height(T), height(trainTable), height(testTable));

fprintf('\nClass balance check:\n');
fprintf('  Train - Label counts:\n');
disp(groupcounts(trainTable, 'Label'));
fprintf('  Test  - Label counts:\n');
disp(groupcounts(testTable, 'Label'));

fprintf('\nSaved:\n  %s\n  %s\n', TRAIN_FILE, TEST_FILE);
fprintf('\nNext step: in Classification Learner, import features_train.csv,\n');
fprintf('train your models there (with cross-validation on the TRAIN set only),\n');
fprintf('then evaluate the final chosen model against features_test.csv,\n');
fprintf('which the model has never seen at all.\n');
