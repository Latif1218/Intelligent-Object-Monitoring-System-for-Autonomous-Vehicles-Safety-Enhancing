%% STEP 4.4-4.6: Build tracked (noisy) features, then compare accuracy
% This runs radar sensor simulation + tracking on a sample of scenarios,
% extracts features from both the ground-truth and noisy tracked paths,
% and compares how well your trained classifier performs on each.
%
% IMPORTANT: Sensor + tracking simulation is slow (it re-runs the full
% scenario timestep by timestep). Start with a small sample (scenarioIDs
% below) before scaling up to more of your 450 scenarios.

clear; clc;

datasetDir = 'dataset_100_scenarios';
scenarioIDs = [5 10 15 20 25 30 205 210 355 360]; % pick a mixed sample; edit as you like

allTracked = {};
allGT = {};

for i = 1:numel(scenarioIDs)
    id = scenarioIDs(i);
    matFile1 = fullfile(datasetDir, sprintf('scenario_%03d.mat', id));
    matFile2 = fullfile(datasetDir, sprintf('scenario_%03d_complexroute.mat', id));
    labelsFile = fullfile(datasetDir, sprintf('scenario_%03d_labels.csv', id));

    if isfile(matFile1)
        matFile = matFile1;
    elseif isfile(matFile2)
        matFile = matFile2;
    else
        fprintf('Skipping scenario %d: .mat file not found.\n', id);
        continue
    end

    fprintf('Processing scenario %d ...\n', id);
    try
        [tracked, gt] = buildTrackedFeatureDataset(matFile, labelsFile);
        allTracked{end+1} = tracked; %#ok<AGROW>
        allGT{end+1} = gt; %#ok<AGROW>
    catch ME
        fprintf('  Error on scenario %d: %s\n', id, ME.message);
    end
end

trackedFeatures = vertcat(allTracked{:});
gtFeatures = vertcat(allGT{:});

writetable(trackedFeatures, fullfile(datasetDir, 'features_tracked_noisy.csv'));
writetable(gtFeatures, fullfile(datasetDir, 'features_groundtruth_recomputed.csv'));

fprintf('\nSaved %d tracked rows and %d ground-truth rows.\n', ...
    height(trackedFeatures), height(gtFeatures));

%% Compare accuracy using your trained classifier
% Make sure trainedModel01 (the clean, leakage-free model) is loaded in
% the workspace first:
%   load('trainedRiskClassifier_clean.mat')
% It must have a .predictFcn and .RequiredVariables (this is what
% "Export Model" produces in Classification Learner).

if exist('trainedModel01', 'var')
    predictorNames = trainedModel01.RequiredVariables; % just the 10 clean motion features now

    gtPred = trainedModel01.predictFcn(gtFeatures(:, predictorNames));
    gtAcc = mean(gtPred == gtFeatures.label);

    trackedPred = trainedModel01.predictFcn(trackedFeatures(:, predictorNames));
    trackedAcc = mean(trackedPred == trackedFeatures.label);

    fprintf('\n--- Robustness comparison (clean, leakage-free model) ---\n');
    fprintf('Ground-truth (re-computed) accuracy : %.2f%%\n', gtAcc * 100);
    fprintf('Sensor-tracked (noisy)     accuracy : %.2f%%\n', trackedAcc * 100);
    fprintf('Accuracy drop due to sensor/tracking noise: %.2f percentage points\n', ...
        (gtAcc - trackedAcc) * 100);
else
    warning(['trainedModel01 not found in workspace. Run: load(''trainedRiskClassifier_clean.mat'') ' ...
        'then re-run this section (from "Compare accuracy" onward).']);
end