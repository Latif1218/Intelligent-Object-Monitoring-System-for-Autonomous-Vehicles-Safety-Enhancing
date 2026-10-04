%% Build a LEAKAGE-FREE feature dataset for retraining the classifier
% Only genuine motion features are kept (no ActorName/ActorType/
% Description/ScenarioID), so the classifier can no longer "cheat" by
% reading the behavior name directly off the label metadata.
%
% This scans ALL scenario .mat + _labels.csv pairs in dataset_100_scenarios
% and, for each labeled actor, computes ground-truth motion features using
% the ego vehicle as the distance reference (same formula as Step 4).

clear; clc;

datasetDir = 'dataset_100_scenarios';
matFiles = [dir(fullfile(datasetDir, 'scenario_*.mat')); ...
            dir(fullfile(datasetDir, 'egofollow_scenario_*.mat'))];

% --- Scenario-level train/test split (80/20), BEFORE extracting features,
%     so all actors from the same scenario stay in only one split. This
%     avoids leakage between train and test. ---
rng(42); % reproducible split
n = numel(matFiles);
shuffledIdx = randperm(n);
nTrain = round(0.8 * n);
trainIdx = shuffledIdx(1:nTrain);
testIdx = shuffledIdx(nTrain+1:end);
splitLabel = repmat({'train'}, n, 1);
splitLabel(testIdx) = {'test'};

allRows = {};

for k = 1:numel(matFiles)
    matPath = fullfile(matFiles(k).folder, matFiles(k).name);
    [~, baseName] = fileparts(matFiles(k).name);
    baseName = erase(baseName, '_complexroute');
    labelsPath = fullfile(datasetDir, [baseName '_labels.csv']);

    if ~isfile(labelsPath)
        continue % no ground-truth labels for this file, skip
    end

    fprintf('Processing %s ...\n', matFiles(k).name);
    try
        data = load(matPath);
        gtLog = getGroundTruthActorLog(data.scenario);
        labelsTable = readtable(labelsPath);

        actorNamesAll = unique(gtLog.ActorName, 'stable');
        egoNameIdx = find(strcmpi(actorNamesAll, 'Ego'), 1);
        if isempty(egoNameIdx)
            egoNameIdx = find(contains(lower(actorNamesAll), 'ego'), 1);
        end
        if isempty(egoNameIdx)
            egoNameIdx = 1;
        end
        egoTraj = gtLog(strcmp(gtLog.ActorName, actorNamesAll{egoNameIdx}), :);

        for r = 1:height(labelsTable)
            aName = labelsTable.name{r};
            label = labelsTable.label(r);

            gt = gtLog(strcmp(gtLog.ActorName, aName), :);
            if height(gt) < 4
                continue
            end
            egoX = interp1(egoTraj.Time, egoTraj.X, gt.Time, 'linear', 'extrap');
            egoY = interp1(egoTraj.Time, egoTraj.Y, gt.Time, 'linear', 'extrap');

            f = extractTrackFeatures(gt.Time, gt.X, gt.Y, gt.Speed, egoX, egoY);
            f.label = label;
            f.split = splitLabel(k);
            allRows{end+1} = f; %#ok<AGROW>
        end
    catch ME
        fprintf('  Skipped due to error: %s\n', ME.message);
    end
end

cleanFeatures = vertcat(allRows{:});
cleanFeatures = cleanFeatures(~any(ismissing(cleanFeatures), 2), :); % drop rows with NaN features

trainFeatures = cleanFeatures(strcmp(cleanFeatures.split, 'train'), :);
testFeatures = cleanFeatures(strcmp(cleanFeatures.split, 'test'), :);
trainFeatures.split = [];
testFeatures.split = [];

writetable(trainFeatures, fullfile(datasetDir, 'features_clean_train.csv'));
writetable(testFeatures, fullfile(datasetDir, 'features_clean_test.csv'));

fprintf('\nDone.\n');
fprintf('Train: %d rows (%d safe, %d risky) from %d scenarios\n', ...
    height(trainFeatures), sum(trainFeatures.label==0), sum(trainFeatures.label==1), numel(trainIdx));
fprintf('Test:  %d rows (%d safe, %d risky) from %d scenarios\n', ...
    height(testFeatures), sum(testFeatures.label==0), sum(testFeatures.label==1), numel(testIdx));