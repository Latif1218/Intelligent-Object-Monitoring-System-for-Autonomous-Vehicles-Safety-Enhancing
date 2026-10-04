%% LIVE DEMO: ego drives a long multi-turn route while classifying
% every nearby car/truck/bicycle/pedestrian/train in real time using the
% trained (clean, leakage-free) classifier. Safe = green box, Risky =
% red box, not-yet-classified = yellow box. Ego also reacts more
% cautiously (extra following distance / early braking / lane change)
% around anything currently predicted risky.

clear; clc; close all;

load('trainedRiskClassifier_clean.mat'); % gives trainedModel01

fprintf('Building long multi-turn demo scenario ...\n');
[scenario, ego, gtTable, meta] = generateLiveDemoScenario(1); %#ok<ASGLU>

fprintf('Running live classified demo (this plays out over the full scenario duration) ...\n');
summary = runLiveClassifiedDemo(scenario, ego, meta, trainedModel01);

writetable(summary.predLog, 'live_demo_predictions_log.csv');
writetable(summary.finalTable, 'live_demo_final_summary.csv');

fprintf('\nSaved live_demo_predictions_log.csv and live_demo_final_summary.csv\n');
