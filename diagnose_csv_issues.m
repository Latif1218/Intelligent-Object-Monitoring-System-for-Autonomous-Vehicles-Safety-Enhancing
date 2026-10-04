%% diagnose_csv_issues.m
%
% Prints the RAW first few lines of a few problem files, exactly as they
% are on disk (no MATLAB CSV parsing/interpretation). Run this and paste
% the full console output back - it will show us exactly how these files
% are malformed so we can write a precise fix instead of guessing.

clear; clc;

DATA_DIR = fullfile(pwd, 'dataset_100_scenarios');

filesToCheck = {
    'scenario_004_labels.csv'
    'scenario_007_labels.csv'
    'scenario_351_labels.csv'
    'scenario_351_actormap.csv'
    'scenario_351_trajectories.csv'
    'scenario_365_labels.csv'
};

for i = 1:numel(filesToCheck)
    fpath = fullfile(DATA_DIR, filesToCheck{i});
    fprintf('\n========================================\n');
    fprintf('FILE: %s\n', filesToCheck{i});
    fprintf('========================================\n');
    if ~isfile(fpath)
        fprintf('  (file does not exist)\n');
        continue;
    end

    fid = fopen(fpath, 'r');
    if fid == -1
        fprintf('  (could not open file)\n');
        continue;
    end

    for lineNum = 1:4
        tline = fgetl(fid);
        if ~ischar(tline)
            break;
        end
        fprintf('LINE %d: %s\n', lineNum, tline);
    end
    fclose(fid);
end

fprintf('\n\nDone. Please copy everything above (from the first ======== onward)\n');
fprintf('and paste it back.\n');
