function recordTable = runAndRecordScenario(scenario, gtTable, meta, outDir)
%RUNANDRECORDSCENARIO Advances the scenario to completion, logs every
% actor's pose at every timestep, and saves the per-scenario dataset
% (.mat + .csv) into outDir.

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

logRows = {};
restart(scenario);

while advance(scenario)
    t = scenario.SimulationTime;
    poses = actorPoses(scenario);
    for i = 1:numel(poses)
        p = poses(i);
        logRows(end+1, :) = {meta.scenarioID, t, p.ActorID, ...
            p.Position(1), p.Position(2), p.Position(3), ...
            p.Velocity(1), p.Velocity(2), p.Velocity(3), p.Yaw}; %#ok<AGROW>
    end
end

recordTable = cell2table(logRows, 'VariableNames', ...
    {'ScenarioID', 'Time', 'ActorID', 'X', 'Y', 'Z', 'Vx', 'Vy', 'Vz', 'Yaw'});

scenarioName = sprintf('scenario_%03d', meta.scenarioID);

% Full scenario object + tables, for reloading/editing later in MATLAB
save(fullfile(outDir, [scenarioName '.mat']), 'scenario', 'gtTable', 'meta', 'recordTable');

% Flat CSVs, convenient for training ML models outside MATLAB too
writetable(recordTable, fullfile(outDir, [scenarioName '_trajectories.csv']));
writetable(gtTable, fullfile(outDir, [scenarioName '_labels.csv']));

end
