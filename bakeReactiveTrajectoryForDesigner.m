function bakedScenario = bakeReactiveTrajectoryForDesigner(scenarioID, datasetDir, outDir)
%BAKEREACTIVETRAJECTORYFORDESIGNER Converts a saved reactive scenario's
%   recorded (closed-loop) motion into ordinary trajectory()-based
%   actors, so it can be opened and played inside Driving Scenario
%   Designer -- its Run button, Step Forward, and 3D Display all work
%   with this baked copy.
%
%   This is for VISUALIZATION ONLY. It reconstructs, as an open-loop
%   trajectory, the exact path the reactive simulation already drove
%   (read from scenario_XXX_trajectories.csv / recordTable). The real
%   ground-truth reactive data stays untouched in scenario_XXX.mat --
%   this is a separate, extra file created just for viewing in the app.
%
%   Usage:
%       s = bakeReactiveTrajectoryForDesigner(207);
%       drivingScenarioDesigner(s);
%
%   Or in one line:
%       drivingScenarioDesigner(bakeReactiveTrajectoryForDesigner(207));

if nargin < 2 || isempty(datasetDir)
    datasetDir = fullfile(pwd, 'dataset_100_scenarios');
end
if nargin < 3
    outDir = datasetDir;
end

matFile = fullfile(datasetDir, sprintf('scenario_%03d.mat', scenarioID));
data = load(matFile, 'scenario', 'recordTable');
origScenario = data.scenario;
rt = data.recordTable;

[bakedScenario, ~] = createRoadNetwork('crossroad');
bakedScenario.StopTime = origScenario.StopTime;

origActors = origScenario.Actors;
allIDs = [origActors.ActorID];
actorIDs = unique(rt.ActorID);

FLOOR_SPEED = 0.05; % m/s -- keeps waypoints from being exactly identical,
                     % which avoids "ambiguous waypoint" issues while
                     % still looking essentially stopped in playback

for a = 1:numel(actorIDs)
    aid = actorIDs(a);
    srcIdx = find(allIDs == aid, 1);
    src = origActors(srcIdx); % ClassID/Name/Length/Width/Height metadata

    rows = rt(rt.ActorID == aid, :);
    rows = sortrows(rows, 'Time');

    wp  = [rows.X, rows.Y, rows.Z];
    spd = sqrt(rows.Vx.^2 + rows.Vy.^2);
    spd = max(spd, FLOOR_SPEED);

    if src.ClassID == 1 || src.ClassID == 2
        act = vehicle(bakedScenario, 'ClassID', src.ClassID, 'Name', src.Name, ...
            'Length', src.Length, 'Width', src.Width, 'Height', src.Height);
    else
        act = actor(bakedScenario, 'ClassID', src.ClassID, 'Name', src.Name, ...
            'Length', src.Length, 'Width', src.Width, 'Height', src.Height);
    end

    trajectory(act, wp, spd);
end

if ~isempty(outDir)
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end
    save(fullfile(outDir, sprintf('scenario_%03d_baked_for_designer.mat', scenarioID)), 'bakedScenario');
end

fprintf('Baked scenario %d ready. Open it with:\n', scenarioID);
fprintf('  drivingScenarioDesigner(bakedScenario)\n');

end
