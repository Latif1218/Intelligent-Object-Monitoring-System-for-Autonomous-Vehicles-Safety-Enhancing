function gtLog = getGroundTruthActorLog(scenario)
%GETGROUNDTRUTHACTORLOG Re-simulates the given scenario from the start and
% logs every actor's true position/speed at every timestep. Used both for
% matching tracker IDs to ground-truth actors, and for computing
% ground-truth features to compare against the noisy tracked features.

restart(scenario);
actorNames = arrayfun(@(a) a.Name, scenario.Actors, 'UniformOutput', false);
actorIDs = arrayfun(@(a) a.ActorID, scenario.Actors);

rows = {};
while advance(scenario)
    t = scenario.SimulationTime;
    poses = actorPoses(scenario);
    for i = 1:numel(poses)
        idx = find(actorIDs == poses(i).ActorID, 1);
        nm = actorNames{idx};
        spd = norm(poses(i).Velocity(1:2));
        rows(end+1, :) = {t, nm, poses(i).Position(1), poses(i).Position(2), spd, poses(i).Yaw}; %#ok<AGROW>
    end
end

gtLog = cell2table(rows, 'VariableNames', {'Time', 'ActorName', 'X', 'Y', 'Speed', 'Yaw'});
end