function visualizeReactiveScenario(scenarioID, datasetDir)
%VISUALIZEREACTIVESCENARIO Replays a saved reactive scenario from its
%   recorded trajectories (top-down 2D animation).
%
%   Use this instead of visualizeScenario.m for reactive scenarios: their
%   actors are moved manually, step by step, inside
%   runAndRecordReactiveScenario.m rather than through a pre-set
%   trajectory(), so drivingScenario's own plot(scenario)+advance()
%   playback (what visualizeScenario.m uses) will not show them moving.
%   This function instead replays the recordTable that was already saved
%   to scenario_XXX.mat.
%
%   Usage:
%       visualizeReactiveScenario(201)
%       visualizeReactiveScenario(201, 'D:\Fahad_latif\dataset_100_scenarios')

if nargin < 2
    datasetDir = fullfile(pwd, 'dataset_100_scenarios');
end

matFile = fullfile(datasetDir, sprintf('scenario_%03d.mat', scenarioID));
data = load(matFile, 'recordTable', 'gtTable', 'meta', 'simInfo');
rt   = data.recordTable;
road = data.simInfo.road;

actorIDs = unique(rt.ActorID);
times    = unique(rt.Time);

figure('Name', sprintf('Reactive scenario %d playback', scenarioID));
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');

xline(road.stopLinePos, 'r:', 'Stop line');
xline(road.crosswalkStart, 'm:', 'Crosswalk start');
xline(road.crosswalkEnd, 'm:', 'Crosswalk end');

colors = lines(numel(actorIDs));
h = gobjects(numel(actorIDs), 1);
for a = 1:numel(actorIDs)
    h(a) = plot(NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(a, :), ...
        'MarkerEdgeColor', 'k', 'DisplayName', sprintf('Actor %d', actorIDs(a)));
end
legend(h, 'Location', 'eastoutside');
xlim([min(rt.X) - 10, max(rt.X) + 10]);
ylim([min(rt.Y) - 10, max(rt.Y) + 10]);

for ti = 1:numel(times)
    tt = times(ti);
    rows = rt(rt.Time == tt, :);
    for a = 1:numel(actorIDs)
        r = rows(rows.ActorID == actorIDs(a), :);
        if ~isempty(r)
            set(h(a), 'XData', r.X(1), 'YData', r.Y(1));
        end
    end
    title(sprintf('Scenario %d (%s) — t = %.1f s', scenarioID, data.meta.roadType, tt));
    drawnow;
    pause(0.02);
end

disp('Playback finished.');
end
