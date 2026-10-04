function visualizeComplexReactiveScenario(scenarioID, datasetDir)
%VISUALIZECOMPLEXREACTIVESCENARIO Replays a saved complex, multi-crossroad,
%   multi-lane reactive scenario from its recorded trajectories (top-down
%   2D animation). Same idea as visualizeReactiveScenario.m, generalized
%   to multiple crossroads.
%
%   Usage:
%       visualizeComplexReactiveScenario(251)
%       visualizeComplexReactiveScenario(251, 'D:\Fahad_latif\dataset_100_scenarios')

if nargin < 2
    datasetDir = fullfile(pwd, 'dataset_100_scenarios');
end

matFile = fullfile(datasetDir, sprintf('scenario_%03d.mat', scenarioID));
data = load(matFile, 'recordTable', 'gtTable', 'meta', 'simInfo');
rt = data.recordTable;
crossroads = data.simInfo.crossroads;

actorIDs = unique(rt.ActorID);
times = unique(rt.Time);

figure('Name', sprintf('Complex reactive scenario %d playback', scenarioID));
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');

for c = 1:numel(crossroads)
    xline(crossroads(c).stopLinePos, 'r:', sprintf('Stop %d', c));
    xline(crossroads(c).crosswalkStart, 'm:');
    xline(crossroads(c).crosswalkEnd, 'm:');
end

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
    title(sprintf('Scenario %d (%s) — t = %.1f s', scenarioID, data.meta.egoManeuver, tt), 'Interpreter', 'none');
    drawnow;
    pause(0.02);
end

disp('Playback finished.');
end
