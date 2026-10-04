function [scenario, roadInfo] = createMultiCrossroadNetwork(numCrossroads)
%CREATEMULTICROSSROADNETWORK Builds a long main road (so ego never runs
% out of road) crossed by 2-3 perpendicular crossroads.

scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 150); % 2.5 minutes

mainSpan = 3000; % main road extends from -mainSpan to +mainSpan
lspecMain = lanespec(2, 'Width', 3.5);
road(scenario, [-mainSpan 0 0; mainSpan 0 0], 'Lanes', lspecMain, 'Name', 'MainRoad');

% Spread crossroad positions along the forward part of the main road
basePositions = [500 1200 2000];
crossroadXs = basePositions(1:numCrossroads) + randi([-80 80], 1, numCrossroads);
crossroadXs = sort(crossroadXs);

lspecCross = lanespec(2, 'Width', 3.3);
for k = 1:numCrossroads
    x = crossroadXs(k);
    road(scenario, [x -mainSpan 0; x mainSpan 0], 'Lanes', lspecCross, ...
        'Name', sprintf('Cross%d', k));
end

roadInfo.mainSpan = mainSpan;
roadInfo.crossroadXs = crossroadXs;
roadInfo.numCrossroads = numCrossroads;
end
