function [scenario, roadInfo] = createRoadNetwork(roadType)
%CREATEROADNETWORK Creates a drivingScenario with a road matching roadType.
% roadType: 'highway' | 'rural' | 'village' | 'mixed'
%
% NOTE: Coordinates below are illustrative. Feel free to open the
% generated scenario in Driving Scenario Designer and adjust road
% centers/curvature visually -- File > Import from Workspace.

scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 120); % 2 minutes

switch roadType
    case 'highway'
        centers = [0 0 0; 300 0 0; 600 20 0; 900 20 0];
        lspec = lanespec(3, 'Width', 3.6); % 3-lane highway
        road(scenario, centers, 'Lanes', lspec, 'Name', 'Highway');
        roadInfo.speedLimit = 30; % m/s (~108 km/h)
        roadInfo.centers = centers;

    case 'rural'
        centers = [0 0 0; 250 40 0; 500 20 0; 750 60 0; 900 60 0];
        lspec = lanespec(2, 'Width', 3.2);
        road(scenario, centers, 'Lanes', lspec, 'Name', 'RuralRoad');
        roadInfo.speedLimit = 20; % ~72 km/h
        roadInfo.centers = centers;

    case 'village'
        centers = [0 0 0; 150 10 0; 300 10 0; 450 0 0; 600 0 0];
        lspec = lanespec(2, 'Width', 3.0);
        road(scenario, centers, 'Lanes', lspec, 'Name', 'VillageRoad');
        roadInfo.speedLimit = 8; % ~30 km/h
        roadInfo.centers = centers;

    case 'mixed'
        % Highway -> Rural -> Village transition on one long road
        centers = [0 0 0; 300 0 0; 600 20 0; 900 40 0; 1100 40 0; ...
                   1300 20 0; 1450 0 0; 1600 0 0];
        lspec = lanespec(3, 'Width', 3.6);
        road(scenario, centers, 'Lanes', lspec, 'Name', 'MixedRoad');
        roadInfo.speedLimit = 20; % average, segments vary in practice
        roadInfo.centers = centers;

    case 'crossroad'
        % A 4-way intersection: one east-west road crossed by a
        % north-south road, meeting at (200,0).
        centersEW = [0 0 0; 200 0 0; 400 0 0];
        centersNS = [200 -150 0; 200 0 0; 200 150 0];
        lspec = lanespec(2, 'Width', 3.3);
        road(scenario, centersEW, 'Lanes', lspec, 'Name', 'CrossroadEW');
        road(scenario, centersNS, 'Lanes', lspec, 'Name', 'CrossroadNS');
        roadInfo.speedLimit = 12; % intersection, lower speed
        roadInfo.centers = centersEW; % ego drives straight through east-west
        roadInfo.intersectionPoint = [200 0 0];

    case 'rail_crossing'
        % A road with a level (railway) crossing at x = 300.
        centers = [0 0 0; 300 0 0; 600 0 0];
        lspec = lanespec(2, 'Width', 3.2);
        road(scenario, centers, 'Lanes', lspec, 'Name', 'RailCrossingRoad');
        roadInfo.speedLimit = 12;
        roadInfo.centers = centers;
        roadInfo.crossingPoint = [300 0 0];  % where the tracks cross the road
        roadInfo.trackDirection = [0 1 0];   % tracks run north-south

    otherwise
        error('Unknown roadType: %s', roadType);
end
end