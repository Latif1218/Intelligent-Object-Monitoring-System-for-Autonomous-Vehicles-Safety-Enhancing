function wp = buildEgoRoute(turnType, turnX, farDistance)
%BUILDEGOROUTE Builds ego's route centerline (Nx3 waypoints) for one of:
%   'straight' - continues straight along the main road
%   'right'    - turns right (curves toward +y) at x = turnX
%   'left'     - turns left (curves toward -y) at x = turnX
%   'uturn'    - executes a U-turn at x = turnX and drives back
%
% turnX is ignored for 'straight'. farDistance controls how far the
% route continues after a turn (should exceed what ego can travel in
% the scenario duration, so it never runs out of road).

R = 15; % turn radius (m)

switch turnType
    case 'straight'
        wp = [0 0 0; farDistance 0 0];

    case 'right'
        preTurn = [0 0 0; turnX-R 0 0];
        t = linspace(-pi/2, 0, 8)';
        arc = [turnX + R*cos(t), R + R*sin(t), zeros(8,1)];
        postTurn = [arc(end,1), farDistance, 0];
        wp = [preTurn; arc; postTurn];

    case 'left'
        preTurn = [0 0 0; turnX-R 0 0];
        t = linspace(-pi/2, 0, 8)';
        arcR = [turnX + R*cos(t), R + R*sin(t), zeros(8,1)];
        arc = [arcR(:,1), -arcR(:,2), arcR(:,3)]; % mirror to curve toward -y
        postTurn = [arc(end,1), -farDistance, 0];
        wp = [preTurn; arc; postTurn];

    case 'uturn'
        preTurn = [0 0 0; turnX-R 0 0];
        t = linspace(-pi/2, pi/2, 12)';
        arc = [turnX + R*cos(t), R + R*sin(t), zeros(12,1)]; % half-circle loop
        postTurn = [-farDistance, arc(end,2), 0]; % drive back on the opposite lane
        wp = [preTurn; arc; postTurn];

    otherwise
        error('Unknown turnType: %s', turnType);
end
end
