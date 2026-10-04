function lib = reactiveCarFollowingLib()
%REACTIVECARFOLLOWINGLIB Library of helper functions for the reactive
%   (car-following + traffic signal + pedestrian) scenario generator.
%
%   lib = reactiveCarFollowingLib() returns a struct of function handles:
%       lib.idmAcceleration        - Intelligent Driver Model acceleration
%       lib.freeRoadAcceleration   - IDM acceleration with no leader
%       lib.trafficSignalState     - red/yellow/green state at time t
%       lib.pedestrianActive       - whether a pedestrian event is active at time t
%       lib.egoControlAcceleration - combines leader + signal + pedestrian
%
%   Usage:
%       lib = reactiveCarFollowingLib();
%       acc = lib.idmAcceleration(v, vLead, gap, p);

    lib.idmAcceleration        = @idmAcceleration;
    lib.freeRoadAcceleration   = @freeRoadAcceleration;
    lib.trafficSignalState     = @trafficSignalState;
    lib.pedestrianActive       = @pedestrianActive;
    lib.egoControlAcceleration = @egoControlAcceleration;
end

% ---------------------------------------------------------------------
function acc = idmAcceleration(v, vLead, gap, p)
%IDMACCELERATION Intelligent Driver Model acceleration toward a leader.
%   v      - current speed (m/s)
%   vLead  - leader speed (m/s)         (use 0 for a stationary obstacle)
%   gap    - bumper-to-bumper distance to leader/obstacle (m), > 0
%   p      - struct with fields: v0, T, a, b, delta, s0

    gap = max(gap, 0.1); % avoid divide-by-zero / negative gap
    dv  = v - vLead;
    sStar = p.s0 + max(0, v * p.T + (v * dv) / (2 * sqrt(p.a * p.b)));
    acc = p.a * (1 - (v / p.v0)^p.delta - (sStar / gap)^2);
end

% ---------------------------------------------------------------------
function acc = freeRoadAcceleration(v, p)
%FREEROADACCELERATION IDM acceleration when there is no leader/obstacle.
    acc = p.a * (1 - (v / p.v0)^p.delta);
end

% ---------------------------------------------------------------------
function [state, nextChangeTime] = trafficSignalState(t, sig)
%TRAFFICSIGNALSTATE Returns 'green' | 'yellow' | 'red' at time t.
%   sig - struct with fields: offset, greenDur, yellowDur, redDur
    cycle = sig.greenDur + sig.yellowDur + sig.redDur;
    tc = mod(t + sig.offset, cycle);
    if tc < sig.greenDur
        state = 'green';
        nextChangeTime = sig.greenDur - tc;
    elseif tc < sig.greenDur + sig.yellowDur
        state = 'yellow';
        nextChangeTime = sig.greenDur + sig.yellowDur - tc;
    else
        state = 'red';
        nextChangeTime = cycle - tc;
    end
end

% ---------------------------------------------------------------------
function tf = pedestrianActive(t, pedEvents)
%PEDESTRIANACTIVE True if any pedestrian crossing event covers time t.
%   pedEvents - Nx2 matrix of [startTime, endTime]
    tf = false;
    for k = 1:size(pedEvents, 1)
        if t >= pedEvents(k, 1) && t <= pedEvents(k, 2)
            tf = true;
            return;
        end
    end
end

% ---------------------------------------------------------------------
function [acc, info] = egoControlAcceleration(t, posEgo, vEgo, ...
        leaderPos, leaderVel, hasLeader, road, sig, pedEvents, p)
%EGOCONTROLACCELERATION Combines leader car-following, traffic signal,
%   and pedestrian-crossing constraints into a single acceleration for
%   the ego vehicle. The most restrictive (minimum) acceleration wins.
%
%   info.mode tells which constraint was active: 'free' | 'leader' |
%   'signal' | 'pedestrian' (useful for logging/plots).

    candidates = [];
    modes = {};

    % 1) Vehicle currently in front of ego
    if hasLeader
        gap = leaderPos - posEgo - p.vehicleLength;
        if gap > 0
            candidates(end+1) = idmAcceleration(vEgo, leaderVel, gap, p); %#ok<AGROW>
            modes{end+1} = 'leader';
        end
    end

    % 2) Traffic signal (only matters if ego has not yet crossed the stop line)
    if posEgo < road.stopLinePos
        state = trafficSignalState(t, sig);
        if strcmp(state, 'red') || strcmp(state, 'yellow')
            gap = road.stopLinePos - posEgo;
            if gap > 0
                candidates(end+1) = idmAcceleration(vEgo, 0, gap, p); %#ok<AGROW>
                modes{end+1} = 'signal';
            end
        end
    end

    % 3) Pedestrian crossing (only matters if ego has not yet reached the crosswalk)
    if posEgo < road.crosswalkStart && pedestrianActive(t, pedEvents)
        gap = road.crosswalkStart - posEgo;
        if gap > 0
            candidates(end+1) = idmAcceleration(vEgo, 0, gap, p); %#ok<AGROW>
            modes{end+1} = 'pedestrian';
        end
    end

    % 4) Nothing restrictive ahead -> accelerate freely toward desired speed
    if isempty(candidates)
        acc = freeRoadAcceleration(vEgo, p);
        info.mode = 'free';
        return;
    end

    [acc, idx] = min(candidates);
    info.mode = modes{idx};
end
