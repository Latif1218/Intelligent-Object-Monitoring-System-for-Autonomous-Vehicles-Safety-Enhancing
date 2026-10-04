function lib = complexScenarioLib()
%COMPLEXSCENARIOLIB Helper functions for the complex, multi-crossroad,
%   multi-lane reactive scenario generator
%   (generateComplexReactiveScenario.m).
%
%   lib = complexScenarioLib() returns a struct of function handles:
%       lib.idmAcceleration             - Intelligent Driver Model acceleration
%       lib.freeRoadAcceleration        - IDM acceleration with no leader
%       lib.trafficSignalState          - red/yellow/green state at time t
%       lib.crossingActive              - whether a crossing window covers time t
%       lib.egoControlAccelerationMulti - combines leader + ALL upcoming
%                                          crossroads + ALL upcoming crossings
%       lib.findLeaderInLane            - nearest vehicle ahead in the same lane
%       lib.shouldChangeLane            - simple "leader is close & slow,
%                                          other lane is clear" lane-change rule

    lib.idmAcceleration             = @idmAcceleration;
    lib.freeRoadAcceleration        = @freeRoadAcceleration;
    lib.trafficSignalState          = @trafficSignalState;
    lib.crossingActive              = @crossingActive;
    lib.egoControlAccelerationMulti = @egoControlAccelerationMulti;
    lib.findLeaderInLane            = @findLeaderInLane;
    lib.shouldChangeLane            = @shouldChangeLane;
end

% ---------------------------------------------------------------------
function acc = idmAcceleration(v, vLead, gap, p)
%IDMACCELERATION Intelligent Driver Model acceleration toward a leader.
    gap = max(gap, 0.1);
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
function tf = crossingActive(t, startEnd)
%CROSSINGACTIVE True if t falls inside [startEnd(1), startEnd(2)].
    tf = (t >= startEnd(1)) && (t <= startEnd(2));
end

% ---------------------------------------------------------------------
function [acc, info] = egoControlAccelerationMulti(t, posEgo, vEgo, ...
        leaderPos, leaderVel, hasLeader, crossroads, crossingEvents, p)
%EGOCONTROLACCELERATIONMULTI Combines leader car-following (in ego's
%   current lane), EVERY upcoming crossroad's traffic signal, and EVERY
%   upcoming pedestrian/child/cyclist crossing into a single
%   acceleration for the ego vehicle. The most restrictive (minimum)
%   acceleration wins. Obstacles the ego has already passed are ignored
%   automatically (their gap would be negative).

    candidates = [];
    modes = {};

    if hasLeader
        gap = leaderPos - posEgo - p.vehicleLength;
        if gap > 0
            candidates(end+1) = idmAcceleration(vEgo, leaderVel, gap, p); %#ok<AGROW>
            modes{end+1} = 'leader';
        end
    end

    for c = 1:numel(crossroads)
        cr = crossroads(c);
        if posEgo < cr.stopLinePos
            state = trafficSignalState(t, cr.signal);
            if strcmp(state, 'red') || strcmp(state, 'yellow')
                gap = cr.stopLinePos - posEgo;
                if gap > 0
                    candidates(end+1) = idmAcceleration(vEgo, 0, gap, p); %#ok<AGROW>
                    modes{end+1} = sprintf('signal_%d', c);
                end
            end
        end
    end

    for e = 1:numel(crossingEvents)
        ev = crossingEvents(e);
        if posEgo < ev.xCross && crossingActive(t, [ev.startT, ev.endT])
            gap = (ev.xCross - 3) - posEgo;
            if gap > 0
                candidates(end+1) = idmAcceleration(vEgo, 0, gap, p); %#ok<AGROW>
                modes{end+1} = sprintf('crossing_%d_%s', e, ev.actorType);
            end
        end
    end

    if isempty(candidates)
        acc = freeRoadAcceleration(vEgo, p);
        info.mode = 'free';
        return;
    end

    [acc, idx] = min(candidates);
    info.mode = modes{idx};
end

% ---------------------------------------------------------------------
function [leaderPos, leaderVel, hasLeader] = findLeaderInLane(egoX, egoLane, otherX, otherLane, otherVel)
%FINDLEADERINLANE Nearest vehicle ahead of egoX that is in the same lane.
    if isempty(otherX)
        leaderPos = NaN; leaderVel = NaN; hasLeader = false;
        return;
    end
    mask = (otherLane == egoLane) & (otherX > egoX);
    if any(mask)
        candX = otherX(mask);
        candV = otherVel(mask);
        [leaderPos, idx] = min(candX);
        leaderVel = candV(idx);
        hasLeader = true;
    else
        leaderPos = NaN; leaderVel = NaN; hasLeader = false;
    end
end

% ---------------------------------------------------------------------
function targetLane = shouldChangeLane(egoX, egoV, egoLane, numLanes, otherX, otherLane, otherVel, gapTrigger, safetyGap)
%SHOULDCHANGELANE Simple rule: if the leader in the current lane is
%   close and slow, and an adjacent lane is clear near egoX, move there.
%   Returns egoLane unchanged if no change is warranted/possible.

    targetLane = egoLane;
    [leadPos, leadVel, hasLead] = findLeaderInLane(egoX, egoLane, otherX, otherLane, otherVel);
    if ~hasLead
        return;
    end
    gap = leadPos - egoX;
    if gap > gapTrigger || leadVel > egoV * 0.75
        return; % leader is far enough away / moving fast enough, no need to change
    end

    candidateLanes = setdiff(0:(numLanes - 1), egoLane);
    for L = candidateLanes
        laneMask = (otherLane == L) & (abs(otherX - egoX) < safetyGap);
        if ~any(laneMask)
            targetLane = L;
            return;
        end
    end
end
