function summary = runLiveClassifiedDemo(scenario, ego, meta, trainedModel)
%RUNLIVECLASSIFIEDDEMO Drives ego reactively along its route while, in
% real time, classifying every nearby actor as safe/risky using
% trainedModel (the clean, leakage-free classifier), visualizing the
% result as green (safe) / red (risky) / yellow (not yet classified)
% boxes, and logging every prediction made during the run.
%
% Usage:
%   load('trainedRiskClassifier_clean.mat');   % gives trainedModel01
%   [scenario, ego, gtTable, meta] = generateLiveDemoScenario(1);
%   summary = runLiveClassifiedDemo(scenario, ego, meta, trainedModel01);

predictorNames = trainedModel.RequiredVariables;

dt = scenario.SampleTime;
maxSpeed  = 15;
maxDecel  = 4;
maxAccel  = 2;
safeTimeGap = 2.0;
minGap    = 5;
riskyGapMultiplier = 1.8; % extra following distance kept from predicted-risky actors

laneWidth   = 3.5;
maxLatRate  = 1.5;
lookaheadObs = 30;
passMargin   = 15;

predEverySteps = 20;   % re-classify every 2 s
renderEverySteps = 5;  % redraw every 0.5 s
minSamplesForPred = 4;

actorNames = arrayfun(@(a) a.Name, scenario.Actors, 'UniformOutput', false);
actorIDs   = arrayfun(@(a) a.ActorID, scenario.Actors);
numActors  = numel(actorNames);

% --- Per-actor rolling history buffers (ground-truth positions) ---
buf = struct();
for i = 1:numActors
    key = matlab.lang.makeValidName(actorNames{i});
    buf.(key) = struct('Time', [], 'X', [], 'Y', [], 'Speed', []);
end
riskLabel = containers.Map(actorNames, num2cell(nan(1, numActors)));

s = 0;
egoSpeed = meta.egoInitialSpeed;
lateralOffset = 0;

predLogRows = {};
stepCount = 0;

% --- Figure setup: clean 2D top-down (radar-style) chase view ---
viewHalfWidth = 65; % meters shown left/right and ahead/behind of ego

fig = figure('Name', 'Live Classified Driving Demo', 'Color', [0.10 0.11 0.13], ...
    'Position', [80 80 1200 850]);
ax = axes('Parent', fig, 'Color', [0.10 0.11 0.13]);
hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on');
set(ax, 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5], 'GridColor', [0.4 0.4 0.4]);
grid(ax, 'on'); ax.GridAlpha = 0.25;
xlabel(ax, 'X (m)', 'Color', [0.8 0.8 0.8]);
ylabel(ax, 'Y (m)', 'Color', [0.8 0.8 0.8]);

% Static full route (background reference line, drawn once)
routeLine = plot(ax, meta.egoWaypoints(:,1), meta.egoWaypoints(:,2), ...
    '--', 'Color', [0.4 0.45 0.5], 'LineWidth', 1); %#ok<NASGU>

restart(scenario);

while advance(scenario)
    t = scenario.SimulationTime;
    stepCount = stepCount + 1;
    poses = actorPoses(scenario);

    [basePos, yawDeg] = pathPointAtDistance(meta.egoWaypoints, meta.egoCumDist, s);
    yawRad = deg2rad(yawDeg);
    perp = [-sin(yawRad), cos(yawRad)];
    egoPos = basePos + [perp * lateralOffset, 0];

    % --- Update history buffers for every actor (ground truth) ---
    for ii = 1:numel(poses)
        idx = find(actorIDs == poses(ii).ActorID, 1);
        nm = actorNames{idx};
        key = matlab.lang.makeValidName(nm);
        b = buf.(key);
        b.Time(end+1) = t;
        b.X(end+1) = poses(ii).Position(1);
        b.Y(end+1) = poses(ii).Position(2);
        b.Speed(end+1) = norm(poses(ii).Velocity(1:2));
        buf.(key) = b;
    end

    % --- Periodically re-classify each actor from its buffer so far ---
    if mod(stepCount, predEverySteps) == 0
        egoKey = matlab.lang.makeValidName('Ego');
        egoBuf = buf.(egoKey);
        for i = 1:numActors
            nm = actorNames{i};
            if strcmp(nm, 'Ego')
                continue
            end
            key = matlab.lang.makeValidName(nm);
            b = buf.(key);
            if numel(b.Time) < minSamplesForPred
                continue
            end
            egoX = interp1(egoBuf.Time, egoBuf.X, b.Time, 'linear', 'extrap');
            egoY = interp1(egoBuf.Time, egoBuf.Y, b.Time, 'linear', 'extrap');
            feat = extractTrackFeatures(b.Time, b.X, b.Y, b.Speed, egoX, egoY);
            if any(ismissing(feat(:, predictorNames)), 'all')
                continue
            end
            pred = trainedModel.predictFcn(feat(:, predictorNames));
            riskLabel(nm) = double(pred);
            predLogRows(end+1, :) = {t, nm, double(pred)}; %#ok<AGROW>
        end
    end

    % --- 1) Car-following, with extra caution around predicted-risky actors ---
    gapToLead = inf;
    leadSpeed = maxSpeed;
    leadIsRisky = false;
    for ii = 1:numel(poses)
        idx = find(actorIDs == poses(ii).ActorID, 1);
        nm = actorNames{idx};
        if strcmp(nm, 'Ego')
            continue
        end
        rel = poses(ii).Position(1:2) - egoPos(1:2);
        along = rel(1)*cos(yawRad) + rel(2)*sin(yawRad);
        lateral = -rel(1)*sin(yawRad) + rel(2)*cos(yawRad);
        if along > 0 && abs(lateral) < 2.2 && along < gapToLead
            gapToLead = along;
            leadSpeed = norm(poses(ii).Velocity(1:2));
            leadIsRisky = isKey(riskLabel, nm) && riskLabel(nm) == 1;
        end
    end
    gapMultiplier = 1;
    if leadIsRisky
        gapMultiplier = riskyGapMultiplier;
    end
    desiredGap = gapMultiplier * (minGap + safeTimeGap * egoSpeed);
    if gapToLead < desiredGap
        speedTarget_lead = max(0, leadSpeed - (desiredGap - gapToLead) * 0.5);
    else
        speedTarget_lead = maxSpeed;
    end

    % --- 2) Next traffic signal ahead ---
    speedTarget_signal = maxSpeed;
    nextEventDist = inf;
    for e = 1:numel(meta.intersectionEvents)
        ev = meta.intersectionEvents(e);
        distToStop = (ev.s - ev.stopLineDist) - s;
        if distToStop > -2 && distToStop < nextEventDist
            nextEventDist = distToStop;
            phase = mod(t + ev.phaseOffset, ev.cycleLen);
            isRed = phase >= ev.greenDuration;
            if isRed && distToStop < 40
                brakingDist = max(egoSpeed^2 / (2*maxDecel), 1);
                if distToStop < brakingDist
                    speedTarget_signal = max(0, egoSpeed - maxDecel*dt*3);
                end
                if distToStop <= 1
                    speedTarget_signal = 0;
                end
            end
        end
    end

    % --- 3) Pedestrians (and trains) ahead: yield, extra caution if risky ---
    speedTarget_ped = maxSpeed;
    for ii = 1:numel(poses)
        idx = find(actorIDs == poses(ii).ActorID, 1);
        nm = actorNames{idx};
        if contains(nm, 'ped') || startsWith(nm, 'Train')
            rel = poses(ii).Position(1:2) - egoPos(1:2);
            along = rel(1)*cos(yawRad) + rel(2)*sin(yawRad);
            lateral = -rel(1)*sin(yawRad) + rel(2)*cos(yawRad);
            lookDist = 20;
            if isKey(riskLabel, nm) && riskLabel(nm) == 1
                lookDist = 35; % react earlier to a predicted-risky pedestrian/train
            end
            if along > 0 && along < lookDist && abs(lateral) < 4
                speedTarget_ped = 0;
            end
        end
    end

    speedTarget = min([speedTarget_lead, speedTarget_signal, speedTarget_ped, maxSpeed]);
    if speedTarget > egoSpeed
        egoSpeed = min(speedTarget, egoSpeed + maxAccel*dt);
    else
        egoSpeed = max(speedTarget, egoSpeed - maxDecel*dt);
    end
    egoSpeed = max(egoSpeed, 0);

    % --- Lateral avoidance: stalled obstacles AND predicted-risky nearby actors ---
    targetLateral = 0;
    for o = 1:numel(meta.obstacleEvents)
        obsS = meta.obstacleEvents(o).s;
        if s + lookaheadObs > obsS && s < obsS + passMargin
            targetLateral = laneWidth;
        end
    end
    for ii = 1:numel(poses)
        idx = find(actorIDs == poses(ii).ActorID, 1);
        nm = actorNames{idx};
        if strcmp(nm, 'Ego') || ~isKey(riskLabel, nm) || riskLabel(nm) ~= 1
            continue
        end
        rel = poses(ii).Position(1:2) - egoPos(1:2);
        along = rel(1)*cos(yawRad) + rel(2)*sin(yawRad);
        lateral = -rel(1)*sin(yawRad) + rel(2)*cos(yawRad);
        if along > 0 && along < 20 && abs(lateral) < 2.2
            targetLateral = laneWidth; % steer around a predicted-risky vehicle ahead
        end
    end
    if lateralOffset < targetLateral
        lateralOffset = min(targetLateral, lateralOffset + maxLatRate*dt);
    else
        lateralOffset = max(targetLateral, lateralOffset - maxLatRate*dt);
    end

    s = s + egoSpeed * dt;
    [basePos, yawDeg] = pathPointAtDistance(meta.egoWaypoints, meta.egoCumDist, s);
    yawRad = deg2rad(yawDeg);
    perp = [-sin(yawRad), cos(yawRad)];
    finalPos = basePos + [perp * lateralOffset, 0];
    ego.Position = finalPos;
    ego.Yaw = yawDeg;
    ego.Velocity = egoSpeed * [cos(yawRad), sin(yawRad), 0];

    % --- Render: 2D top-down, chase-cam centered on ego ---
    if mod(stepCount, renderEverySteps) == 0
        cla(ax);
        plot(ax, meta.egoWaypoints(:,1), meta.egoWaypoints(:,2), ...
            '--', 'Color', [0.4 0.45 0.5], 'LineWidth', 1);

        nSafe = 0; nRisky = 0; nUnknown = 0;
        for ii = 1:numel(poses)
            idx = find(actorIDs == poses(ii).ActorID, 1);
            nm = actorNames{idx};
            if strcmp(nm, 'Ego')
                continue
            end
            if isKey(riskLabel, nm) && riskLabel(nm) == 1
                col = [0.90 0.15 0.15]; nRisky = nRisky + 1;
            elseif isKey(riskLabel, nm) && riskLabel(nm) == 0
                col = [0.15 0.80 0.35]; nSafe = nSafe + 1;
            else
                col = [0.95 0.80 0.10]; nUnknown = nUnknown + 1;
            end
            actH = scenario.Actors(strcmp(actorNames, nm));
            drawBox2D(ax, poses(ii).Position, poses(ii).Yaw, actH.Length, actH.Width, col, nm);
        end

        % Ego drawn last so it's always on top, with a heading arrow
        drawBox2D(ax, finalPos, yawDeg, 4.7, 1.8, [0.15 0.55 1.0], 'Ego');

        % Chase-cam: keep ego centered in a fixed-size window
        xlim(ax, [finalPos(1)-viewHalfWidth, finalPos(1)+viewHalfWidth]);
        ylim(ax, [finalPos(2)-viewHalfWidth, finalPos(2)+viewHalfWidth]);

        % Live info panel (top-left corner, screen-fixed, redrawn each frame since cla() wipes it)
        text(ax, 0.02, 0.97, sprintf('t = %.0f s / %.0f s\nSafe: %d   Risky: %d   Unclassified: %d', ...
            t, meta.duration, nSafe, nRisky, nUnknown), ...
            'Parent', ax, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
            'FontSize', 12, 'FontWeight', 'bold', 'Color', [1 1 1], ...
            'BackgroundColor', [0 0 0 0.35], 'Margin', 6);

        % Static color key, top-right corner (screen-fixed)
        text(ax, 0.98, 0.97, '\color[rgb]{0.15,0.80,0.35}\bullet safe   \color[rgb]{0.90,0.15,0.15}\bullet risky   \color[rgb]{0.95,0.80,0.10}\bullet unclassified   \color[rgb]{0.15,0.55,1.0}\bullet ego', ...
            'Parent', ax, 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
            'VerticalAlignment', 'top', 'FontSize', 10, 'Color', [0.9 0.9 0.9]);

        title(ax, sprintf('Live Classified Driving Demo   |   %.0f%% along route', ...
            100*s/meta.egoCumDist(end)), 'Color', 'w', 'FontSize', 13);

        drawnow limitrate
    end
end

predLog = cell2table(predLogRows, 'VariableNames', {'Time', 'ActorName', 'PredictedLabel'});

% --- Final per-actor summary: last prediction made for each actor ---
finalRows = {};
allActors = keys(riskLabel);
for i = 1:numel(allActors)
    nm = allActors{i};
    lbl = riskLabel(nm);
    if ~isnan(lbl)
        finalRows(end+1, :) = {nm, lbl}; %#ok<AGROW>
    end
end
finalTable = cell2table(finalRows, 'VariableNames', {'ActorName', 'FinalPredictedLabel'});

summary.predLog = predLog;
summary.finalTable = finalTable;
summary.numSafe = sum(finalTable.FinalPredictedLabel == 0);
summary.numRisky = sum(finalTable.FinalPredictedLabel == 1);

fprintf('\n--- Live demo finished ---\n');
fprintf('Actors predicted SAFE : %d\n', summary.numSafe);
fprintf('Actors predicted RISKY: %d\n', summary.numRisky);
disp(finalTable);

end

% ---------------------------------------------------------------------
function drawBox2D(ax, pos, yawDeg, len, wid, color, label)
% Draws a top-down colored rectangle at pos with heading yawDeg, plus a
% small triangular "nose" showing which way it's facing, and an optional
% text label above it.
yawRad = deg2rad(yawDeg);
R = [cos(yawRad) -sin(yawRad); sin(yawRad) cos(yawRad)];

corners2D = [ len/2  wid/2; len/2 -wid/2; -len/2 -wid/2; -len/2 wid/2] * R';
x = pos(1) + corners2D(:,1);
y = pos(2) + corners2D(:,2);
patch(ax, 'XData', x, 'YData', y, 'FaceColor', color, ...
    'EdgeColor', [1 1 1], 'LineWidth', 1, 'FaceAlpha', 0.9);

nose2D = [ len/2  0; len*0.30  wid*0.35; len*0.30 -wid*0.35] * R';
nx = pos(1) + nose2D(:,1);
ny = pos(2) + nose2D(:,2);
patch(ax, 'XData', nx, 'YData', ny, 'FaceColor', [1 1 1], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.9);

if nargin >= 7 && ~isempty(label)
    text(ax, pos(1), pos(2) + wid/2 + 1.5, label, ...
        'HorizontalAlignment', 'center', 'FontSize', 8, ...
        'Color', [0.9 0.9 0.9], 'Interpreter', 'none');
end
end