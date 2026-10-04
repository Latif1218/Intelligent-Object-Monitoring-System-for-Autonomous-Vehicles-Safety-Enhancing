function matchTable = matchTracksToActors(trackLog, gtLog)
%MATCHTRACKSTOACTORS For each TrackID in trackLog, finds the ground-truth
% actor (from gtLog) whose position is closest on average during their
% overlapping time window. Returns TrackID, MatchedActor, AvgMatchDistance.

trackIDs = unique(trackLog.TrackID);
actorNames = unique(gtLog.ActorName);

rows = {};
for i = 1:numel(trackIDs)
    tid = trackIDs(i);
    tk = trackLog(trackLog.TrackID == tid, :);

    bestActor = '';
    bestDist = inf;
    for j = 1:numel(actorNames)
        aName = actorNames{j};
        gt = gtLog(strcmp(gtLog.ActorName, aName), :);
        if height(gt) < 2
            continue
        end
        gx = interp1(gt.Time, gt.X, tk.Time, 'linear', 'extrap');
        gy = interp1(gt.Time, gt.Y, tk.Time, 'linear', 'extrap');
        d = sqrt((tk.X - gx).^2 + (tk.Y - gy).^2);
        avgD = mean(d, 'omitnan');

        if avgD < bestDist
            bestDist = avgD;
            bestActor = aName;
        end
    end
    rows(end+1, :) = {tid, bestActor, bestDist}; %#ok<AGROW>
end

matchTable = cell2table(rows, 'VariableNames', {'TrackID', 'MatchedActor', 'AvgMatchDistance'});
end
