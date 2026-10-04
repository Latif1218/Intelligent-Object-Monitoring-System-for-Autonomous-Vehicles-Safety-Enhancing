function visualizeScenario(scenario)
%VISUALIZESCENARIO Plays back the scenario visually in a bird's-eye view.
% Usage:
%   [scenario, gtTable, meta] = generateScenario(5, 1005);
%   visualizeScenario(scenario);
%
% For an even richer 3D view, you can instead open the scenario in the
% Driving Scenario Designer app:
%   drivingScenarioDesigner(scenario)

restart(scenario);
plot(scenario);
title(sprintf('Scenario playback (%d actors)', numel(scenario.Actors)));

while advance(scenario)
    pause(scenario.SampleTime); % real-time-ish playback
end

disp('Playback finished.');
end
