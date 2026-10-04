function wp = buildChainedEgoRoute(legs)
%BUILDCHAINEDEGOROUTE Builds a long route by chaining multiple legs.
% legs: struct array, each with fields:
%   straightLength - meters to drive straight before the turn (can be 0)
%   turnAngleDeg   - signed turn angle in degrees: +90 = turn left,
%                    -90 = turn right, +/-180 = U-turn, 0 = no turn
%
% Route starts at [0 0], heading due +x (0 degrees). Uses the standard
% constant-curvature (unicycle) path formula, which is well-behaved for
% any sequence/combination of turns and never produces a malformed path.

R = 20; % turn radius (m)
pos = [0 0];
heading = 0; % radians

wp = [0 0 0];

for i = 1:numel(legs)
    L = legs(i).straightLength;
    if L > 0
        newPos = pos + L * [cos(heading), sin(heading)];
        wp = [wp; newPos, 0]; %#ok<AGROW>
        pos = newPos;
    end

    angDeg = legs(i).turnAngleDeg;
    if abs(angDeg) > 1e-6
        ang = deg2rad(angDeg);
        kappa = sign(ang) / R; % +1/R = left turn, -1/R = right turn
        s = linspace(0, R * abs(ang), 12)';
        psi = heading + kappa .* s;
        x = pos(1) + (1/kappa) * (sin(psi) - sin(heading));
        y = pos(2) - (1/kappa) * (cos(psi) - cos(heading));
        wp = [wp; x, y, zeros(numel(s), 1)]; %#ok<AGROW>
        pos = [x(end), y(end)];
        heading = heading + ang;
    end
end
end
