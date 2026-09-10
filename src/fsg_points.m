function pts = fsg_points(tEff, lapLen, tMin, pMax)
%FSG_POINTS  Driverless Cup Autocross score, Formula Student Rules 2026 D 9.3.2.
%
%   pts = FSG_POINTS(tEff, lapLen, tMin, pMax)
%
%       AUTOCROSS_SCORE = 0.9*pMax * (tMax - tTeam)/(tMax - tMin) + 0.1*pMax
%
%   with tMax the time for driving the lap at 6 m/s, tTeam the team's time
%   including penalties, and tMin the fastest such time in the comparison set.
%   pMax is 100 points for DV Autocross (rules table 3).
%
%   Turning the KPIs into the score the rules actually award is what makes the
%   final verdict of the report a competition statement rather than a
%   preference: a controller that is a little slower but knocks no cones can
%   still win, exactly as it would at the event.

if nargin < 4, pMax = 100; end
tMax = lapLen / 6;                       % D 9.3.2: lap at 6 m/s
tEff = min(tEff, tMax);                  % times at or beyond tMax score pMin
pts  = 0.9*pMax * (tMax - tEff)/(tMax - tMin) + 0.1*pMax;
end
