function k = kpi_compute(log, trk, p)
%KPI_COMPUTE  Key performance indicators of one Autocross lap.
%
%   k = KPI_COMPUTE(log, trk, p)
%
%   KPIs
%     lapTime    [s]      time to complete one lap
%     eyMax      [m]      maximum absolute cross-track error
%     eyRms      [m]      RMS cross-track error
%     dRateRms   [deg/s]  RMS steering rate: actuator load and ride smoothness
%     doo        [-]      cones Down or Out
%     oc         [-]      Off Course events
%
%   Competition metric
%     tEff = lapTime + 2*doo + 10*oc
%   from the Autocross penalty table of the Formula Student Rules 2026,
%   D 10.1.7 (DOO 2 s, OC 10 s). A cone is counted at most once, matching
%   D 10.1.3: cones knocked down during an autonomous run are not reset.
%
%   The competition points that follow from tEff are computed separately in
%   fsg_points.m, because they depend on the best time of the comparison set.

t  = log.t;
ey = log.ey;

% --- tracking accuracy ---------------------------------------------------
k.lapTime = t(end) - t(1);
k.eyMax   = max(abs(ey));
k.eyRms   = sqrt(mean(ey.^2));

% --- steering activity ---------------------------------------------------
dDelta      = diff(log.delta) ./ diff(t);
k.dRateRms  = rad2deg(sqrt(mean(dDelta.^2)));
k.dMaxUsed  = rad2deg(max(abs(log.delta)));

% --- cones down or out ---------------------------------------------------
cones = [trk.coneL; trk.coneR; trk.coneStart];
xc    = log.x + p.bodyCtrOff*cos(log.psi);      % body centre path
yc    = log.y + p.bodyCtrOff*sin(log.psi);
cs    = cos(log.psi);
sn    = sin(log.psi);
halfL = p.bodyLen/2 + p.coneR;
halfW = p.bodyWid/2 + p.coneR;

hit = false(size(cones,1),1);
for i = 1:size(cones,1)
    dx = cones(i,1) - xc;
    dy = cones(i,2) - yc;
    xb =  cs.*dx + sn.*dy;                      % cone in body frame
    yb = -sn.*dx + cs.*dy;
    hit(i) = any(abs(xb) <= halfL & abs(yb) <= halfW);
end
k.doo      = sum(hit);
k.coneHits = cones(hit,:);

% --- off course ----------------------------------------------------------
% "all four wheels outside the track boundary" (D 10.1.5) is approximated by
% the whole body being clear of the corridor: |ey| - bodyWid/2 > width/2.
outside = abs(ey) > trk.width/2 + p.bodyWid/2;
k.oc    = sum(diff([false; outside(:)]) == 1);   % count entries, not samples

% --- competition time ----------------------------------------------------
k.tEff = k.lapTime + 2*k.doo + 10*k.oc;
end
