function k = kpi_compute(log, trk, p)
t  = log.t;
ey = log.ey;

k.lapTime = t(end) - t(1);
k.eyMax   = max(abs(ey));
k.eyRms   = sqrt(mean(ey.^2));

dDelta     = diff(log.delta) ./ diff(t);
k.dRateRms = rad2deg(sqrt(mean(dDelta.^2)));

cones = [trk.coneL; trk.coneR; trk.coneStart];
xc    = log.x + p.bodyCtrOff*cos(log.psi);
yc    = log.y + p.bodyCtrOff*sin(log.psi);
cs    = cos(log.psi);
sn    = sin(log.psi);
halfL = p.bodyLen/2 + p.coneR;
halfW = p.bodyWid/2 + p.coneR;

hit = false(size(cones,1),1);
for i = 1:size(cones,1)
    dx = cones(i,1) - xc;
    dy = cones(i,2) - yc;
    xb =  cs.*dx + sn.*dy;
    yb = -sn.*dx + cs.*dy;
    hit(i) = any(abs(xb) <= halfL & abs(yb) <= halfW);
end
k.doo = sum(hit);

outside = abs(ey) > trk.width/2 + p.bodyWid/2;
k.oc    = sum(diff([false; outside(:)]) == 1);

k.score = k.lapTime + 2*k.doo + 10*k.oc;
end
