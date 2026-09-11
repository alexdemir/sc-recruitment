function E = kpi_exectime(trk, p, nCalls)

if nargin < 3, nCalls = 20000; end

i0 = round(trk.N * 0.35);
x  = trk.x(i0) + 0.3*(-sin(trk.psi(i0)));
y  = trk.y(i0) + 0.3*( cos(trk.psi(i0)));
ps = trk.psi(i0) - 0.05;
v  = 11.0;

for k = 1:50
    ctrl_pure_pursuit(x, y, ps, v, trk.x, trk.y, trk.psi, i0, trk.ds, p.L, p.Ld0, p.kv, p.searchWin);
    ctrl_stanley(x, y, ps, v, trk.x, trk.y, trk.psi, i0, p.L, p.ke, p.kSoft, p.searchWin);
end

t0 = tic;
for k = 1:nCalls
    ctrl_pure_pursuit(x, y, ps, v, trk.x, trk.y, trk.psi, i0, trk.ds, p.L, p.Ld0, p.kv, p.searchWin);
end
E.ppUs = toc(t0) / nCalls * 1e6;

t0 = tic;
for k = 1:nCalls
    ctrl_stanley(x, y, ps, v, trk.x, trk.y, trk.psi, i0, p.L, p.ke, p.kSoft, p.searchWin);
end
E.stUs = toc(t0) / nCalls * 1e6;

E.nCalls   = nCalls;
E.ppMaxHz  = 1e6 / E.ppUs;
E.stMaxHz  = 1e6 / E.stUs;
E.ppDuty   = E.ppUs / 100;
E.stDuty   = E.stUs / 100;
end
