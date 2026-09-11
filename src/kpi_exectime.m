function E = kpi_exectime(trk, p, nCalls)
%KPI_EXECTIME  Cost of one control step, and the rate it implies.
%
%   E = KPI_EXECTIME(trk, p)
%
%   A sixth KPI that matters only in a driverless context: a steering law that
%   cannot be evaluated inside the control period is unusable whatever its
%   tracking error. Both laws here are a handful of trigonometric operations
%   plus a bounded nearest-point search, so both are cheap - but "cheap" should
%   be a measured number in the report rather than an assumption, and the ratio
%   between them is a real difference.
%
%   Timed on this desktop, so the absolute figures are not the vehicle's ECU.
%   What transfers is the ratio and the order of magnitude: at tens of
%   microseconds per call, a 100 Hz loop spends well under 1 % of its period in
%   the steering law, and the margin to the target rate is three orders of
%   magnitude.

if nargin < 3, nCalls = 20000; end

% a representative operating point: mid-corner, moving, slightly off the line
i0 = round(trk.N * 0.35);
x  = trk.x(i0) + 0.3*(-sin(trk.psi(i0)));
y  = trk.y(i0) + 0.3*( cos(trk.psi(i0)));
ps = trk.psi(i0) - 0.05;
v  = 11.0;

% warm up, so the first-call overhead is not what gets measured
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
E.ppDuty   = E.ppUs / 100;       % percent of a 10 ms control period
E.stDuty   = E.stUs / 100;
end
