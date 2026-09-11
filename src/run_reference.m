function [log, k] = run_reference(trk, p, ctrl, opts)

if nargin < 4, opts = struct(); end
d.dtPlant = 1e-3;  d.dtCtrl = 1e-2;  d.tMax = 120;  d.speedScale = 1.0;
d.latency = 0;     d.posNoise = 0;   d.seed = 0;    d.standingStart = false;
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end

PX = trk.x;  PY = trk.y;  PPSI = trk.psi;  N = trk.N;
vref = speed_profile(trk.kappa, trk.ds, p) * opts.speedScale;

if opts.standingStart
    st = [PX(1) - 6*cos(PPSI(1)); PY(1) - 6*sin(PPSI(1)); PPSI(1); 0; 0; 0];
else
    st = [PX(1); PY(1); PPSI(1); vref(1); 0; 0];
end
iPrev = 1;

nDelay = max(0, round(opts.latency / opts.dtCtrl));
buf    = repmat(st(1:3)', nDelay+1, 1);
rand_state(opts.seed);

nSub  = max(1, round(opts.dtCtrl / opts.dtPlant));
iTrue = 1;
winT  = 3;
nStep = ceil(opts.tMax / opts.dtPlant);
log.t = zeros(nStep,1);  log.x = zeros(nStep,1);  log.y = zeros(nStep,1);
log.psi = zeros(nStep,1);  log.v = zeros(nStep,1);  log.delta = zeros(nStep,1);
log.deltaCmd = zeros(nStep,1);  log.ey = zeros(nStep,1);  log.vref = zeros(nStep,1);
n = 0;  t = 0;  prog = 0;  deltaCmd = 0;  lapDone = false;

while ~lapDone && n < nStep
    buf  = [buf(2:end,:); st(1:3)'];
    pose = buf(1,:);
    if opts.posNoise > 0
        pose(1:2) = pose(1:2) + opts.posNoise*randn(1,2);
    end

    if strcmp(ctrl, 'pp')
        [deltaCmd, i0] = ctrl_pure_pursuit(pose(1), pose(2), pose(3), st(4), ...
            PX, PY, PPSI, iPrev, trk.ds, p.L, p.Ld0, p.kv, p.searchWin);
    else
        [deltaCmd, i0] = ctrl_stanley(pose(1), pose(2), pose(3), st(4), ...
            PX, PY, PPSI, iPrev, p.L, p.ke, p.kSoft, p.searchWin);
    end

    prog  = prog + mod(i0 - iPrev + N/2, N) - N/2;
    iPrev = i0;

    for sub = 1:nSub
        [iTrue, eyTrue] = path_nearest(st(1), st(2), PX, PY, PPSI, iTrue, winT);
        n = n + 1;
        log.t(n) = t;         log.x(n) = st(1);    log.y(n) = st(2);
        log.psi(n) = st(3);   log.v(n) = st(4);    log.delta(n) = st(5);
        log.deltaCmd(n) = deltaCmd;  log.ey(n) = eyTrue;  log.vref(n) = vref(i0);

        st = rk4_step(st, deltaCmd, vref(i0), p, opts.dtPlant);
        t  = t + opts.dtPlant;

        if prog >= N - 1 && n > nSub
            lapDone = true;  break;
        end
    end
end

fn = fieldnames(log);
for i = 1:numel(fn), log.(fn{i}) = log.(fn{i})(1:n); end
log.completed = lapDone;
log.ctrl      = ctrl;

if nargout > 1, k = kpi_compute(log, trk, p); end
end

function st = rk4_step(st, deltaCmd, vref, p, dt)
k1 = vehicle_ode(st,             deltaCmd, vref, p);
k2 = vehicle_ode(st + dt/2*k1,   deltaCmd, vref, p);
k3 = vehicle_ode(st + dt/2*k2,   deltaCmd, vref, p);
k4 = vehicle_ode(st + dt*k3,     deltaCmd, vref, p);
st = st + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

function rand_state(seed)
if exist('rng', 'file') || exist('rng', 'builtin')
    rng(seed);
else
    randn('seed', seed);
end
end
