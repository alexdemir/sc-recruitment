function ok = verify_equivalence(varargin)

opt = struct('dtPlant', 1e-3, 'dtCtrl', 0.01, 'tol_pos', 1e-3, ...
             'tol_ang', 1e-4, 'tol_v', 1e-3);
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'src'));
trk = track_autox();
p   = params_vehicle();

names = {'Pure Pursuit','Stanley'};
ctrls = {'pp','stanley'};
ok    = true;

for c = 1:2
    fprintf('\n=== %s ===\n', names{c});

    [ref, kref] = run_reference(trk, p, ctrls{c}, ...
        struct('dtPlant', opt.dtPlant, 'dtCtrl', opt.dtCtrl, 'tMax', 60));

    mdl = build_model(trk, p, 'ctrl', c-1, 'dtPlant', opt.dtPlant, ...
                      'dtCtrl', opt.dtCtrl, 'stopTime', ceil(kref.lapTime)+1);
    out = sim(mdl);
    stl = local_get(out, 'st_log');

    t  = stl.time;
    X  = squeeze(stl.signals.values);
    if size(X,1) == 6, X = X.'; end

    n  = min(numel(ref.t), sum(t <= ref.t(end)));
    tg = ref.t(1:n);
    dx = interp1(t, X(:,1), tg) - ref.x(1:n);
    dy = interp1(t, X(:,2), tg) - ref.y(1:n);
    dp = interp1(t, X(:,3), tg) - ref.psi(1:n);
    dv = interp1(t, X(:,4), tg) - ref.v(1:n);
    dd = interp1(t, X(:,5), tg) - ref.delta(1:n);

    ePos = max(hypot(dx, dy));
    ePsi = max(abs(dp));
    eV   = max(abs(dv));
    eDel = max(abs(dd));

    fprintf('  reference lap time        %.4f s over %d samples\n', kref.lapTime, n);
    fprintf('  max position deviation    %.3e m    (tol %.0e)\n', ePos, opt.tol_pos);
    fprintf('  max heading deviation     %.3e rad  (tol %.0e)\n', ePsi, opt.tol_ang);
    fprintf('  max speed deviation       %.3e m/s  (tol %.0e)\n', eV,   opt.tol_v);
    fprintf('  max steering deviation    %.3e rad  (tol %.0e)\n', eDel, opt.tol_ang);

    pass = ePos < opt.tol_pos && ePsi < opt.tol_ang && eV < opt.tol_v && eDel < opt.tol_ang;
    fprintf('  --> %s\n', local_verdict(pass));
    ok = ok && pass;
end

fprintf('\n%s\n', local_verdict(ok));
end

function v = local_get(out, name)
if isa(out, 'Simulink.SimulationOutput')
    v = out.get(name);
else
    v = evalin('base', name);
end
end

function s = local_verdict(tf)
if tf, s = 'PASS'; else, s = 'FAIL'; end
end
