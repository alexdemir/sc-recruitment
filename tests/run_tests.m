function nFail = run_tests()
addpath('src'); addpath('tests');
nFail = 0;
fprintf('\n== track ==\n');
nFail = nFail + t_track();
fprintf('\n== steering laws ==\n');
nFail = nFail + t_zero_error();
nFail = nFail + t_pp_ackermann();
nFail = nFail + t_stanley_circle();
nFail = nFail + t_straight_convergence();
nFail = nFail + t_mirror_symmetry();
if nFail == 0, verdict = 'ALL TESTS PASSED'; else, verdict = 'TESTS FAILED'; end
fprintf('\n%s  (%d failure(s))\n', verdict, nFail);
end

function f = chk(name, cond, fmt, varargin)
f = ~cond;
if cond, st = 'PASS'; else, st = 'FAIL'; end
fprintf('  [%s] %-34s ', st, name);
fprintf(fmt, varargin{:});
fprintf('\n');
end

function f = t_track()
trk = track_autox();  s = trk.stats;  f = 0;
f = f + chk('lap length 200-500 m (D 8.1.2)', s.lapLen>200 && s.lapLen<500, '%.1f m', s.lapLen);
f = f + chk('min turning diameter >= 9 m', 2*s.minRadius >= 9, '%.2f m dia', 2*s.minRadius);
f = f + chk('straights <= 80 m (D 8.1.1)', s.maxStraight <= 80, '%.1f m', s.maxStraight);
f = f + chk('track width >= 3 m (D 8.1.1)', s.width >= 3, '%.2f m', s.width);
f = f + chk('closed loop: total turn 360 deg', abs(abs(s.turnDegrees)-360) < 1e-6, '%.4f deg', s.turnDegrees);
f = f + chk('closure error < 1 mm', abs(s.closureError) < 1e-3, '%.2e m', s.closureError);
dpsi = abs(wrap_pi(diff(trk.psi)));
f = f + chk('heading continuous', max(dpsi) < 1.5*trk.ds/s.minRadius, 'max step %.4f rad', max(dpsi));
kNZ = abs(trk.kappa(abs(trk.kappa) > 1e-9));
f = f + chk('curvature matches fillet radii', all(min(abs(1./kNZ - trk.stats.radii'),[],2) < 1e-9), '%d arc samples', numel(kNZ));
end

function f = t_zero_error()
p = params_vehicle();  pth = mk_path('straight', 0);  i0 = 200;
[dpp, ~] = ctrl_pure_pursuit(pth.x(i0), pth.y(i0), 0, 10, pth.x, pth.y, pth.psi, i0, pth.ds, p.L, p.Ld0, p.kv, p.searchWin);
[dst, ~] = ctrl_stanley     (pth.x(i0), pth.y(i0), 0, 10, pth.x, pth.y, pth.psi, i0, p.L, p.ke, p.kSoft, p.searchWin);
f = chk('pure pursuit: delta = 0', abs(dpp) < 1e-12, '%.2e rad', dpp) + ...
    chk('stanley: delta = 0',      abs(dst) < 1e-12, '%.2e rad', dst);
end

function f = t_pp_ackermann()
p = params_vehicle();  f = 0;
for R = [8 15 30 60]
    pth = mk_path('circle', R);  i0 = 400;
    dExp = atan(p.L/R);
    dGot = ctrl_pure_pursuit(pth.x(i0), pth.y(i0), pth.psi(i0), 10, ...
        pth.x, pth.y, pth.psi, i0, pth.ds, p.L, p.Ld0, p.kv, p.searchWin);
    f = f + chk(sprintf('PP = Ackermann at R = %d m', R), abs(dGot-dExp) < 1e-4, ...
                'got %.5f, exact %.5f rad', dGot, dExp);
end
end

function f = t_stanley_circle()
p = params_vehicle();  f = 0;
for R = [15 30]
    pth = mk_path('circle', R);  i0 = 400;
    dAck = atan(p.L/R);
    dGot = ctrl_stanley(pth.x(i0), pth.y(i0), pth.psi(i0), 10, ...
        pth.x, pth.y, pth.psi, i0, p.L, p.ke, p.kSoft, p.searchWin);
    f = f + chk(sprintf('Stanley near Ackermann at R = %d', R), ...
                abs(dGot-dAck) < deg2rad(3), 'got %.4f, Ackermann %.4f rad', dGot, dAck);
end
end

function f = t_straight_convergence()
p = params_vehicle();  f = 0;
trk = struct('x', [], 'y', [], 'psi', [], 'kappa', [], 's', [], 'ds', 0.25, 'N', 4000);
pth = mk_path('straight', 0);
fn = {'x','y','psi','kappa','s'};
for i=1:numel(fn), trk.(fn{i}) = pth.(fn{i}); end
trk.width = 3.5;  trk.coneL = zeros(0,2);  trk.coneR = zeros(0,2);  trk.coneStart = zeros(0,2);
for c = {'pp','stanley'}
    ey = local_offset_run(trk, p, c{1}, 1.0);
    f = f + chk(sprintf('%s: converges from 1 m offset', c{1}), abs(ey(end)) < 0.01, 'final %.4f m', ey(end));
    f = f + chk(sprintf('%s: overshoot bounded', c{1}), min(ey) > -1.0, 'worst %.4f m', min(ey));
end
end

function f = t_mirror_symmetry()
p = params_vehicle();  trk = track_autox();  f = 0;
mir = trk;
mir.x = trk.x;  mir.y = -trk.y;  mir.psi = -trk.psi;  mir.kappa = -trk.kappa;
mir.coneL = [trk.coneR(:,1), -trk.coneR(:,2)];
mir.coneR = [trk.coneL(:,1), -trk.coneL(:,2)];
mir.coneStart = [trk.coneStart(:,1), -trk.coneStart(:,2)];
for c = {'pp','stanley'}
    [~, k1] = run_reference(trk, p, c{1}, struct('tMax',40));
    [~, k2] = run_reference(mir, p, c{1}, struct('tMax',40));
    f = f + chk(sprintf('%s: mirrored lap time equal', c{1}), abs(k1.lapTime-k2.lapTime) < 1e-6, ...
                '%.6f vs %.6f s', k1.lapTime, k2.lapTime);
    f = f + chk(sprintf('%s: mirrored eyRms equal', c{1}), abs(k1.eyRms-k2.eyRms) < 1e-9, ...
                '%.6f vs %.6f m', k1.eyRms, k2.eyRms);
end
end

function ey = local_offset_run(trk, p, ctrl, y0)
dt = 1e-3;  nSub = 10;  st = [0; y0; 0; 10; 0; 0];  iPrev = 1;  ey = [];
for it = 1:400
    if strcmp(ctrl,'pp')
        [dc, i0] = ctrl_pure_pursuit(st(1), st(2), st(3), st(4), trk.x, trk.y, trk.psi, iPrev, trk.ds, p.L, p.Ld0, p.kv, p.searchWin);
    else
        [dc, i0] = ctrl_stanley(st(1), st(2), st(3), st(4), trk.x, trk.y, trk.psi, iPrev, p.L, p.ke, p.kSoft, p.searchWin);
    end
    iPrev = i0;
    for sub = 1:nSub
        k1 = vehicle_ode(st, dc, 10, p);           k2 = vehicle_ode(st+dt/2*k1, dc, 10, p);
        k3 = vehicle_ode(st+dt/2*k2, dc, 10, p);   k4 = vehicle_ode(st+dt*k3, dc, 10, p);
        st = st + dt/6*(k1+2*k2+2*k3+k4);
    end
    ey(end+1,1) = st(2);
end
end
