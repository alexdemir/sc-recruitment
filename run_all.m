function R = run_all(varargin)

opt = struct('retune', false, 'quick', false, 'outdir', 'figures', 'verbose', false);
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'src'));
addpath(fullfile(here,'tests'));

dt  = 1e-3;  if opt.quick, dt = 4e-3; end
trk = track_autox();
p   = params_vehicle();

fprintf('\n=== track ===\n');
fprintf('lap %.1f m | min radius %.2f m | longest straight %.1f m | width %.2f m | closure %.1e m\n', ...
    trk.stats.lapLen, trk.stats.minRadius, trk.stats.maxStraight, trk.width, trk.stats.closureError);

tuneFile = fullfile(here, 'results_tuning.mat');
if opt.retune || ~exist(tuneFile, 'file')
    fprintf('\n=== tuning (grid search, same objective for both) ===\n');
    T = tune_gains(trk, p, 'verbose', true);
    save('-mat', tuneFile, 'T');
else
    fprintf('\n=== tuning: loaded %s ===\n', tuneFile);
    S = load(tuneFile);  T = S.T;
end
fprintf('pure pursuit : Ld0 = %.1f m, kv = %.2f s   (J = %.3f s)\n', T.pp.best.Ld0, T.pp.best.kv, T.pp.best.J);
fprintf('stanley      : ke = %.1f 1/s, kSoft = %.2f m/s (J = %.3f s)\n', T.st.best.ke, T.st.best.kSoft, T.st.best.J);


pPP = p;  pPP.Ld0 = T.pp.best.Ld0;  pPP.kv = T.pp.best.kv;
pST = p;  pST.ke  = T.st.best.ke;   pST.kSoft = T.st.best.kSoft;
pars = {pPP, pST};  ctrls = {'pp','stanley'};  names = {'Pure Pursuit','Stanley'};

fprintf('\n=== nominal lap ===\n');
for i = 1:2
    [lg, k] = run_reference(trk, pars{i}, ctrls{i}, struct('dtPlant', dt, 'tMax', 60));
    res(i) = struct('name', names{i}, 'log', lg, 'kpi', k);
end
local_table(res, trk);

if opt.verbose, fprintf('\n=== robustness sweeps ===\n'); end
S.speed.x   = [0.90 1.00 1.10 1.20 1.30];
S.latency.x = [0 0.02 0.05 0.10];
S.noise.x   = [0 0.02 0.05 0.10];
for f = {'speed','latency','noise'}
    fn = f{1};
    S.(fn).pp = zeros(size(S.(fn).x));
    S.(fn).st = zeros(size(S.(fn).x));
end
for j = 1:numel(S.speed.x)
    for i = 1:2
        o = struct('dtPlant', dt, 'tMax', 60, 'speedScale', S.speed.x(j));
        S.speed.(local_key(i))(j) = local_teff(trk, pars{i}, ctrls{i}, o);
    end
    if opt.verbose
        fprintf('  speed x%.2f : PP %6.2f s   ST %6.2f s\n', S.speed.x(j), S.speed.pp(j), S.speed.st(j));
    end
end
for j = 1:numel(S.latency.x)
    for i = 1:2
        o = struct('dtPlant', dt, 'tMax', 60, 'latency', S.latency.x(j));
        S.latency.(local_key(i))(j) = local_teff(trk, pars{i}, ctrls{i}, o);
    end
    if opt.verbose
        fprintf('  latency %3.0f ms : PP %6.2f s   ST %6.2f s\n', S.latency.x(j)*1e3, S.latency.pp(j), S.latency.st(j));
    end
end
for j = 1:numel(S.noise.x)
    for i = 1:2
        o = struct('dtPlant', dt, 'tMax', 60, 'posNoise', S.noise.x(j), 'seed', 7);
        S.noise.(local_key(i))(j) = local_teff(trk, pars{i}, ctrls{i}, o);
    end
    if opt.verbose
        fprintf('  noise %4.0f cm : PP %6.2f s   ST %6.2f s\n', S.noise.x(j)*100, S.noise.pp(j), S.noise.st(j));
    end
end

if opt.verbose, fprintf('\n=== lookahead sweep: corner cut vs. lap time ===\n'); end
pBoth = pPP;  pBoth.ke = T.st.best.ke;  pBoth.kSoft = T.st.best.kSoft;
Lk = sweep_lookahead(trk, pBoth, 'verbose', opt.verbose);

if opt.verbose, fprintf('\n=== track width sweep (D 8.1.1 allows 3 m) ===\n'); end
Wd = sweep_width(p, T, 'dtPlant', dt, 'verbose', opt.verbose);

if opt.verbose, fprintf('\n=== gain vs. latency maps ===\n'); end
G = sweep_gain_latency(trk, p, 'verbose', false);

if opt.verbose, fprintf('\n=== figures ===\n'); end
plot_summary(trk, p, res, opt.outdir);
plot_report_figure(trk, res, opt.outdir);
plot_results(trk, p, res, opt.outdir);
plot_tuning(T, opt.outdir);
plot_robustness(S, opt.outdir);
plot_width(Wd, opt.outdir);
plot_gain_latency(G, T, opt.outdir);
fprintf('\nfigures written to %s/  (summary: fig11_summary.png)\n', opt.outdir);

R = struct('trk', trk, 'p', p, 'tuning', T, 'res', res, ...
           'sweeps', S, 'width', Wd, 'gainLatency', G, 'lookahead', Lk);
save('-mat', fullfile(here,'results.mat'), 'R');
end

function k = local_key(i)
if i == 1, k = 'pp'; else, k = 'st'; end
end

function te = local_teff(trk, q, ctrl, o)
[lg, k] = run_reference(trk, q, ctrl, o);
if lg.completed, te = k.score; else, te = NaN; end
end

function local_table(res, trk)
fprintf('\n%-24s %10s %10s\n', 'KPI', res(1).name, res(2).name);
fprintf('%s\n', repmat('-', 1, 46));
rows = { 'lap time            [s]', 'lapTime', '%10.3f'
         'max cross-track     [m]', 'eyMax',   '%10.3f'
         'RMS cross-track     [m]', 'eyRms',   '%10.4f'
         'RMS steering rate [d/s]', 'dRateRms','%10.2f'
         'cones down or out   [-]', 'doo',     '%10d' };
for r = 1:size(rows,1)
    fprintf('%-24s', rows{r,1});
    fprintf(rows{r,3}, res(1).kpi.(rows{r,2}));
    fprintf(rows{r,3}, res(2).kpi.(rows{r,2}));
    fprintf('\n');
end
fprintf('\n(%.1f m wide track; the narrower-track runs are in R.width)\n', trk.width);
end
