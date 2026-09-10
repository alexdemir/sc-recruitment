function R = run_all(varargin)
%RUN_ALL  Full comparison: tune, run, sweep, plot, tabulate.
%
%   R = RUN_ALL()              uses results_tuning.mat if it exists
%   R = RUN_ALL('retune',true) re-runs the gain sweep (a few minutes)
%   R = RUN_ALL('quick',true)  coarser integration, for a fast sanity pass
%
%   Runs in MATLAB and in GNU Octave. Everything downstream of this script -
%   the KPI table, the figures and the numbers quoted in report/REPORT.md - is
%   produced here, so the report cannot drift from the code.

opt = struct('retune', false, 'quick', false, 'outdir', 'figures');
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

% ---- gains --------------------------------------------------------------
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
fprintf('stanley      : ke = %.1f 1/s, kSoft = %.1f m/s (J = %.3f s)\n', T.st.best.ke, T.st.best.kSoft, T.st.best.J);

pPP = p;  pPP.Ld0 = T.pp.best.Ld0;  pPP.kv = T.pp.best.kv;
pST = p;  pST.ke  = T.st.best.ke;   pST.kSoft = T.st.best.kSoft;
pars = {pPP, pST};  ctrls = {'pp','stanley'};  names = {'Pure Pursuit','Stanley'};

% ---- nominal lap --------------------------------------------------------
fprintf('\n=== nominal lap ===\n');
for i = 1:2
    [lg, k] = run_reference(trk, pars{i}, ctrls{i}, struct('dtPlant', dt, 'tMax', 60));
    res(i) = struct('name', names{i}, 'log', lg, 'kpi', k);
end
tMin = min([res(1).kpi.tEff, res(2).kpi.tEff]);
for i = 1:2
    res(i).kpi.points = fsg_points(res(i).kpi.tEff, trk.lapLen, tMin);
end
local_table(res, trk);

% ---- robustness sweeps --------------------------------------------------
fprintf('\n=== robustness sweeps ===\n');
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
    fprintf('  speed x%.2f : PP %6.2f s   ST %6.2f s\n', S.speed.x(j), S.speed.pp(j), S.speed.st(j));
end
for j = 1:numel(S.latency.x)
    for i = 1:2
        o = struct('dtPlant', dt, 'tMax', 60, 'latency', S.latency.x(j));
        S.latency.(local_key(i))(j) = local_teff(trk, pars{i}, ctrls{i}, o);
    end
    fprintf('  latency %3.0f ms : PP %6.2f s   ST %6.2f s\n', S.latency.x(j)*1e3, S.latency.pp(j), S.latency.st(j));
end
for j = 1:numel(S.noise.x)
    for i = 1:2
        o = struct('dtPlant', dt, 'tMax', 60, 'posNoise', S.noise.x(j), 'seed', 7);
        S.noise.(local_key(i))(j) = local_teff(trk, pars{i}, ctrls{i}, o);
    end
    fprintf('  noise %4.0f cm : PP %6.2f s   ST %6.2f s\n', S.noise.x(j)*100, S.noise.pp(j), S.noise.st(j));
end

% ---- figures and results ------------------------------------------------
fprintf('\n=== figures ===\n');
plot_results(trk, p, res, opt.outdir);
plot_tuning(T, opt.outdir);
plot_robustness(S, opt.outdir);
fprintf('written to %s/\n', opt.outdir);

R = struct('trk', trk, 'p', p, 'tuning', T, 'res', res, 'sweeps', S);
save('-mat', fullfile(here,'results.mat'), 'R');
local_markdown(res, trk, T, S, fullfile(here,'report','kpi_tables.md'));
end

% =========================================================================
function k = local_key(i)
if i == 1, k = 'pp'; else, k = 'st'; end
end

% =========================================================================
function te = local_teff(trk, q, ctrl, o)
[lg, k] = run_reference(trk, q, ctrl, o);
if lg.completed, te = k.tEff; else, te = NaN; end
end

% =========================================================================
function local_table(res, trk)
fprintf('\n%-24s %10s %10s\n', 'KPI', res(1).name, res(2).name);
fprintf('%s\n', repmat('-', 1, 46));
rows = { 'lap time            [s]', 'lapTime', '%10.3f'
         'max cross-track     [m]', 'eyMax',   '%10.3f'
         'RMS cross-track     [m]', 'eyRms',   '%10.4f'
         'RMS steering rate [d/s]', 'dRateRms','%10.2f'
         'peak steering     [deg]', 'dMaxUsed','%10.2f'
         'cones down/out      [-]', 'doo',     '%10d'
         'off course          [-]', 'oc',      '%10d'
         'effective time      [s]', 'tEff',    '%10.3f'
         'FSG DV points       [-]', 'points',  '%10.2f' };
for r = 1:size(rows,1)
    fprintf('%-24s', rows{r,1});
    fprintf(rows{r,3}, res(1).kpi.(rows{r,2}));
    fprintf(rows{r,3}, res(2).kpi.(rows{r,2}));
    fprintf('\n');
end
fprintf('\n(effective time = lap + 2 s per cone + 10 s per off-course, FS Rules 2026 D 10.1.7;\n');
fprintf(' points per D 9.3.2 with tMax = lap at 6 m/s = %.2f s)\n', trk.lapLen/6);
end

% =========================================================================
function local_markdown(res, trk, T, S, fname)
%LOCAL_MARKDOWN  Emit the KPI tables the report includes, so the prose and the
%   numbers cannot disagree.
d = fileparts(fname);
if ~exist(d,'dir'), mkdir(d); end
fid = fopen(fname, 'w');
fprintf(fid, '<!-- generated by run_all.m - do not edit by hand -->\n\n');
fprintf(fid, '### Track\n\n');
fprintf(fid, '| property | value | rule |\n|---|---|---|\n');
fprintf(fid, '| lap length | %.1f m | D 8.1.2: 200-500 m |\n', trk.stats.lapLen);
fprintf(fid, '| minimum turning diameter | %.2f m | D 8.1.1: >= 9 m |\n', 2*trk.stats.minRadius);
fprintf(fid, '| longest straight | %.1f m | D 8.1.1: <= 80 m |\n', trk.stats.maxStraight);
fprintf(fid, '| track width | %.2f m | D 8.1.1: >= 3 m |\n', trk.width);
fprintf(fid, '| closure error | %.1e m | closed loop |\n\n', trk.stats.closureError);

fprintf(fid, '### Tuned gains\n\n');
fprintf(fid, '| controller | gains | objective J |\n|---|---|---|\n');
fprintf(fid, '| Pure Pursuit | Ld0 = %.1f m, kv = %.2f s | %.3f s |\n', T.pp.best.Ld0, T.pp.best.kv, T.pp.best.J);
fprintf(fid, '| Stanley | ke = %.1f 1/s, ksoft = %.1f m/s | %.3f s |\n\n', T.st.best.ke, T.st.best.kSoft, T.st.best.J);

fprintf(fid, '### KPIs, nominal lap\n\n');
fprintf(fid, '| KPI | %s | %s |\n|---|---|---|\n', res(1).name, res(2).name);
f = { 'lap time [s]','lapTime','%.3f'; 'max cross-track error [m]','eyMax','%.3f';
      'RMS cross-track error [m]','eyRms','%.4f'; 'RMS steering rate [deg/s]','dRateRms','%.2f';
      'peak steering [deg]','dMaxUsed','%.2f'; 'cones Down or Out','doo','%d';
      'off-course events','oc','%d'; 'effective time [s]','tEff','%.3f';
      'FSG DV Autocross points','points','%.2f' };
for r = 1:size(f,1)
    fprintf(fid, '| %s | ', f{r,1});
    fprintf(fid, f{r,3}, res(1).kpi.(f{r,2}));  fprintf(fid, ' | ');
    fprintf(fid, f{r,3}, res(2).kpi.(f{r,2}));  fprintf(fid, ' |\n');
end

fprintf(fid, '\n### Robustness, effective time [s]\n\n');
fprintf(fid, '| stressor | value | Pure Pursuit | Stanley |\n|---|---|---|---|\n');
for j = 1:numel(S.speed.x)
    fprintf(fid, '| speed scale | x%.2f | %.2f | %.2f |\n', S.speed.x(j), S.speed.pp(j), S.speed.st(j));
end
for j = 1:numel(S.latency.x)
    fprintf(fid, '| pose latency | %.0f ms | %.2f | %.2f |\n', S.latency.x(j)*1e3, S.latency.pp(j), S.latency.st(j));
end
for j = 1:numel(S.noise.x)
    fprintf(fid, '| position noise | %.0f cm | %.2f | %.2f |\n', S.noise.x(j)*100, S.noise.pp(j), S.noise.st(j));
end
fclose(fid);
end
