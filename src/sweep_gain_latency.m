function G = sweep_gain_latency(trk, p, varargin)

opt = struct('dtPlant', 2e-3, 'verbose', true);
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end

G.lat   = [0 0.02 0.05 0.10];
G.st.ke = [1 2 3 4 6 9 14];
G.pp.Ld0 = [2 3 4 5 6];
G.st.score = nan(numel(G.st.ke), numel(G.lat));
G.pp.score = nan(numel(G.pp.Ld0), numel(G.lat));

for a = 1:numel(G.st.ke)
    for b = 1:numel(G.lat)
        q = p;  q.ke = G.st.ke(a);
        G.st.score(a,b) = local_run(trk, q, 'stanley', G.lat(b), opt);
    end
    if opt.verbose
        fprintf('  stanley ke=%5.1f : ', G.st.ke(a));
        fprintf('%9.2f', G.st.score(a,:));  fprintf('\n');  flush_out();
    end
end
for a = 1:numel(G.pp.Ld0)
    for b = 1:numel(G.lat)
        q = p;  q.Ld0 = G.pp.Ld0(a);
        G.pp.score(a,b) = local_run(trk, q, 'pp', G.lat(b), opt);
    end
    if opt.verbose
        fprintf('  pp     Ld0=%4.1f : ', G.pp.Ld0(a));
        fprintf('%9.2f', G.pp.score(a,:));  fprintf('\n');  flush_out();
    end
end
end

function te = local_run(trk, q, ctrl, lat, opt)
[lg, k] = run_reference(trk, q, ctrl, ...
    struct('dtPlant', opt.dtPlant, 'tMax', 60, 'latency', lat));
if lg.completed, te = k.score; else, te = NaN; end
end
