function T = tune_gains(trk, p, varargin)

DNF_PENALTY = 1e3;

opt = struct('dtPlant', 2e-3, 'verbose', true);
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end

conds = { struct('speedScale',1.00) , ...
          struct('speedScale',1.15) , ...
          struct('speedScale',1.30) , ...
          struct('latency',0.02)    , ...
          struct('latency',0.10)    };

T.pp.Ld0 = [2 3 4 5 6 8];
T.pp.kv  = [0.05 0.10 0.20 0.30 0.40 0.50];
T.pp.J   = nan(numel(T.pp.Ld0), numel(T.pp.kv));
for a = 1:numel(T.pp.Ld0)
    for b = 1:numel(T.pp.kv)
        q = p;  q.Ld0 = T.pp.Ld0(a);  q.kv = T.pp.kv(b);
        T.pp.J(a,b) = local_score(trk, q, 'pp', conds, opt, DNF_PENALTY);
        if opt.verbose
            fprintf('  pp   Ld0=%.1f kv=%.2f  J=%8.3f\n', q.Ld0, q.kv, T.pp.J(a,b));
            flush_out();
        end
    end
end
[~, i1] = min(T.pp.J(:));
[a, b]  = ind2sub(size(T.pp.J), i1);
T.pp.best = struct('Ld0', T.pp.Ld0(a), 'kv', T.pp.kv(b), 'J', T.pp.J(a,b));

T.st.ke    = [1 2 3 4 6 9 14 20];
T.st.kSoft = [0.25 0.5 1.0 2.0 4.0];
T.st.J     = nan(numel(T.st.ke), numel(T.st.kSoft));
for a = 1:numel(T.st.ke)
    for b = 1:numel(T.st.kSoft)
        q = p;  q.ke = T.st.ke(a);  q.kSoft = T.st.kSoft(b);
        T.st.J(a,b) = local_score(trk, q, 'stanley', conds, opt, DNF_PENALTY);
        if opt.verbose
            fprintf('  st   ke=%.1f kSoft=%.1f  J=%8.3f\n', q.ke, q.kSoft, T.st.J(a,b));
            flush_out();
        end
    end
end
[~, i1] = min(T.st.J(:));
[a, b]  = ind2sub(size(T.st.J), i1);
T.st.best = struct('ke', T.st.ke(a), 'kSoft', T.st.kSoft(b), 'J', T.st.J(a,b));

T.conds       = conds;
T.dnfPenalty  = DNF_PENALTY;
T.dtPlantUsed = opt.dtPlant;
end

function J = local_score(trk, q, ctrl, conds, opt, dnf)
J = 0;
for c = 1:numel(conds)
    o = conds{c};
    o.dtPlant = opt.dtPlant;
    o.tMax    = 60;
    [lg, k] = run_reference(trk, q, ctrl, o);
    if ~lg.completed
        J = J + dnf;
    else
        J = J + k.score;
    end
end
J = J / numel(conds);
end
