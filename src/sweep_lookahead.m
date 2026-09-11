function K = sweep_lookahead(trk, p, varargin)

opt = struct('Ld0', [1 2 3 4 5 6], 'dtPlant', 1e-3, 'verbose', true);
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end

K.centreline = trk.lapLen;
K.Ld0 = opt.Ld0;
n = numel(K.Ld0);
for f = {'path','cut','lap','eyMax','doo','tEff'}, K.(f{1}) = nan(n,1); end

for a = 1:n
    q = p;  q.Ld0 = K.Ld0(a);
    [lg, k] = run_reference(trk, q, 'pp', struct('dtPlant', opt.dtPlant, 'tMax', 60));
    K.path(a)  = sum(hypot(diff(lg.x), diff(lg.y)));
    K.cut(a)   = K.path(a) - trk.lapLen;
    K.lap(a)   = k.lapTime;
    K.eyMax(a) = k.eyMax;
    K.doo(a)   = k.doo;
    K.tEff(a)  = k.tEff;
    if opt.verbose
        fprintf('  Ld0 = %.1f m : path %.2f m (%+.2f), lap %.3f s, max|ey| %.3f m, %d cones, tEff %.2f s\n', ...
            K.Ld0(a), K.path(a), K.cut(a), K.lap(a), K.eyMax(a), K.doo(a), K.tEff(a));
        flush_out();
    end
end

[lg, k] = run_reference(trk, p, 'stanley', struct('dtPlant', opt.dtPlant, 'tMax', 60));
K.st.path  = sum(hypot(diff(lg.x), diff(lg.y)));
K.st.cut   = K.st.path - trk.lapLen;
K.st.lap   = k.lapTime;
K.st.eyMax = k.eyMax;
K.st.doo   = k.doo;
K.st.tEff  = k.tEff;
if opt.verbose
    fprintf('  Stanley     : path %.2f m (%+.2f), lap %.3f s, max|ey| %.3f m, %d cones, tEff %.2f s\n', ...
        K.st.path, K.st.cut, K.st.lap, K.st.eyMax, K.st.doo, K.st.tEff);
end
end
