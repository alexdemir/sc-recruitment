function W = sweep_width(p, T, varargin)

opt = struct('dtPlant', 1e-3, 'verbose', true);
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end

pPP = p;  pPP.Ld0 = T.pp.best.Ld0;  pPP.kv = T.pp.best.kv;
pST = p;  pST.ke  = T.st.best.ke;   pST.kSoft = T.st.best.kSoft;

W.width = [3.50 3.25 3.00];
W.speed = [1.00 1.30];
n = numel(W.width);  m = numel(W.speed);
for f = {'ppDoo','stDoo','ppClear','stClear','ppLap','stLap'}
    W.(f{1}) = nan(n, m);
end

for a = 1:n
    trk = track_autox(struct('width', W.width(a)));
    for b = 1:m
        o = struct('dtPlant', opt.dtPlant, 'tMax', 60, 'speedScale', W.speed(b));
        [~, kPP] = run_reference(trk, pPP, 'pp',      o);
        [~, kST] = run_reference(trk, pST, 'stanley', o);
        W.ppDoo(a,b)   = kPP.doo;      W.stDoo(a,b) = kST.doo;
        W.ppLap(a,b)   = kPP.lapTime;  W.stLap(a,b) = kST.lapTime;
        W.ppClear(a,b) = W.width(a)/2 - kPP.eyMax - p.bodyWid/2;
        W.stClear(a,b) = W.width(a)/2 - kST.eyMax - p.bodyWid/2;
        if opt.verbose
            fprintf('  width %.2f m, speed x%.2f : PP %d %s, %+.2f m clear | ST %d %s, %+.2f m clear\n', ...
                W.width(a), W.speed(b), ...
                W.ppDoo(a,b), local_plural(W.ppDoo(a,b)), W.ppClear(a,b), ...
                W.stDoo(a,b), local_plural(W.stDoo(a,b)), W.stClear(a,b));
            flush_out();
        end
    end
end
end

function w = local_plural(n)
if n == 1, w = 'cone'; else, w = 'cones'; end
end
