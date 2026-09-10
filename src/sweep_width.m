function W = sweep_width(p, T, varargin)
%SWEEP_WIDTH  Points against track width, at the rules' lower bound.
%
%   W = SWEEP_WIDTH(p, T)
%
%   D 8.1.1 sets the minimum Autocross track width at 3 m, and the organisers
%   are free to use it. The rest of this study runs on 3.5 m, so a controller
%   could be leading on points purely because of half a metre the rules do not
%   guarantee. This sweep removes that assumption: the same tuned gains are run
%   at 3.50, 3.25 and 3.00 m, at nominal speed and at +30 %, and scored.
%
%   Also recorded is the clearance
%
%       clearance = width/2 - max|ey| - bodyWidth/2
%
%   the room left between the outer edge of the car and the cone line at the
%   worst point of the lap. It is the quantity a cone penalty actually depends
%   on, and it is what the max cross-track KPI is a proxy for.

opt = struct('dtPlant', 1e-3, 'verbose', true);
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end

pPP = p;  pPP.Ld0 = T.pp.best.Ld0;  pPP.kv = T.pp.best.kv;
pST = p;  pST.ke  = T.st.best.ke;   pST.kSoft = T.st.best.kSoft;

W.width = [3.50 3.25 3.00];
W.speed = [1.00 1.30];
n = numel(W.width);  m = numel(W.speed);
for f = {'ppPts','stPts','ppDoo','stDoo','ppClear','stClear','ppTeff','stTeff'}
    W.(f{1}) = nan(n, m);
end

for a = 1:n
    trk = track_autox(struct('width', W.width(a)));
    for b = 1:m
        o = struct('dtPlant', opt.dtPlant, 'tMax', 60, 'speedScale', W.speed(b));
        [~, kPP] = run_reference(trk, pPP, 'pp',      o);
        [~, kST] = run_reference(trk, pST, 'stanley', o);
        tMin = min(kPP.tEff, kST.tEff);
        W.ppPts(a,b)   = fsg_points(kPP.tEff, trk.lapLen, tMin);
        W.stPts(a,b)   = fsg_points(kST.tEff, trk.lapLen, tMin);
        W.ppDoo(a,b)   = kPP.doo;              W.stDoo(a,b)   = kST.doo;
        W.ppTeff(a,b)  = kPP.tEff;             W.stTeff(a,b)  = kST.tEff;
        W.ppClear(a,b) = W.width(a)/2 - kPP.eyMax - p.bodyWid/2;
        W.stClear(a,b) = W.width(a)/2 - kST.eyMax - p.bodyWid/2;
        if opt.verbose
            fprintf('  width %.2f m, speed x%.2f : PP %6.2f pts (%d %s, %+.2f m clear) | ST %6.2f pts (%d %s, %+.2f m clear)\n', ...
                W.width(a), W.speed(b), ...
                W.ppPts(a,b), W.ppDoo(a,b), local_plural(W.ppDoo(a,b)), W.ppClear(a,b), ...
                W.stPts(a,b), W.stDoo(a,b), local_plural(W.stDoo(a,b)), W.stClear(a,b));
            flush_out();
        end
    end
end
end

% =========================================================================
function w = local_plural(n)
if n == 1, w = 'cone'; else, w = 'cones'; end
end
