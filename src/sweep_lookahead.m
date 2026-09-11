function K = sweep_lookahead(trk, p, varargin)
%SWEEP_LOOKAHEAD  Where pure pursuit's lap-time advantage comes from.
%
%   K = SWEEP_LOOKAHEAD(trk, p)
%
%   Pure pursuit finishes the lap ahead of Stanley while tracking the centreline
%   seven times less accurately, which looks backwards until the mechanism is
%   measured. Both controllers receive the same speed reference at the same
%   arc-length station, so neither chooses its speed and lap time reduces to the
%   distance driven. Pure pursuit steers along a chord to a point ahead on the
%   centreline, and a chord always passes inside the arc, so its path sits inside
%   the centreline through every corner - further inside the longer the lookahead.
%
%   Sweeping Ld0 makes the chain explicit: more lookahead, more corner cut,
%   shorter path, faster lap, larger cross-track error - until the error spends
%   the clearance to the cone line and the 2 s penalties erase the gain.
%
%   p must carry BOTH controllers' tuned gains: the sweep overrides Ld0 for the
%   pure pursuit rows, but the final Stanley row uses p.ke and p.kSoft as given.
%   Passing a pure-pursuit-only parameter set silently compares against an
%   untuned Stanley.
%
%   The row worth reading twice is the comparison at equal accuracy. Held to
%   Stanley's tracking error, pure pursuit is the slower of the two: the
%   nominal-tuning advantage is not the law being better, it is the law cutting
%   more corner.

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

% the reference point: Stanley at its own tuned gain
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
