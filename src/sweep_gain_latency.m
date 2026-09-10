function G = sweep_gain_latency(trk, p, varargin)
%SWEEP_GAIN_LATENCY  Effective time as a function of one gain and loop delay.
%
%   G = SWEEP_GAIN_LATENCY(trk, p)
%
%   The nominal sweeps show Stanley losing control somewhere between 20 ms and
%   50 ms of pose latency at its tuned gain, while pure pursuit is nearly
%   unaffected at 100 ms. That raises the question the comparison actually turns
%   on: is Stanley delay-intolerant, or is its *tuned gain* delay-intolerant?
%
%   This maps the primary gain of each law (Stanley's cross-track gain ke, pure
%   pursuit's base lookahead Ld0) against latency, so the stability boundary can
%   be read off directly rather than inferred from two points.

opt = struct('dtPlant', 2e-3, 'verbose', true);
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end

G.lat   = [0 0.02 0.05 0.10];
G.st.ke = [1 2 3 4 6 9 14];
G.pp.Ld0 = [2 3 4 5 6];
G.st.tEff = nan(numel(G.st.ke), numel(G.lat));
G.pp.tEff = nan(numel(G.pp.Ld0), numel(G.lat));

for a = 1:numel(G.st.ke)
    for b = 1:numel(G.lat)
        q = p;  q.ke = G.st.ke(a);
        G.st.tEff(a,b) = local_run(trk, q, 'stanley', G.lat(b), opt);
    end
    if opt.verbose
        fprintf('  stanley ke=%5.1f : ', G.st.ke(a));
        fprintf('%9.2f', G.st.tEff(a,:));  fprintf('\n');  flush_out();
    end
end
for a = 1:numel(G.pp.Ld0)
    for b = 1:numel(G.lat)
        q = p;  q.Ld0 = G.pp.Ld0(a);
        G.pp.tEff(a,b) = local_run(trk, q, 'pp', G.lat(b), opt);
    end
    if opt.verbose
        fprintf('  pp     Ld0=%4.1f : ', G.pp.Ld0(a));
        fprintf('%9.2f', G.pp.tEff(a,:));  fprintf('\n');  flush_out();
    end
end
end

% =========================================================================
function te = local_run(trk, q, ctrl, lat, opt)
[lg, k] = run_reference(trk, q, ctrl, ...
    struct('dtPlant', opt.dtPlant, 'tMax', 60, 'latency', lat));
if lg.completed, te = k.tEff; else, te = NaN; end
end
