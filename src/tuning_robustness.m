function R = tuning_robustness(T, tol)
%TUNING_ROBUSTNESS  How much of each controller's gain space is usable.
%
%   R = TUNING_ROBUSTNESS(T)        tol = 0.01 and 0.05
%
%   Returns, for each controller, the fraction of the swept gain grid whose
%   objective lands within tol of that controller's own best. It answers a
%   question the nominal KPIs cannot: if the gains are set slightly wrong at the
%   event - a different surface, a hurried change between runs - how much does
%   it cost? A law with a wide plateau is worth points that a law with a narrow
%   optimum will lose to a mistuned run.
%
%   Grid points that did not complete the lap are counted as outside the band,
%   which is the honest treatment: they would have been a DNF.

if nargin < 2, tol = [0.01 0.05]; end
for i = 1:numel(tol)
    R.tol(i)    = tol(i);
    R.pp.frac(i) = local_frac(T.pp.J, tol(i));
    R.st.frac(i) = local_frac(T.st.J, tol(i));
end
R.pp.n = numel(T.pp.J);
R.st.n = numel(T.st.J);
R.pp.worst = max(T.pp.J(:));
R.st.worst = max(T.st.J(:));
R.pp.best  = min(T.pp.J(:));
R.st.best  = min(T.st.J(:));
end

% =========================================================================
function f = local_frac(J, tol)
b = min(J(:));
f = sum(J(:) <= b*(1+tol)) / numel(J);
end
