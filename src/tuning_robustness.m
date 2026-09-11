function R = tuning_robustness(T, tol)

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

function f = local_frac(J, tol)
b = min(J(:));
f = sum(J(:) <= b*(1+tol)) / numel(J);
end
