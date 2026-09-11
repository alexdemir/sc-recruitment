function [i0, ey] = path_nearest(x, y, PX, PY, PPSI, iPrev, win)

N    = numel(PX);
best = inf;
i0   = iPrev;

for k = -win:win
    i = mod(iPrev - 1 + k, N) + 1;
    d = (PX(i) - x)^2 + (PY(i) - y)^2;
    if d < best
        best = d;
        i0   = i;
    end
end

dx = x - PX(i0);
dy = y - PY(i0);
ey = -sin(PPSI(i0))*dx + cos(PPSI(i0))*dy;
end
