function [delta, i0] = ctrl_pure_pursuit(x, y, psi, v, PX, PY, PPSI, iPrev, ds, L, Ld0, kv, win)

NSEARCH = 400;

N  = numel(PX);
Ld = Ld0 + kv * v;

[i0, ~] = path_nearest(x, y, PX, PY, PPSI, iPrev, win);

gx = PX(i0);  gy = PY(i0);
dp = hypot(gx - x, gy - y);
for k = 1:NSEARCH
    j  = mod(i0 - 1 + k, N) + 1;
    xj = PX(j);  yj = PY(j);
    dj = hypot(xj - x, yj - y);
    if dj >= Ld
        if dj > dp
            lam = (Ld - dp) / (dj - dp);
        else
            lam = 0;
        end
        gx = gx + lam*(xj - gx);
        gy = gy + lam*(yj - gy);
        break;
    end
    gx = xj;  gy = yj;  dp = dj;
end

alpha = wrap_pi(atan2(gy - y, gx - x) - psi);

delta = atan2(2 * L * sin(alpha), Ld);
end
