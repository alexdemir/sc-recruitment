function [delta, i0] = ctrl_stanley(x, y, psi, v, PX, PY, PPSI, iPrev, L, ke, kSoft, win)

xf = x + L*cos(psi);
yf = y + L*sin(psi);

[i0, ey] = path_nearest(xf, yf, PX, PY, PPSI, iPrev, win);

epsi = wrap_pi(PPSI(i0) - psi);

delta = epsi + atan2(-ke * ey, kSoft + v);
end
