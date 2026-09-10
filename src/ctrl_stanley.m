function [delta, i0] = ctrl_stanley(x, y, psi, v, PX, PY, PPSI, iPrev, L, ke, kSoft, win)
%CTRL_STANLEY  Stanley steering law.
%
%   [delta, i0] = CTRL_STANLEY(x, y, psi, v, PX, PY, PPSI, iPrev, L, ke, ...
%                              kSoft, win)
%
%   (x, y, psi) is the REAR AXLE pose. Stanley is derived at the FRONT axle, so
%   the front axle position is formed here:
%
%       (xf, yf) = (x + L*cos(psi), y + L*sin(psi))
%
%   Feeding both controllers from the same point is the most common way to get
%   an unfair comparison, so the reference point of each law is honoured
%   explicitly: pure pursuit at the rear axle, Stanley at the front axle.
%
%   The law superposes a heading term and a cross-track term,
%
%       delta = e_psi + atan( -ke * ey / (kSoft + v) )
%
%   ey is positive when the front axle is to the LEFT of the centreline (see
%   path_nearest), so the cross-track term must be negative there to steer back
%   to the right; hence the leading minus sign. kSoft keeps the term bounded at
%   low speed. (ke, kSoft) are the two gains tuned in tune_gains.m.

% front axle pose
xf = x + L*cos(psi);
yf = y + L*sin(psi);

% signed cross-track error at the front axle
[i0, ey] = path_nearest(xf, yf, PX, PY, PPSI, iPrev, win);

% heading error against the path tangent
epsi = wrap_pi(PPSI(i0) - psi);

delta = epsi + atan2(-ke * ey, kSoft + v);
end
