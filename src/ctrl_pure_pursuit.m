function [delta, i0] = ctrl_pure_pursuit(x, y, psi, v, PX, PY, PPSI, iPrev, ds, L, Ld0, kv, win)
%CTRL_PURE_PURSUIT  Geometric pure pursuit steering law.
%
%   [delta, i0] = CTRL_PURE_PURSUIT(x, y, psi, v, PX, PY, PPSI, iPrev, ds, ...
%                                   L, Ld0, kv, win)
%
%   (x, y, psi) is the REAR AXLE pose, the reference point the pure pursuit
%   geometry is derived for. The goal point is where the path crosses the circle
%   of radius Ld centred on the rear axle, and the steering angle that puts the
%   rear axle on the arc through that point is
%
%       delta = atan( 2 * L * sin(alpha) / Ld )
%
%   with alpha the bearing of the goal point in the vehicle frame.
%
%   Ld is a STRAIGHT-LINE distance, not an arc length. Marching Ld along the arc
%   length instead places the goal point too close whenever the path is curved
%   (arc > chord), which makes the commanded angle too small in tight corners -
%   a bias that grows as the radius shrinks. tests/run_tests.m pins this down:
%   with the chord form the law returns the Ackermann angle atan(L/R) exactly on
%   a circle of any radius, independently of Ld.
%
%   The lookahead is scheduled with speed, Ld = Ld0 + kv * v, because a fixed
%   lookahead that is stable in a hairpin cuts corners on a fast section and
%   vice versa. (Ld0, kv) are the two gains tuned in tune_gains.m.
%
%   Inputs are plain arrays rather than a struct so that this file can be read
%   into a Simulink MATLAB Function block unchanged. The goal-point search has a
%   constant iteration bound, which keeps it codegen-safe.

NSEARCH = 400;                 % constant bound: 100 m of path at ds = 0.25 m

N  = numel(PX);
Ld = Ld0 + kv * v;

% nearest centreline sample (index tracked locally, see path_nearest)
[i0, ~] = path_nearest(x, y, PX, PY, PPSI, iPrev, win);

% goal point: first crossing of the circle of radius Ld, interpolated between
% the two samples that bracket it
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

% bearing of the goal point in the vehicle frame
alpha = wrap_pi(atan2(gy - y, gx - x) - psi);

delta = atan2(2 * L * sin(alpha), Ld);
end
