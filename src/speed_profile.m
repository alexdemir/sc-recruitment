function vref = speed_profile(kappa, ds, p)
%SPEED_PROFILE  Reference speed along a closed centreline.
%
%   vref = SPEED_PROFILE(kappa, ds, p)
%
%   Three limits are applied in turn:
%     1. lateral grip     v <= sqrt(aLatMax / |kappa|)
%     2. braking          reachable backwards from every point
%     3. acceleration     reachable forwards from every point
%   capped by p.vMax throughout.
%
%   The backward and forward passes WRAP around the start/finish line and are
%   repeated until they stop changing. On a closed loop a single non-wrapping
%   pass leaves an artificial speed notch at s = 0, which would show up as a
%   fake braking event in the lap time of both controllers.
%
%   This profile is a function of arc length only, so it is identical for the
%   two lateral controllers by construction. That is what makes the comparison
%   attributable to the steering law alone.

N    = numel(kappa);
vref = min(sqrt(p.aLatMax ./ max(abs(kappa), 1e-9)), p.vMax);

for pass = 1:20
    v0 = vref;

    % backward pass: v(i) must be low enough to brake down to v(i+1)
    for i = N:-1:1
        j = mod(i, N) + 1;                                  % i+1, wrapped
        vref(i) = min(vref(i), sqrt(vref(j)^2 + 2*p.aBrkMax*ds));
    end

    % forward pass: v(i) cannot exceed what can be reached from v(i-1)
    for i = 1:N
        j = mod(i - 2, N) + 1;                              % i-1, wrapped
        vref(i) = min(vref(i), sqrt(vref(j)^2 + 2*p.aAccMax*ds));
    end

    if max(abs(vref - v0)) < 1e-9, break; end
end
end
