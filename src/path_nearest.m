function [i0, ey] = path_nearest(x, y, PX, PY, PPSI, iPrev, win)
%PATH_NEAREST  Nearest centreline sample and signed lateral offset.
%
%   [i0, ey] = PATH_NEAREST(x, y, PX, PY, PPSI, iPrev, win)
%
%   Searches only the window iPrev +/- win (indices wrap around the closed
%   loop). The window is not an optimisation detail, it is required for
%   correctness: on a closed track with hairpins the globally nearest sample
%   can jump to the opposite side of the loop, which would make the cross-track
%   error discontinuous and lock the steering. Tracking the index locally keeps
%   the reference point continuous in arc length.
%
%   ey is the lateral offset of the query point in the path frame, POSITIVE
%   WHEN THE POINT IS TO THE LEFT of the centreline (left = +90 deg from the
%   path heading). Both controllers rely on this convention; tests/test_signs.m
%   pins it down with a mirrored track.
%
%   Codegen-safe: fixed-bound loop, no dynamic allocation, scalar outputs.

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

% signed lateral offset in the path frame at i0
dx = x - PX(i0);
dy = y - PY(i0);
ey = -sin(PPSI(i0))*dx + cos(PPSI(i0))*dy;
end
