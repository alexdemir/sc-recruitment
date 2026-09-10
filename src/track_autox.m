function trk = track_autox(cfg)
%TRACK_AUTOX  Closed, cone-delimited Autocross track for a driverless FS car.
%
%   trk = TRACK_AUTOX()     default layout
%   trk = TRACK_AUTOX(cfg)  overrides fields of the default configuration
%
%   The centreline is a closed polygon whose vertices are rounded by circular
%   arcs tangent to both adjacent edges - the way kart and autocross layouts are
%   actually drawn. Consequences that matter here:
%
%     * closure is exact, because the polygon closes; no least-squares patching
%     * straights are exactly straight, so "straights <= 80 m" is a real check
%     * each corner radius is set explicitly, so a 9 m diameter hairpin is a
%       design input rather than something to be hoped for
%     * heading and curvature are analytic (kappa = 0 on a straight,
%       +/-1/R on an arc), so no finite-difference noise enters the reference
%       signals the controllers see
%
%   Layout guidelines checked against the Formula Student Rules 2026:
%     D 8.1.1  closed loop, straights <= 80 m, min track width 3 m,
%              min turning diameter 9 m
%     D 8.1.2  lap length approximately 200 m to 500 m
%   (D 6.1.3: the DV Autocross layout follows D 8.1.)
%
%   The default layout deliberately contains four features that separate the
%   two steering laws, see report/REPORT.md:
%     - a long straight leading into a tight hairpin  (pure pursuit cuts in)
%     - a slalom of alternating short-radius corners   (Stanley oscillates)
%     - a decreasing-radius corner                    (fixed lookahead fails)
%     - a fast open sweep                             (both should be clean)
%
%   Output fields
%     x,y      [N x 1]  centreline, uniformly spaced by ds, s = 0 on the line
%     psi      [N x 1]  heading, unwrapped [rad]
%     kappa    [N x 1]  signed curvature [1/m]  (+ = left turn)
%     s        [N x 1]  arc length from the start/finish line [m]
%     coneL,coneR       blue (left) and yellow (right) cone positions
%     coneStart         orange start/finish gate
%     stats             measured rule-compliance figures

if nargin < 1, cfg = struct(); end

% ---- default layout -----------------------------------------------------
%  vertices of the closed polygon [m] and the fillet radius at each [m]
d.V = [   0    0;      % long straight runs along the bottom edge
         72    0;      % -> hairpin entry
         92   16;
         74   34;      % hairpin exit
         56   26;      % slalom
         42   40;
         26   28;      % decreasing-radius corner
          6   34];
d.R = [  14;  6;  7;  6;  9;  8;  16;  11];

d.width  = 3.5;      % track width [m], >= 3 m per D 8.1.1
d.ds     = 0.25;     % centreline sample spacing [m]
d.coneSpacingStraight = 5.0;
d.coneSpacingCorner   = 3.0;
d.straightKappa       = 1/200;   % |kappa| below this counts as straight

f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(cfg, f{i}), cfg.(f{i}) = d.(f{i}); end
end
V = cfg.V;  R = cfg.R;  n = size(V,1);

% ---- fillet geometry at every vertex ------------------------------------
A    = zeros(n,2);   % arc entry (tangency on the incoming edge)
B    = zeros(n,2);   % arc exit  (tangency on the outgoing edge)
turn = zeros(n,1);   % signed turn angle at the vertex [rad]
tlen = zeros(n,1);   % tangent length from the vertex to A and to B

for i = 1:n
    P = V(mod(i-2,n)+1, :);   C = V(i,:);   Q = V(mod(i,n)+1, :);
    u1 = (C - P) / norm(C - P);
    u2 = (Q - C) / norm(Q - C);
    turn(i) = wrap_pi(atan2(u2(2),u2(1)) - atan2(u1(2),u1(1)));
    tlen(i) = R(i) * tan(abs(turn(i))/2);
    A(i,:)  = C - u1*tlen(i);
    B(i,:)  = C + u2*tlen(i);
end

% every edge must be long enough for the two fillets that eat into it
for i = 1:n
    j    = mod(i,n)+1;
    edge = norm(V(j,:) - V(i,:));
    if tlen(i) + tlen(j) > edge
        error('track_autox: fillets at vertices %d and %d overrun the %.1f m edge (need %.1f m). Reduce R or move the vertex.', ...
              i, j, edge, tlen(i)+tlen(j));
    end
end

% ---- walk the loop: straight, arc, straight, arc ... --------------------
x = [];  y = [];  psi = [];  kap = [];
straightLens = zeros(n,1);

for i = 1:n
    % straight from the previous arc exit to this arc entry
    p0 = B(mod(i-2,n)+1, :);
    p1 = A(i,:);
    seg = norm(p1 - p0);
    straightLens(i) = seg;
    if seg > 0
        m   = max(1, round(seg/cfg.ds));
        th  = atan2(p1(2)-p0(2), p1(1)-p0(1));
        lam = (0:m-1)'/m;
        x   = [x; p0(1) + lam*(p1(1)-p0(1))];
        y   = [y; p0(2) + lam*(p1(2)-p0(2))];
        psi = [psi; repmat(th, m, 1)];
        kap = [kap; zeros(m,1)];
    end

    % arc through the vertex
    sgn = sign(turn(i));
    th0 = atan2(A(i,2)-p0(2), A(i,1)-p0(1));           % heading entering the arc
    ctr = A(i,:) + sgn*R(i)*[-sin(th0), cos(th0)];     % centre is 90 deg to the left/right
    a0  = atan2(A(i,2)-ctr(2), A(i,1)-ctr(1));
    arcLen = R(i)*abs(turn(i));
    m   = max(1, round(arcLen/cfg.ds));
    lam = (0:m-1)'/m;
    a   = a0 + sgn*abs(turn(i))*lam;
    x   = [x; ctr(1) + R(i)*cos(a)];
    y   = [y; ctr(2) + R(i)*sin(a)];
    psi = [psi; th0 + sgn*abs(turn(i))*lam];
    kap = [kap; repmat(sgn/R(i), m, 1)];
end

N  = numel(x);
ds = 0;  % recomputed below from the true perimeter
per = sum(hypot(diff([x; x(1)]), diff([y; y(1)])));
ds  = per / N;
s   = (0:N-1)' * ds;

% put the start/finish line in the middle of the longest straight
[~, iLong] = max(straightLens);
p0 = B(mod(iLong-2,n)+1, :);  p1 = A(iLong,:);
mid = (p0 + p1)/2;
[~, iStart] = min((x-mid(1)).^2 + (y-mid(2)).^2);
rot = @(v) [v(iStart:end); v(1:iStart-1)];
x = rot(x);  y = rot(y);  psi = rot(psi);  kap = rot(kap);

% keep psi continuous across the rotation and around the loop
psi = psi(1) + cumsum(wrap_pi(diff([psi(1); psi])));

% ---- rule-compliance measurements --------------------------------------
stats.lapLen       = per;
stats.minRadius    = 1/max(abs(kap));
stats.maxStraight  = max(straightLens);
stats.width        = cfg.width;
stats.closureError = hypot(x(1)-x(end), y(1)-y(end)) - ds;
stats.turnDegrees  = sum(turn)*180/pi;      % must be +/-360 for a closed loop
stats.straightLens = straightLens;
stats.radii        = R;

% ---- cones on both boundaries ------------------------------------------
nx = -sin(psi);  ny = cos(psi);
idx = local_cone_indices(kap, ds, cfg);
coneL = [x(idx) + nx(idx)*cfg.width/2, y(idx) + ny(idx)*cfg.width/2];
coneR = [x(idx) - nx(idx)*cfg.width/2, y(idx) - ny(idx)*cfg.width/2];
coneStart = [x(1) + nx(1)*cfg.width/2, y(1) + ny(1)*cfg.width/2;
             x(1) - nx(1)*cfg.width/2, y(1) - ny(1)*cfg.width/2];

trk = struct('x', x, 'y', y, 'psi', psi, 'kappa', kap, 's', s, ...
             'ds', ds, 'N', N, 'lapLen', per, 'width', cfg.width, ...
             'coneL', coneL, 'coneR', coneR, 'coneStart', coneStart, ...
             'cfg', cfg, 'stats', stats);
end

% =========================================================================
function idx = local_cone_indices(kappa, ds, cfg)
%LOCAL_CONE_INDICES  Cone stations: wider pitch on straights, tighter in
%   corners, marched along arc length so the pitch is a true distance.
N = numel(kappa);  idx = 1;  i = 1;
while true
    if abs(kappa(i)) < cfg.straightKappa
        step = cfg.coneSpacingStraight;
    else
        step = cfg.coneSpacingCorner;
    end
    i = i + max(1, round(step/ds));
    if i > N, break; end
    idx(end+1,1) = i; %#ok<AGROW>
end
end
