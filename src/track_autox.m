function trk = track_autox(cfg)

if nargin < 1, cfg = struct(); end

d.V = [   0    0;
         72    0;
         92   16;
         74   34;
         56   26;
         42   40;
         26   28;
          6   34];
d.R = [  14;  6;  7;  6;  9;  8;  16;  11];

d.width  = 3.5;
d.ds     = 0.25;
d.coneSpacingStraight = 5.0;
d.coneSpacingCorner   = 3.0;
d.straightKappa       = 1/200;

f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(cfg, f{i}), cfg.(f{i}) = d.(f{i}); end
end
V = cfg.V;  R = cfg.R;  n = size(V,1);

A    = zeros(n,2);
B    = zeros(n,2);
turn = zeros(n,1);
tlen = zeros(n,1);

for i = 1:n
    P = V(mod(i-2,n)+1, :);   C = V(i,:);   Q = V(mod(i,n)+1, :);
    u1 = (C - P) / norm(C - P);
    u2 = (Q - C) / norm(Q - C);
    turn(i) = wrap_pi(atan2(u2(2),u2(1)) - atan2(u1(2),u1(1)));
    tlen(i) = R(i) * tan(abs(turn(i))/2);
    A(i,:)  = C - u1*tlen(i);
    B(i,:)  = C + u2*tlen(i);
end

for i = 1:n
    j    = mod(i,n)+1;
    edge = norm(V(j,:) - V(i,:));
    if tlen(i) + tlen(j) > edge
        error('track_autox: fillets at vertices %d and %d overrun the %.1f m edge (need %.1f m). Reduce R or move the vertex.', ...
              i, j, edge, tlen(i)+tlen(j));
    end
end

x = [];  y = [];  psi = [];  kap = [];
straightLens = zeros(n,1);

for i = 1:n
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

    sgn = sign(turn(i));
    th0 = atan2(A(i,2)-p0(2), A(i,1)-p0(1));
    ctr = A(i,:) + sgn*R(i)*[-sin(th0), cos(th0)];
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
ds = 0;
per = sum(hypot(diff([x; x(1)]), diff([y; y(1)])));
ds  = per / N;
s   = (0:N-1)' * ds;

[~, iLong] = max(straightLens);
p0 = B(mod(iLong-2,n)+1, :);  p1 = A(iLong,:);
mid = (p0 + p1)/2;
[~, iStart] = min((x-mid(1)).^2 + (y-mid(2)).^2);
rot = @(v) [v(iStart:end); v(1:iStart-1)];
x = rot(x);  y = rot(y);  psi = rot(psi);  kap = rot(kap);

psi = psi(1) + cumsum(wrap_pi(diff([psi(1); psi])));

stats.lapLen       = per;
stats.minRadius    = 1/max(abs(kap));
stats.maxStraight  = max(straightLens);
stats.width        = cfg.width;
stats.closureError = hypot(x(1)-x(end), y(1)-y(end)) - ds;
stats.turnDegrees  = sum(turn)*180/pi;
stats.straightLens = straightLens;
stats.radii        = R;

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

function idx = local_cone_indices(kappa, ds, cfg)
N = numel(kappa);  idx = 1;  i = 1;
while true
    if abs(kappa(i)) < cfg.straightKappa
        step = cfg.coneSpacingStraight;
    else
        step = cfg.coneSpacingCorner;
    end
    i = i + max(1, round(step/ds));
    if i > N, break; end
    idx(end+1,1) = i;
end
end
