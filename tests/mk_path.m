function pth = mk_path(kind, arg, ds, N)
%MK_PATH  Synthetic reference paths with analytically known geometry.
%   Test fixture only. 'circle' with radius arg (left turn, centre at (0,arg)),
%   or 'straight' along the x axis with heading arg.
if nargin < 3, ds = 0.25; end
if nargin < 4, N  = 4000; end
s = (0:N-1)' * ds;
switch kind
    case 'circle'
        R   = arg;
        th  = s / R;                     % arc angle from the start
        pth.x     = R*sin(th);
        pth.y     = R - R*cos(th);
        pth.psi   = th;
        pth.kappa = ones(N,1)/R;
    case 'straight'
        pth.x     = s*cos(arg);
        pth.y     = s*sin(arg);
        pth.psi   = arg*ones(N,1);
        pth.kappa = zeros(N,1);
    otherwise
        error('mk_path: unknown kind %s', kind);
end
pth.s = s;  pth.ds = ds;  pth.N = N;
end
