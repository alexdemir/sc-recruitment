function fig_axes(ax)
%FIG_AXES  Recessive grid and axis furniture on the light surface.
if nargin < 1, ax = gca; end
set(ax, 'color',[0.988 0.988 0.984], ...
        'gridcolor',[0.85 0.85 0.84], 'gridalpha',1, ...
        'xcolor',[0.322 0.318 0.306], 'ycolor',[0.322 0.318 0.306]);
grid(ax, 'on');
end
