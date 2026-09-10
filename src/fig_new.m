function f = fig_new(w, h)
%FIG_NEW  A figure with an explicit light surface.
%
%   MATLAB's print leaves the figure background transparent, which renders as
%   black in most viewers and makes every recessive grid line and secondary
%   label unreadable. The surface is therefore pinned here, and InvertHardcopy
%   is turned off so print keeps it instead of substituting white.
f = figure('visible','off', 'position',[0 0 w h], ...
           'color',[0.988 0.988 0.984], 'InvertHardcopy','off');
end
