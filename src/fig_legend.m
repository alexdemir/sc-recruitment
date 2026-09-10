function fig_legend(labels, loc)
%FIG_LEGEND  Legend on the light surface with text in ink, not in series colour.
%   MATLAB's default legend box picks up the dark figure colour it was created
%   under and renders as a black panel on the light surface.
h = legend(labels, 'location', loc);
set(h, 'color', [0.988 0.988 0.984], 'textcolor', [0.043 0.043 0.043], ...
       'edgecolor', [0.85 0.85 0.84]);
end
