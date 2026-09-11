function fig_legend(labels, loc)
h = legend(labels, 'location', loc);
set(h, 'color', [0.988 0.988 0.984], 'textcolor', [0.043 0.043 0.043], ...
       'edgecolor', [0.85 0.85 0.84]);
end
