function f = plot_summary(trk, p, res, outdir)
if nargin < 4, outdir = 'figures'; end
if exist('OCTAVE_VERSION', 'builtin')
    graphics_toolkit('gnuplot');
end

s1 = [0.165 0.471 0.839];
s2 = [0.922 0.408 0.204];
ink = [0.043 0.043 0.043];
cone = [0.72 0.72 0.71];
cols = {s1, s2};

f = fig_new(1350, 760);

subplot(2, 3, [1 2 4 5]); hold on;
plot(trk.coneL(:,1), trk.coneL(:,2), '.', 'color', cone, 'markersize', 9);
plot(trk.coneR(:,1), trk.coneR(:,2), '.', 'color', cone, 'markersize', 9);
for i = 1:numel(res)
    plot(res(i).log.x, res(i).log.y, '-', 'color', cols{i}, 'linewidth', 2);
end
axis equal; fig_axes();
xlim([min(trk.x)-6, max(trk.x)+6]);
ylim([min(trk.y)-6, max(trk.y)+6]);
xlabel('x  [m]'); ylabel('y  [m]');
title(sprintf('%s lap, %.1f m track, %.1f m wide', 'DV Autocross', trk.lapLen, trk.width), ...
      'color', ink, 'fontsize', 12);
fig_legend([{'cones',''}, {res.name}], 'southeast');

subplot(2, 3, 3); hold on;
half = trk.width/2;
plot([0 trk.lapLen], [ half  half], ':', 'color', [0.85 0.85 0.84], 'linewidth', 1.5);
plot([0 trk.lapLen], [-half -half], ':', 'color', [0.85 0.85 0.84], 'linewidth', 1.5);
for i = 1:numel(res)
    d = [0; cumsum(hypot(diff(res(i).log.x), diff(res(i).log.y)))];
    plot(d, res(i).log.ey, '-', 'color', cols{i}, 'linewidth', 1.8);
end
fig_axes(); xlim([0 trk.lapLen]);
xlabel('distance along the lap  [m]'); ylabel('cross-track error  [m]');
title('tracking error', 'color', ink, 'fontsize', 11);

subplot(2, 3, 6); hold on;
vals = [res(1).kpi.eyMax res(2).kpi.eyMax; res(1).kpi.eyRms res(2).kpi.eyRms];
h = bar(vals, 'grouped');
set(h(1), 'facecolor', s1, 'edgecolor', 'none');
set(h(2), 'facecolor', s2, 'edgecolor', 'none');
for g = 1:2
    for i = 1:2
        xo = g + (i-1.5)*0.28;
        text(xo, vals(g,i), sprintf(' %.3f', vals(g,i)), 'color', ink, ...
             'fontsize', 9, 'horizontalalignment', 'center', 'verticalalignment', 'bottom');
    end
end
fig_axes();
set(gca, 'xtick', [1 2], 'xticklabel', {'max |e_y|', 'RMS e_y'});
ylim([0 1.18*max(vals(:))]);
ylabel('cross-track error  [m]');
title(sprintf('lap %.2f s / %.2f s    points %.1f / %.1f', ...
      res(1).kpi.lapTime, res(2).kpi.lapTime, res(1).kpi.points, res(2).kpi.points), ...
      'color', ink, 'fontsize', 10);

print(f, fullfile(outdir, 'fig11_summary.png'), '-dpng', '-r130');
if usejava('desktop')
    set(f, 'visible', 'on');
else
    close(f);
end
end
