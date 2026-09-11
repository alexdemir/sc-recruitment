function f = plot_report_figure(trk, res, outdir)
if nargin < 3, outdir = 'figures'; end
if exist('OCTAVE_VERSION', 'builtin')
    graphics_toolkit('gnuplot');
end

s1 = [0.165 0.471 0.839];
s2 = [0.922 0.408 0.204];
ink = [0.043 0.043 0.043];
cone = [0.70 0.70 0.69];
gr  = [0.85 0.85 0.84];
cols = {s1, s2};

f = fig_new(1150, 1180);

subplot(2, 1, 1); hold on;
plot(trk.coneL(:,1), trk.coneL(:,2), '.', 'color', cone, 'markersize', 10);
plot(trk.coneR(:,1), trk.coneR(:,2), '.', 'color', cone, 'markersize', 10);
for i = 1:numel(res)
    plot(res(i).log.x, res(i).log.y, '-', 'color', cols{i}, 'linewidth', 2);
end
axis equal; fig_axes();
xlim([min(trk.x)-5, max(trk.x)+5]);
ylim([min(trk.y)-5, max(trk.y)+5]);
xlabel('x  [m]'); ylabel('y  [m]');
title('The track, and the line each controller drove', 'color', ink, 'fontsize', 13);

subplot(2, 1, 2); hold on;
half = trk.width/2;
plot([0 trk.lapLen], [ half  half], ':', 'color', gr, 'linewidth', 1.5);
plot([0 trk.lapLen], [-half -half], ':', 'color', gr, 'linewidth', 1.5);
for i = 1:numel(res)
    d = [0; cumsum(hypot(diff(res(i).log.x), diff(res(i).log.y)))];
    plot(d, res(i).log.ey, '-', 'color', cols{i}, 'linewidth', 2);
end
fig_axes(); xlim([0 trk.lapLen]);
xlabel('distance along the lap  [m]'); ylabel('distance off the line  [m]');
title('How far each one strayed from the middle of the track', 'color', ink, 'fontsize', 13);
fig_legend([{'track edge',''}, {res.name}], 'southwest');

print(f, fullfile(outdir, 'report_figure.png'), '-dpng', '-r120');
close(f);
end
