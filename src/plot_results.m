function plot_results(trk, p, res, outdir)
%PLOT_RESULTS  Figures for the controller comparison report.
%
%   plot_results(trk, p, res, outdir)
%
%   res is a struct array with fields .name .log .kpi, one entry per controller.
%
%   Figure conventions, applied everywhere: two series carry the two fixed
%   categorical hues in a fixed order (never cycled), every multi-series figure
%   has a legend, lines are thin, the grid is recessive, and no figure uses two
%   y scales. Cone colours in the layout figure are the rule colours (blue on
%   the left boundary, yellow on the right, orange for the start gate) because
%   there they carry meaning; in the trajectory figure the cones drop to grey so
%   that the two trajectories own the colour.

if nargin < 4, outdir = 'figures'; end
if ~exist(outdir, 'dir'), mkdir(outdir); end
graphics_toolkit('gnuplot');

C  = struct('s1',[0.165 0.471 0.839], ...   % #2a78d6  categorical slot 1
            's2',[0.922 0.408 0.204], ...   % #eb6834  categorical slot 2
            'ink',[0.043 0.043 0.043], ...  % primary text
            'ink2',[0.322 0.318 0.306], ... % secondary text
            'grid',[0.85 0.85 0.84], ...
            'cone',[0.72 0.72 0.71], ...
            'blue',[0.10 0.35 0.75], ...    % rule cone colours
            'yellow',[0.90 0.70 0.05], ...
            'orange',[0.95 0.45 0.10]);
LW = 2;

% ---------------------------------------------------------------- fig 1: layout
f = figure('visible','off','position',[0 0 1100 850]);
hold on;
plot(trk.x, trk.y, '--', 'color', C.grid, 'linewidth', 1);
plot(trk.coneL(:,1), trk.coneL(:,2), 'o', 'markersize', 4, ...
     'markerfacecolor', C.blue,   'markeredgecolor', C.blue);
plot(trk.coneR(:,1), trk.coneR(:,2), 'o', 'markersize', 4, ...
     'markerfacecolor', C.yellow, 'markeredgecolor', C.yellow);
plot(trk.coneStart(:,1), trk.coneStart(:,2), 's', 'markersize', 9, ...
     'markerfacecolor', C.orange, 'markeredgecolor', C.orange);
axis equal; grid on; set(gca,'gridcolor',C.grid,'gridalpha',1,'xcolor',C.ink2,'ycolor',C.ink2);
xlabel('x  [m]'); ylabel('y  [m]');
title(sprintf('DV Autocross layout   |   lap %.1f m,  min radius %.1f m,  longest straight %.1f m,  width %.1f m', ...
      trk.stats.lapLen, trk.stats.minRadius, trk.stats.maxStraight, trk.width), ...
      'color', C.ink, 'fontsize', 11);
legend({'centreline','left boundary (blue)','right boundary (yellow)','start / finish'}, ...
       'location','northeastoutside');
print(f, fullfile(outdir,'fig1_track.png'), '-dpng', '-r140'); close(f);

% ----------------------------------------------------- fig 2: trajectories
f = figure('visible','off','position',[0 0 1100 850]);
hold on;
plot(trk.coneL(:,1), trk.coneL(:,2), '.', 'color', C.cone, 'markersize', 8);
plot(trk.coneR(:,1), trk.coneR(:,2), '.', 'color', C.cone, 'markersize', 8);
cols = {C.s1, C.s2};
for i = 1:numel(res)
    plot(res(i).log.x, res(i).log.y, '-', 'color', cols{i}, 'linewidth', LW);
end
axis equal; grid on; set(gca,'gridcolor',C.grid,'gridalpha',1,'xcolor',C.ink2,'ycolor',C.ink2);
xlabel('x  [m]'); ylabel('y  [m]');
title('Driven trajectories over the cone corridor', 'color', C.ink, 'fontsize', 11);
legend([{'cones',''}, {res.name}], 'location','northeastoutside');
print(f, fullfile(outdir,'fig2_trajectories.png'), '-dpng', '-r140'); close(f);

% ------------------------------------------------------ fig 3: cross-track
f = figure('visible','off','position',[0 0 1100 450]);
hold on;
half = trk.width/2;
plot([0 trk.lapLen], [ half  half], ':', 'color', C.grid, 'linewidth', 1);
plot([0 trk.lapLen], [-half -half], ':', 'color', C.grid, 'linewidth', 1);
for i = 1:numel(res)
    s = local_arclen(res(i).log, trk);
    plot(s, res(i).log.ey, '-', 'color', cols{i}, 'linewidth', LW);
end
grid on; set(gca,'gridcolor',C.grid,'gridalpha',1,'xcolor',C.ink2,'ycolor',C.ink2);
xlim([0 trk.lapLen]);
xlabel('distance along the lap  [m]'); ylabel('cross-track error  [m]');
title('Cross-track error (positive = left of the centreline)', 'color', C.ink, 'fontsize', 11);
legend([{'track edge',''}, {res.name}], 'location','northeastoutside');
print(f, fullfile(outdir,'fig3_crosstrack.png'), '-dpng', '-r140'); close(f);

% --------------------------------------------------------- fig 4: steering
f = figure('visible','off','position',[0 0 1100 450]);
hold on;
for i = 1:numel(res)
    s = local_arclen(res(i).log, trk);
    plot(s, rad2deg(res(i).log.delta), '-', 'color', cols{i}, 'linewidth', LW);
end
plot([0 trk.lapLen], rad2deg([ p.dMax  p.dMax]), ':', 'color', C.grid, 'linewidth', 1);
plot([0 trk.lapLen], rad2deg([-p.dMax -p.dMax]), ':', 'color', C.grid, 'linewidth', 1);
grid on; set(gca,'gridcolor',C.grid,'gridalpha',1,'xcolor',C.ink2,'ycolor',C.ink2);
xlim([0 trk.lapLen]);
xlabel('distance along the lap  [m]'); ylabel('steering angle  [deg]');
title('Steering angle at the road wheels', 'color', C.ink, 'fontsize', 11);
legend([{res.name}, {'actuator limit'}], 'location','northeastoutside');
print(f, fullfile(outdir,'fig4_steering.png'), '-dpng', '-r140'); close(f);

% ------------------------------------------------------------ fig 5: speed
f = figure('visible','off','position',[0 0 1100 450]);
hold on;
plot(trk.s, speed_profile(trk.kappa, trk.ds, p), '--', 'color', C.grid, 'linewidth', 1.5);
for i = 1:numel(res)
    s = local_arclen(res(i).log, trk);
    plot(s, res(i).log.v, '-', 'color', cols{i}, 'linewidth', LW);
end
grid on; set(gca,'gridcolor',C.grid,'gridalpha',1,'xcolor',C.ink2,'ycolor',C.ink2);
xlim([0 trk.lapLen]);
xlabel('distance along the lap  [m]'); ylabel('speed  [m/s]');
title('Speed: reference profile and what each controller achieved', 'color', C.ink, 'fontsize', 11);
legend([{'reference profile'}, {res.name}], 'location','northeastoutside');
print(f, fullfile(outdir,'fig5_speed.png'), '-dpng', '-r140'); close(f);
end

% =========================================================================
function s = local_arclen(log, trk)
%LOCAL_ARCLEN  Distance travelled along the lap, for the x axis of the traces.
s = [0; cumsum(hypot(diff(log.x), diff(log.y)))];
s = s * (trk.lapLen / max(s(end), eps));
end
