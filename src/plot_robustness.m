function plot_robustness(S, outdir)
%PLOT_ROBUSTNESS  How each controller degrades away from nominal conditions.
%
%   Three small multiples, one per stressor, each with a single y axis carrying
%   the same quantity (effective competition time), so the panels are directly
%   comparable. Nominal is marked on every panel. A rising line means the
%   controller is losing points; a step means it started hitting cones.

if nargin < 2, outdir = 'figures'; end
graphics_toolkit('gnuplot');
s1 = [0.165 0.471 0.839];  s2 = [0.922 0.408 0.204];
ink = [0.043 0.043 0.043];  ink2 = [0.322 0.318 0.306];  gr = [0.85 0.85 0.84];

panels = { struct('x',S.speed.x,   'pp',S.speed.pp,   'st',S.speed.st,   ...
                  'xl','speed profile scale  [-]',        'nom',1.0), ...
           struct('x',S.latency.x*1e3, 'pp',S.latency.pp, 'st',S.latency.st, ...
                  'xl','pose latency  [ms]',              'nom',0), ...
           struct('x',S.noise.x*100,   'pp',S.noise.pp,   'st',S.noise.st,   ...
                  'xl','position noise  \sigma  [cm]',    'nom',0) };

f = figure('visible','off','position',[0 0 1300 420]);
for i = 1:3
    q = panels{i};
    subplot(1,3,i); hold on;
    plot(q.x, q.pp, '-o', 'color', s1, 'linewidth', 2, 'markersize', 5, ...
         'markerfacecolor', s1, 'markeredgecolor', s1);
    plot(q.x, q.st, '-o', 'color', s2, 'linewidth', 2, 'markersize', 5, ...
         'markerfacecolor', s2, 'markeredgecolor', s2);
    yl = ylim();
    plot([q.nom q.nom], yl, ':', 'color', gr, 'linewidth', 1.5);
    ylim(yl);
    grid on; set(gca,'gridcolor',gr,'gridalpha',1,'xcolor',ink2,'ycolor',ink2);
    xlabel(q.xl);
    if i == 1
        ylabel('effective time  [s]   (lap + 2s/cone + 10s/OC)');
        legend({'Pure Pursuit','Stanley','nominal'}, 'location','northwest');
    end
end
print(f, fullfile(outdir,'fig8_robustness.png'), '-dpng', '-r140'); close(f);
end
