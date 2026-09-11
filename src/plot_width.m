function plot_width(W, outdir)

if nargin < 2, outdir = 'figures'; end
if exist('OCTAVE_VERSION', 'builtin')
    graphics_toolkit('gnuplot');
end
s1 = [0.165 0.471 0.839];  s2 = [0.922 0.408 0.204];
ink = [0.043 0.043 0.043];  gr = [0.85 0.85 0.84];

YL = [-0.3, max(1, 1.25*max([W.ppDoo(:); W.stDoo(:)]))];
f  = fig_new(1150, 470);
for b = 1:numel(W.speed)
    subplot(1, 2, b);  hold on;
    plot(W.width, W.ppDoo(:,b), '-o', 'color', s1, 'linewidth', 2, 'markersize', 6, ...
         'markerfacecolor', s1, 'markeredgecolor', s1);
    plot(W.width, W.stDoo(:,b), '-o', 'color', s2, 'linewidth', 2, 'markersize', 6, ...
         'markerfacecolor', s2, 'markeredgecolor', s2);
    plot([3 3], YL, ':', 'color', gr, 'linewidth', 1.5);
    ylim(YL);  xlim([2.93 3.57]);
    set(gca, 'xtick', fliplr(W.width), 'xdir', 'reverse');
    fig_axes();
    for a = 1:numel(W.width)
        if W.ppDoo(a,b) > 0
            text(W.width(a), W.ppDoo(a,b), sprintf('  %.2f m lap', W.ppLap(a,b)), ...
                 'color', ink, 'fontsize', 9, 'verticalalignment','bottom');
        end
    end
    xlabel('track width  [m]     (3 m is the minimum the rules allow)');
    title(sprintf('speed profile x%.2f', W.speed(b)), 'color', ink, 'fontsize', 11);
    if b == 1
        ylabel('cones knocked over');
        fig_legend({'Pure Pursuit','Stanley','rules minimum'}, 'southeast');
    end
end
print(f, fullfile(outdir,'fig10_width.png'), '-dpng', '-r140'); close(f);
end
