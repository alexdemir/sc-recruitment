function plot_width(W, outdir)
%PLOT_WIDTH  DV Autocross points against track width.
%
%   Two panels, nominal speed and +30 %, sharing one y axis so the two can be
%   compared at a glance. The dashed marker at 3 m is the width the rules
%   permit; everything to the right of it is track the organisers are not
%   obliged to give. Cone counts are labelled directly on the points that lost
%   any, because the drop is caused by cones and the reader should not have to
%   infer that.

if nargin < 2, outdir = 'figures'; end
if exist('OCTAVE_VERSION', 'builtin')
    graphics_toolkit('gnuplot');
end
s1 = [0.165 0.471 0.839];  s2 = [0.922 0.408 0.204];
ink = [0.043 0.043 0.043];  gr = [0.85 0.85 0.84];

YL = [0.97*min([W.ppPts(:); W.stPts(:)]) 101.5];
f  = fig_new(1150, 470);
for b = 1:numel(W.speed)
    subplot(1, 2, b);  hold on;
    plot(W.width, W.ppPts(:,b), '-o', 'color', s1, 'linewidth', 2, 'markersize', 6, ...
         'markerfacecolor', s1, 'markeredgecolor', s1);
    plot(W.width, W.stPts(:,b), '-o', 'color', s2, 'linewidth', 2, 'markersize', 6, ...
         'markerfacecolor', s2, 'markeredgecolor', s2);
    plot([3 3], YL, ':', 'color', gr, 'linewidth', 1.5);
    ylim(YL);  xlim([2.93 3.57]);
    set(gca, 'xtick', fliplr(W.width), 'xdir', 'reverse');
    fig_axes();
    for a = 1:numel(W.width)
        if W.ppDoo(a,b) > 0
            if W.ppDoo(a,b) == 1, word = 'cone'; else, word = 'cones'; end
            text(W.width(a), W.ppPts(a,b), sprintf('  %d %s', W.ppDoo(a,b), word), ...
                 'color', ink, 'fontsize', 9, 'verticalalignment','top');
        end
    end
    xlabel('track width  [m]     (3 m is the minimum the rules allow)');
    title(sprintf('speed profile x%.2f', W.speed(b)), 'color', ink, 'fontsize', 11);
    if b == 1
        ylabel('FSG DV Autocross points');
        fig_legend({'Pure Pursuit','Stanley','rules minimum'}, 'southeast');
    end
end
print(f, fullfile(outdir,'fig10_width.png'), '-dpng', '-r140'); close(f);
end
