function plot_gain_latency(G, T, outdir)

if nargin < 3, outdir = 'figures'; end
if exist('OCTAVE_VERSION', 'builtin')
    graphics_toolkit('gnuplot');
end
cm  = local_blue_ramp(64);
ink = [0.043 0.043 0.043];

panels = { struct('Z',G.st.tEff, 'yv',G.st.ke,  'ti','Stanley: cross-track gain vs. loop delay', ...
                  'yl','cross-track gain  k_e  [1/s]', 'by',T.st.best.ke), ...
           struct('Z',G.pp.tEff, 'yv',G.pp.Ld0, 'ti','Pure Pursuit: base lookahead vs. loop delay', ...
                  'yl','base lookahead  L_{d0}  [m]',  'by',T.pp.best.Ld0) };

allZ = [G.st.tEff(:); G.pp.tEff(:)];
allZ = allZ(isfinite(allZ));
CL   = log10([min(allZ) max(allZ)]);

f = fig_new(1250, 480);
for i = 1:2
    q = panels{i};
    subplot(1,2,i);
    Z = q.Z;  Z(~isfinite(Z)) = 10^CL(2);
    imagesc(G.lat*1e3, 1:numel(q.yv), log10(Z), CL);
    set(gca, 'ydir','normal', 'ytick', 1:numel(q.yv), ...
             'yticklabel', arrayfun(@(v) sprintf('%g', v), q.yv, 'UniformOutput', false), ...
             'xtick', G.lat*1e3, 'xcolor',[0.322 0.318 0.306], 'ycolor',[0.322 0.318 0.306]);
    colormap(cm);
    if i == 2
        cb = colorbar();
        tk = [18 25 40 70 120 200 350];
        set(cb, 'ytick', log10(tk), ...
                'yticklabel', arrayfun(@(v) sprintf('%d', v), tk, 'UniformOutput', false), ...
                'color', [0.043 0.043 0.043]);
        ylabel(cb, 'effective time  [s]   (log scale, DNF at max)', 'color', [0.043 0.043 0.043]);
    end
    hold on;
    plot(0, find(q.yv == q.by), 'o', 'markersize', 12, 'linewidth', 2.5, 'color', ink);
    text(2, find(q.yv == q.by), '  tuned', 'color', ink, 'fontsize', 10);
    xlabel('pose latency  [ms]');  ylabel(q.yl);
    title(q.ti, 'color', ink, 'fontsize', 11);
end
print(f, fullfile(outdir,'fig9_gain_latency.png'), '-dpng', '-r140'); close(f);
end

function cm = local_blue_ramp(n)
steps = [205 226 251; 183 211 246; 158 197 244; 134 182 239; 109 167 236;
          85 152 231;  57 135 229;  42 120 214;  37 106 191;  28  92 171;
          24  79 149;  16  66 129;  13  54 107] / 255;
t  = linspace(0, 1, size(steps,1))';
tt = linspace(0, 1, n)';
cm = [interp1(t, steps(:,1), tt), interp1(t, steps(:,2), tt), interp1(t, steps(:,3), tt)];
end
