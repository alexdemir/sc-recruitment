function plot_tuning(T, outdir)

if nargin < 2, outdir = 'figures'; end
if exist('OCTAVE_VERSION', 'builtin')
    graphics_toolkit('gnuplot');
end
cm = local_blue_ramp(64);

maps = { struct('J',T.pp.J, 'xv',T.pp.kv,    'yv',T.pp.Ld0, ...
                'xl','lookahead speed gain  k_v  [s]', 'yl','base lookahead  L_{d0}  [m]', ...
                'ti','Pure Pursuit: mean effective time over the three conditions', ...
                'bx',T.pp.best.kv, 'by',T.pp.best.Ld0, 'bJ',T.pp.best.J, 'fn','fig6_tuning_pp.png'), ...
         struct('J',T.st.J, 'xv',T.st.kSoft, 'yv',T.st.ke, ...
                'xl','softening term  k_{soft}  [m/s]', 'yl','cross-track gain  k_e  [1/s]', ...
                'ti','Stanley: mean effective time over the three conditions', ...
                'bx',T.st.best.kSoft, 'by',T.st.best.ke, 'bJ',T.st.best.J, 'fn','fig7_tuning_st.png') };

for m = 1:numel(maps)
    q = maps{m};
    Jd  = q.J;
    fin = Jd(Jd < 100);
    hi  = max(fin(:));
    Jd(Jd >= 100) = hi;

    f = fig_new(780, 560);
    imagesc(q.xv, q.yv, Jd);
    set(gca,'ydir','normal');
    colormap(cm);
    cb = colorbar();
    ylabel(cb, 'mean effective time  [s]   (clipped: DNF shown at max)');
    hold on;
    plot(q.bx, q.by, 'o', 'markersize', 13, 'linewidth', 2.5, 'color', [0.043 0.043 0.043]);
    text(q.bx, q.by, sprintf('  best  %.2f s', q.bJ), 'color', [0.043 0.043 0.043], ...
         'fontsize', 10, 'verticalalignment','bottom');
    set(gca,'xtick',q.xv,'ytick',q.yv,'xcolor',[0.322 0.318 0.306],'ycolor',[0.322 0.318 0.306]);
    set(gca,'position',[0.11 0.13 0.70 0.76]);
    xlabel(q.xl); ylabel(q.yl);
    title(q.ti, 'color', [0.043 0.043 0.043], 'fontsize', 11);
    print(f, fullfile(outdir, q.fn), '-dpng', '-r140'); close(f);
end
end

function cm = local_blue_ramp(n)
steps = [205 226 251; 183 211 246; 158 197 244; 134 182 239; 109 167 236;
          85 152 231;  57 135 229;  42 120 214;  37 106 191;  28  92 171;
          24  79 149;  16  66 129;  13  54 107] / 255;
t  = linspace(0, 1, size(steps,1))';
tt = linspace(0, 1, n)';
cm = [interp1(t, steps(:,1), tt), interp1(t, steps(:,2), tt), interp1(t, steps(:,3), tt)];
end
