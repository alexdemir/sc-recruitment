function out = sim_and_plot(ctrl)

if nargin < 1, ctrl = 0; end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'src'));

trk = track_autox();
p   = params_vehicle();
if exist(fullfile(here,'results_tuning.mat'), 'file')
    T = load(fullfile(here,'results_tuning.mat')).T;
    if ctrl == 0
        p.Ld0 = T.pp.best.Ld0;  p.kv = T.pp.best.kv;
    else
        p.ke  = T.st.best.ke;   p.kSoft = T.st.best.kSoft;
    end
end
if ctrl == 0, name = 'Pure Pursuit'; else, name = 'Stanley'; end

mdl = build_model(trk, p, 'ctrl', ctrl, 'stopTime', 25);
out = sim(mdl);
st  = out.get('st_log');
X   = squeeze(st.signals.values);
if size(X,1) == 6, X = X.'; end

col = [0.165 0.471 0.839];
if ctrl == 1, col = [0.922 0.408 0.204]; end

f = fig_new(1200, 780);
subplot(2,2,[1 3]); hold on;
plot(trk.coneL(:,1), trk.coneL(:,2), '.', 'color', [0.72 0.72 0.71], 'markersize', 8);
plot(trk.coneR(:,1), trk.coneR(:,2), '.', 'color', [0.72 0.72 0.71], 'markersize', 8);
plot(X(:,1), X(:,2), '-', 'color', col, 'linewidth', 2);
axis equal; fig_axes();
xlabel('x  [m]'); ylabel('y  [m]');
title([name ' - driven line'], 'color', [0.043 0.043 0.043], 'fontsize', 11);

ey = zeros(size(X,1),1);  i0 = 1;
for k = 1:size(X,1)
    [i0, ey(k)] = path_nearest(X(k,1), X(k,2), trk.x, trk.y, trk.psi, i0, 3);
end
subplot(2,2,2);
plot(st.time, ey, '-', 'color', col, 'linewidth', 2); fig_axes();
xlabel('time  [s]'); ylabel('cross-track error  [m]');
title(sprintf('max |e_y| = %.3f m,  RMS = %.4f m', max(abs(ey)), sqrt(mean(ey.^2))), ...
      'color', [0.043 0.043 0.043], 'fontsize', 10);

subplot(2,2,4);
plot(st.time, rad2deg(X(:,5)), '-', 'color', col, 'linewidth', 2); fig_axes();
xlabel('time  [s]'); ylabel('steering angle  [deg]');
title('road-wheel angle', 'color', [0.043 0.043 0.043], 'fontsize', 10);

if usejava('desktop')
    set(f, 'visible', 'on');
end
fprintf('%s: %d logged samples, max |e_y| = %.3f m\n', name, size(X,1), max(abs(ey)));
end
