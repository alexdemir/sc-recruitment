function mdl = build_model(trk, p, varargin)
%BUILD_MODEL  Create the Simulink comparison model programmatically.
%
%   mdl = BUILD_MODEL()            default track and parameters
%   mdl = BUILD_MODEL(trk, p)      explicit track and parameters
%   BUILD_MODEL(..., 'open', true) open the diagram after building
%
%   The model is built by script rather than shipped as a binary .slx, for three
%   reasons: it is a plain-text diff in git, it cannot be locked to one MATLAB
%   release, and the controller blocks read the very same .m files that the
%   reference loop calls, so the two cannot drift apart.
%
%   Structure
%     state          one 6-element Integrator: x, y, psi, v, delta, eInt
%     plant          plant_block -> vehicle_ode, the continuous derivative
%     ZOH + charts   the two steering laws, sampled at 100 Hz
%     switch         CTRL_SEL picks which command reaches the actuator,
%                    0 = pure pursuit, 1 = Stanley (the switch is zero-based)
%     unit delay     carries the centreline index between control steps
%
%   Both controllers are computed every step and one is selected, so a single
%   run also records what the other law would have commanded.

if nargin < 1 || isempty(trk), trk = track_autox(); end
if nargin < 2 || isempty(p),   p   = params_vehicle(); end
opt = struct('name','autox_lateral', 'open', false, 'ctrl', 0, ...
             'dtCtrl', 0.01, 'dtPlant', 1e-3, 'stopTime', 60, ...
             'liveView', false);
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end
mdl = opt.name;

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'src'));

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);

% ---- model workspace: everything the blocks resolve ---------------------
vref = speed_profile(trk.kappa, trk.ds, p);
ST0  = [trk.x(1); trk.y(1); trk.psi(1); vref(1); 0; 0];
W = get_param(mdl, 'ModelWorkspace');
% Parameter-scope chart data resolves by NAME, so these must be spelled exactly
% as the arguments of the controller and plant functions.
vars = { 'PX', trk.x;   'PY', trk.y;     'PPSI', trk.psi;  'VREF', vref; ...
         'ds', trk.ds;  'L', p.L;        'Ld0', p.Ld0;     'kv', p.kv; ...
         'ke', p.ke;    'kSoft', p.kSoft; 'win', p.searchWin; ...
         'P', params_pack(p); 'ST0', ST0; 'CTRL_SEL', opt.ctrl; ...
         'DTCTRL', opt.dtCtrl };
for i = 1:size(vars,1), W.assignin(vars{i,1}, vars{i,2}); end

% ---- blocks -------------------------------------------------------------
add_block('simulink/Continuous/Integrator', [mdl '/state'], ...
          'InitialCondition','ST0', 'Position',[520 120 560 160]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/hold 100 Hz'], ...
          'SampleTime','DTCTRL', 'Position',[620 200 660 240]);
add_block('simulink/Signal Routing/Demux', [mdl '/split'], ...
          'Outputs','6', 'Position',[700 180 705 300]);
add_block('simulink/Discrete/Unit Delay', [mdl '/index memory'], ...
          'SampleTime','DTCTRL', 'InitialCondition','1', 'Position',[760 420 800 460]);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Pure Pursuit'], ...
          'Position',[860 140 990 220]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Stanley'], ...
          'Position',[860 260 990 340]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/speed reference'], ...
          'Position',[860 380 990 430]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/plant'], ...
          'Position',[320 120 450 220]);

add_block('simulink/Sources/Constant', [mdl '/CTRL_SEL'], ...
          'Value','CTRL_SEL', 'Position',[1040 60 1070 90]);
add_block('simulink/Signal Routing/Multiport Switch', [mdl '/select steering'], ...
          'Inputs','2', 'DataPortOrder','Zero-based contiguous', ...
          'Position',[1120 140 1140 260]);
add_block('simulink/Signal Routing/Multiport Switch', [mdl '/select index'], ...
          'Inputs','2', 'DataPortOrder','Zero-based contiguous', ...
          'Position',[1120 300 1140 420]);

% Live trajectory while the simulation runs. XY Graph is a stock Simulink sink,
% so this costs no extra toolbox; the axis limits are taken from the track so
% the whole lap is in frame from the first step.
if opt.liveView
    add_block('simulink/Sinks/XY Graph', [mdl '/live view'], ...
              'xmin', num2str(min(trk.x) - 5), 'xmax', num2str(max(trk.x) + 5), ...
              'ymin', num2str(min(trk.y) - 5), 'ymax', num2str(max(trk.y) + 5), ...
              'st', num2str(opt.dtCtrl), 'Position',[760 40 800 80]);
end

add_block('simulink/Sinks/To Workspace', [mdl '/log_state'], ...
          'VariableName','st_log', 'SaveFormat','Structure With Time', ...
          'Position',[620 60 680 90]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_delta_cmd'], ...
          'VariableName','dcmd_log', 'SaveFormat','Structure With Time', ...
          'Position',[1220 100 1280 130]);

% ---- block functions: the same source files the reference loop uses -----
local_set_chart(mdl, 'plant',            fullfile(here,'src','plant_block.m'),        {'P'});
local_set_chart(mdl, 'speed reference',  fullfile(here,'src','vref_block.m'),         {'VREF'});
local_set_chart(mdl, 'Pure Pursuit',     fullfile(here,'src','ctrl_pure_pursuit.m'), ...
                {'PX','PY','PPSI','ds','L','Ld0','kv','win'});
local_set_chart(mdl, 'Stanley',          fullfile(here,'src','ctrl_stanley.m'), ...
                {'PX','PY','PPSI','L','ke','kSoft','win'});

% ---- wiring -------------------------------------------------------------
lines = { 'plant/1',            'state/1'
          'state/1',            'hold 100 Hz/1'
          'state/1',            'log_state/1'
          'hold 100 Hz/1',      'split/1'
          'split/1',            'Pure Pursuit/1'
          'split/2',            'Pure Pursuit/2'
          'split/3',            'Pure Pursuit/3'
          'split/4',            'Pure Pursuit/4'
          'index memory/1',     'Pure Pursuit/5'
          'split/1',            'Stanley/1'
          'split/2',            'Stanley/2'
          'split/3',            'Stanley/3'
          'split/4',            'Stanley/4'
          'index memory/1',     'Stanley/5'
          'CTRL_SEL/1',         'select steering/1'
          'Pure Pursuit/1',     'select steering/2'
          'Stanley/1',          'select steering/3'
          'CTRL_SEL/1',         'select index/1'
          'Pure Pursuit/2',     'select index/2'
          'Stanley/2',          'select index/3'
          'select index/1',     'index memory/1'
          'select index/1',     'speed reference/1'
          'select steering/1',  'plant/2'
          'speed reference/1',  'plant/3'
          'state/1',            'plant/1'
          'select steering/1',  'log_delta_cmd/1'
          };
for i = 1:size(lines,1)
    add_line(mdl, lines{i,1}, lines{i,2}, 'autorouting','on');
end
if opt.liveView
    add_line(mdl, 'split/1', 'live view/1', 'autorouting','on');
    add_line(mdl, 'split/2', 'live view/2', 'autorouting','on');
end

% unused demux outputs (delta and eInt are not controller inputs)
for k = [5 6]
    add_block('simulink/Sinks/Terminator', sprintf('%s/term%d', mdl, k), ...
              'Position',[745 240+45*k 760 255+45*k]);
    add_line(mdl, sprintf('split/%d', k), sprintf('term%d/1', k), 'autorouting','on');
end

% ---- solver and sample time -------------------------------------------
set_param(mdl, 'SolverType','Fixed-step', 'Solver','ode4', ...
               'FixedStep', num2str(opt.dtPlant), 'StopTime', num2str(opt.stopTime), ...
               'SaveOutput','off', 'SaveTime','off');

save_system(mdl, fullfile(here, [mdl '.slx']));
if opt.open, open_system(mdl); end
fprintf('built %s.slx  (%d blocks, %d lines)\n', mdl, ...
        numel(find_system(mdl,'SearchDepth',1,'Type','Block')), size(lines,1)+2);
end

% =========================================================================
function local_set_chart(mdl, blk, srcFile, paramNames)
%LOCAL_SET_CHART  Load a .m file into a MATLAB Function block and mark the
%   arguments that are constants as Parameter-scope data, so they resolve from
%   the model workspace instead of becoming input ports.
ch = find(sfroot, '-isa','Stateflow.EMChart', 'Path', [mdl '/' blk]);
ch.Script = fileread(srcFile);
d = ch.find('-isa','Stateflow.Data');
for i = 1:numel(d)
    if any(strcmp(d(i).Name, paramNames))
        d(i).Scope = 'Parameter';
    end
end
end
