function p = params_vehicle()
%PARAMS_VEHICLE Parameters of a representative Formula Student driverless car.
%
%   All values are engineering assumptions for a typical FS/FSAE driverless
%   vehicle, chosen conservatively for an Autocross run. They are listed in
%   report/REPORT.md together with the reasoning, so that every number used in
%   the comparison is traceable.

% --- geometry ------------------------------------------------------------
p.L        = 1.55;    % wheelbase [m]
p.bodyLen  = 2.90;    % overall length, used for cone-contact detection [m]
p.bodyWid  = 1.20;    % overall width  (track + tyre), cone detection    [m]
p.bodyCtrOff = 1.10; % body centre, measured ahead of the rear axle       [m]
p.coneR    = 0.114;   % FS small-cone base half-width                      [m]

% --- steering actuator ---------------------------------------------------
p.dMax     = deg2rad(24);   % steering angle saturation [rad]
p.dRateMax = deg2rad(120);  % steering rate limit       [rad/s]
p.tauSteer = 0.05;          % first-order actuator lag  [s]

% --- performance envelope ------------------------------------------------
p.aLatMax  = 11.8;   % max lateral acceleration (~1.2 g, slick tyres) [m/s^2]
p.aAccMax  = 5.0;    % max longitudinal acceleration                  [m/s^2]
p.aBrkMax  = 8.0;    % max deceleration (magnitude)                   [m/s^2]
p.vMax     = 15.0;   % speed cap for an Autocross lap                 [m/s]

% --- longitudinal controller (identical for both lateral controllers) ----
p.kpV      = 2.0;    % speed-tracking proportional gain
p.kiV      = 0.5;    % speed-tracking integral gain

% --- lateral controller gains (defaults; tune_gains.m overrides these) ---
p.Ld0      = 3.0;    % Pure Pursuit: base lookahead distance   [m]
p.kv       = 0.35;   % Pure Pursuit: lookahead speed gain      [s]
p.ke       = 2.5;    % Stanley: cross-track gain               [1/s]
p.kSoft    = 1.0;    % Stanley: softening term                 [m/s]

% --- nearest-point search ------------------------------------------------
p.searchWin = 25;    % +/- samples searched around the previous index
                     % (at 15 m/s a 10 ms control step advances ~0.6 samples)
end
