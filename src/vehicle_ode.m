function dst = vehicle_ode(st, deltaCmd, vref, p)
%VEHICLE_ODE  Kinematic bicycle with steering actuator and longitudinal PI.
%
%   dst = VEHICLE_ODE(st, deltaCmd, vref, p)
%
%   State vector (rear axle reference):
%     st(1) x      [m]      st(4) v      [m/s]
%     st(2) y      [m]      st(5) delta  [rad]   actuator output
%     st(3) psi    [rad]    st(6) eInt   [m]     speed-error integral
%
%   Every state maps one-to-one onto an Integrator block in the Simulink model,
%   which is what lets verify_equivalence.m compare the two implementations to
%   solver precision.
%
%   The steering actuator is a rate-limited first-order lag:
%
%       ddelta/dt = sat( (sat(deltaCmd, dMax) - delta)/tauSteer , dRateMax )
%
%   Written as a single ODE rather than a Rate Limiter block in series with a
%   transfer function, because this form integrates identically under RK4 here
%   and under ode4 in Simulink (Saturation -> Gain -> Saturation -> Integrator).
%
%   Limitation, stated for the report: this is a kinematic model. It assumes no
%   tyre slip, so it cannot show understeer at the grip limit. The speed
%   profile therefore caps lateral acceleration at p.aLatMax rather than
%   letting the tyres saturate.

psi   = st(3);
v     = st(4);
delta = st(5);
eInt  = st(6);

% longitudinal PI on the speed reference (identical for both controllers)
eV = vref - v;
a  = p.kpV*eV + p.kiV*eInt;
a  = max(min(a, p.aAccMax), -p.aBrkMax);

% steering actuator
dSat  = max(min(deltaCmd, p.dMax), -p.dMax);
dDot  = (dSat - delta) / p.tauSteer;
dDot  = max(min(dDot, p.dRateMax), -p.dRateMax);

dst = [ v*cos(psi);
        v*sin(psi);
        v*tan(delta)/p.L;
        a;
        dDot;
        eV ];
end
