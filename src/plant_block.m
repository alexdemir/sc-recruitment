function dst = plant_block(st, deltaCmd, vref, P)
%PLANT_BLOCK  Simulink wrapper around vehicle_ode.
%   The wrapper exists only to unpack the parameter vector; the dynamics are
%   vehicle_ode.m, the same file the reference loop integrates. That is what
%   makes verify_equivalence a test of the model rather than of a second
%   implementation of the physics.
dst = vehicle_ode(st, deltaCmd, vref, params_unpack(P));
end
