function dst = plant_block(st, deltaCmd, vref, P)
dst = vehicle_ode(st, deltaCmd, vref, params_unpack(P));
end
