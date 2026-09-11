function dst = vehicle_ode(st, deltaCmd, vref, p)

psi   = st(3);
v     = st(4);
delta = st(5);
eInt  = st(6);

eV = vref - v;
a  = p.kpV*eV + p.kiV*eInt;
a  = max(min(a, p.aAccMax), -p.aBrkMax);

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
