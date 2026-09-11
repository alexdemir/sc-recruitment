function vref = speed_profile(kappa, ds, p)

N    = numel(kappa);
vref = min(sqrt(p.aLatMax ./ max(abs(kappa), 1e-9)), p.vMax);

for pass = 1:20
    v0 = vref;

    for i = N:-1:1
        j = mod(i, N) + 1;
        vref(i) = min(vref(i), sqrt(vref(j)^2 + 2*p.aBrkMax*ds));
    end

    for i = 1:N
        j = mod(i - 2, N) + 1;
        vref(i) = min(vref(i), sqrt(vref(j)^2 + 2*p.aAccMax*ds));
    end

    if max(abs(vref - v0)) < 1e-9, break; end
end
end
