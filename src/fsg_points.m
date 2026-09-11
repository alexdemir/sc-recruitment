function pts = fsg_points(tEff, lapLen, tMin, pMax)

if nargin < 4, pMax = 100; end
tMax = lapLen / 6;
tEff = min(tEff, tMax);
pts  = 0.9*pMax * (tMax - tEff)/(tMax - tMin) + 0.1*pMax;
end
