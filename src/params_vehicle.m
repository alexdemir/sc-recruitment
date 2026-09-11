function p = params_vehicle()

p.L        = 1.55;
p.bodyLen  = 2.90;
p.bodyWid  = 1.20;
p.bodyCtrOff = 1.10;
p.coneR    = 0.114;

p.dMax     = deg2rad(24);
p.dRateMax = deg2rad(120);
p.tauSteer = 0.05;

p.aLatMax  = 11.8;
p.aAccMax  = 5.0;
p.aBrkMax  = 8.0;
p.vMax     = 15.0;

p.kpV      = 2.0;
p.kiV      = 0.5;

p.Ld0      = 3.0;
p.kv       = 0.35;
p.ke       = 2.5;
p.kSoft    = 1.0;

p.searchWin = 25;
end
