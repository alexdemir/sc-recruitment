function p = params_unpack(P)
p.L          = P(1);   p.bodyLen  = P(2);   p.bodyWid = P(3);
p.bodyCtrOff = P(4);   p.coneR    = P(5);
p.dMax       = P(6);   p.dRateMax = P(7);   p.tauSteer = P(8);
p.aLatMax    = P(9);   p.aAccMax  = P(10);  p.aBrkMax  = P(11);  p.vMax = P(12);
p.kpV        = P(13);  p.kiV      = P(14);
p.Ld0        = P(15);  p.kv       = P(16);  p.ke = P(17);  p.kSoft = P(18);
p.searchWin  = P(19);
end
