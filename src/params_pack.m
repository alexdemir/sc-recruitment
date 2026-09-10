function P = params_pack(p)
%PARAMS_PACK  Flatten the vehicle parameters into a plain double vector.
%
%   Simulink MATLAB Function blocks take struct parameters only through a bus or
%   a Simulink.Parameter of struct type, which is fragile across releases. A
%   double vector is not, so the model passes P and unpacks it inside the block
%   with params_unpack. Both directions live next to each other here so the
%   order cannot drift.
P = [p.L; p.bodyLen; p.bodyWid; p.bodyCtrOff; p.coneR; ...
     p.dMax; p.dRateMax; p.tauSteer; ...
     p.aLatMax; p.aAccMax; p.aBrkMax; p.vMax; ...
     p.kpV; p.kiV; ...
     p.Ld0; p.kv; p.ke; p.kSoft; p.searchWin];
end
