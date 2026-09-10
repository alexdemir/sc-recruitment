function a = wrap_pi(a)
%WRAP_PI Wrap angle(s) to (-pi, pi].
%   Toolbox-free replacement for wrapToPi (Mapping Toolbox). Codegen-safe.
a = mod(a + pi, 2*pi) - pi;
end
