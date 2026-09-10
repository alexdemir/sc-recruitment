function flush_out()
%FLUSH_OUT  Flush stdout in Octave; a no-op in MATLAB, which flushes already.
%   Octave buffers fprintf when its output is redirected to a file, which hides
%   the progress of a long sweep until it finishes.
if exist('OCTAVE_VERSION', 'builtin')
    fflush(stdout);
end
end
