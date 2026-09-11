function flush_out()
if exist('OCTAVE_VERSION', 'builtin')
    fflush(stdout);
end
end
