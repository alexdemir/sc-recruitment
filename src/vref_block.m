function vref = vref_block(i0, VREF)
i = min(max(round(i0), 1), numel(VREF));
vref = VREF(i);
end
