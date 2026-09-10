function vref = vref_block(i0, VREF)
%VREF_BLOCK  Reference speed at the current centreline index.
%   Split out as its own block so that the speed reference is visibly shared by
%   both controllers in the model diagram, not buried inside either of them.
i = min(max(round(i0), 1), numel(VREF));
vref = VREF(i);
end
