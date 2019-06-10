function L = limb_length( y, C )
%LIMB_LENGTH computes squared limb length
    nlimbs = length(C);
    for i=1:nlimbs
        L(i) = norm(C{i}*y);
    end
    L = L .* L;
end

