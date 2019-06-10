function C = select_limb(ids, njoints)
%SELECT_LIMB generate limb selection matrix

    nlimbs = size(ids, 2);
    for i=1:nlimbs
        Sel_1 = zeros(3, njoints*3);
        Sel_1(:, 3*(ids(1, i)-1)+1:3*ids(1,i)) = eye(3);
        Sel_2 = zeros(3, njoints*3);
        Sel_2(:, 3*(ids(2, i)-1)+1:3*ids(2,i)) = eye(3);
        C{i} = Sel_1 - Sel_2;
    end

end

