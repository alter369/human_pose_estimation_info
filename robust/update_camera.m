function [M, s, t] = update_camera( x, y)
    

    dim = length(x) / 2;
    idx = [2 3 6 9 10 13];
%     idx = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15];
    x = reshape(x, 2, []);
    y = reshape(y, 3, []);
    x = x(:, idx); x = x(:);
    y = y(:, idx); y = y(:);
    
    
    
    njoints = length(x) / 2;
    x = reshape(x, 2, []);
    t = mean(x, 2);
    x = x - repmat(t, 1, njoints);

    y = reshape(y, 3, []);
    u = mean(y, 2);
    y = y - repmat(u, 1, njoints);

    M = x*y'/(y*y');
    [U, S, V] = svd(M);
    S = diag(diag(S));

    Raff = U*[1 0 0; 0 1 0]*V';
    r3 = cross(Raff(1,:), Raff(2, :));
    r3 = r3 / norm(r3);
    R = [Raff; r3];

    if(sum(abs(diag(U)))>1)
        s = S(1:2, 1:2);
    else
        s = S(1:2, 1:2);
        s = rot90(s,2);
    end

    M = s * Raff;

    t = repmat(t, dim, 1);
    M = kron(eye(dim), M);
    s = mean(diag(s));

end
