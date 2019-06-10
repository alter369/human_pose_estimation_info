function err = mse( esti, gnd )
%MSE computes mean square error on each joint
    esti = reshape(esti, 3, []);
    gnd = reshape(gnd, 3, []);
    
    err = mean(sqrt(sum((esti-gnd).*(esti-gnd),1)));

end

