function [ oCoor ] = world2object( c3d )

    oCoor = zeros(size(c3d));
    lshld = 3;
    rshld = 6;
    wist = 9;
    
    njoints = size(c3d, 1) / 3;
    nframe = size(c3d, 2);
    
    
    for i=1:nframe
        %world coordinate
        wCoor = reshape(c3d(:, i),3,[]);
        
        xaxis = wCoor(:, rshld) - wCoor(:, lshld);
        xaxis = xaxis ./ norm(xaxis);
        
        mid_shld = 0.5*(wCoor(:, rshld)+wCoor(:, lshld));
        yaxis = mid_shld - wCoor(:,wist);
        yaxis = yaxis ./ norm(yaxis);
        
        zaxis = cross(xaxis, yaxis);
        zaxis = zaxis ./ norm(zaxis);
        tranH = [xaxis yaxis zaxis];
        tranH = [tranH; 0 0 0];
        tranS = [mid_shld; 1];
        
        T_OW = [tranH tranS];         
        t = T_OW\ [wCoor; ones(1, njoints)];
        t = t(1:3, :);
        oCoor(:,i) = t(:);
    end
end








