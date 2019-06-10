function [pose, M, t, m_limb_length] = L1WAWS( x, B, mu, init_pose, C, L, spam_param, param )
%L1WAWS Solves the optimization problem
%   min || x - M(Ba+u) ||_1 + theta ||a||_1
%     s.t. ||C_i(Ba+u)||^2 = L_i, i=1,...,t
%   x is 2D pose            of dimension R^{2n*1}
%   B is basis dictionary   of dimension R^{2n*k}
%   mu is mean 3D pose      of dimension R^{3n*1}
%   C is a cell             of length L
%   param.max_iter          maximum iteration for updating camera and pose
%   param.terminate         threshold for terminating the update
    njoints = length(x) / 2;
    [M, s, t] = update_camera(x, init_pose);
    disp(M(1:2,1:3));
    pose = init_pose;
       
    for i=1:param.max_iter
        coarse_alpha = mexLasso(x/s-M/s*mu-t/s, M/s*B, spam_param);
        m_alpha = update_pose(x, B(:, coarse_alpha~=0), mu, C, L, s, M, t, coarse_alpha(coarse_alpha~=0), param);
        old_pose = pose; 
        pose = B(:, coarse_alpha~=0) * m_alpha + mu;
        
        old_M = M; old_t = t;
        [M, s, t] = update_camera(x, pose);
        
        disp(M(1:2,1:3));
        
        if max(norm(old_M-M), norm(pose-old_pose)) < param.terminate
            break;
        end
    end
    m_limb_length = limb_length(pose, C);
end
