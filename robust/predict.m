% The code relies on CVX for solving convex problems and SPAMS for learning
% sparse bases. They can be downloaded from 
% (1)  http://spams-devel.gforge.inria.fr/ and
% (2) http://cvxr.com/cvx/ respectively

addpath(genpath('cvx'));
addpath(genpath('spams-matlab'));
cvx_startup;

% Parameters for spams (testing)
param_test.iter = 200;
param_test.lambda = 0.005;

% Parameters for Aternating Direction Method
param.max_iter = 10;
param.terminate = 0.2;
param.max_iter2 = 10;
param.terminate2 = 0.01;
param.max_iter3 = 10;
param.terminate3 = 0.01;
param.theta = 0.05;
param.delta = 20;

% Load the model
load('model');
njoints = size(B, 1) / 3;
limb_ids = [1 2; 2 3; 4 5; 5 6; 7 8; 8 9; 10 11; 11 12]';
C = select_limb(limb_ids, njoints);
disp('im here');
disp(B);
disp(mu);
disp(init_pose);
s = [1 0 0; 0 1 0];
theta = pi/6;
R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
M = kron(eye(njoints), s*R);

for i=1:20:size(tposes,2)
    y = tposes(:, i);
    x = M * y;
    
    L = limb_length(y, C);

    [y_, esti_limb_length] = L1WAWS( x, B, mu, init_pose,...
        C, L, param_test, param );
    
    scale = 1 / sqrt(L(end));
    err = mse(y_, y) * scale;
    err2 = mse(init_pose, y) * scale;
    disp([err err2]);
    display_pose(y_, y);
end
