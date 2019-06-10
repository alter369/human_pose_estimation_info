% The code relies on CVX for solving convex problems and SPAMS for learning
% sparse bases. They can be downloaded from 
% (1)  http://spams-devel.gforge.inria.fr/ and
% (2) http://cvxr.com/cvx/ respectively

addpath(genpath('cvx'));
addpath(genpath('spams-matlab/test_release'));
addpath(genpath('spams-matlab/src_release'));
addpath(genpath('spams-matlab/build'));
cvx_startup;

% Parameters for spams (learning)
param_learn.K = 200;               % Number of bases
param_learn.lambda = 0.01;
param_learn.iter = 300;

% Parameters for spams (testing)
param_test = param_learn;
param_test.lambda = 0.001;

param.max_iter = 10;
param.terminate = 0.1;
param.max_iter2 = 10;
param.terminate2 = 0.01;
param.max_iter3 = 10;
param.terminate3 = 0.01;
param.theta = 0.05;
param.delta = 20;

% Load data for learning bases & testing
load('S1_Box_1_C1.mat');
pose3Ds = world2object(pose3Ds);


% mean-center the pose
for i=1:size(pose3Ds,2)
    pose = reshape(pose3Ds(:,i), 3, []);
    center = mean(pose,2);
    pose = pose - repmat(center, 1, size(pose, 2));
    pose3Ds(:,i) = pose(:);
end

scale = sqrt(sum(pose3Ds.*pose3Ds,1));
pose3Ds = pose3Ds ./ repmat(scale, size(pose3Ds,1), 1);
mu = zeros(size(pose3Ds,1), 1);
train = pose3Ds - repmat(mu, 1, size(pose3Ds,2));
B = mexTrainDL(train,param_learn);

% intializate pose for optimization
init_pose = mean(pose3Ds, 2);

% Synthesize virtual cameras
njoints = size(B, 1) / 3;
s = [1 0 0; 0 1 0];
theta = pi / 6;
R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
M = kron(eye(njoints), s*R);

npose = size(pose3Ds, 2);
esti_Ys = cell(1, npose);
esti_Ms = cell(1, npose);
esti_Ts = cell(1, npose);

for i=1:size(pose3Ds,2)
    y = pose3Ds(:, i);
    x = pose2Ds(:, i);
    
    % compute scales & mean-center coordinates
    x = reshape(x, 2, []);
    x = x - repmat(mean(x, 2), 1, size(x,2));
    x = x(:);
    [~, s, t] = update_camera( x, y);
    x = x / s;
    
    % joint indices for eight limbs
    limb_ids = [3 4; 4 5; 6 7; 7 8; 10 11; 11 12; 13 14; 14 15]';
    C = select_limb(limb_ids, njoints);
    L = limb_length(y, C);

    try
        [y_, M_, t_, esti_limb_length] = L1WAWS(x, B, mu, init_pose,...
            C, L, param_test, param );
        esti_Ys{i} = y_;
        esti_Ms{i} = M_;
        esti_Ts{i} = t_;
    catch
        esti_Ys{i} = init_pose;   
    end

    
    err = mse(y_, y) * scale(i);
    err2 = mse(init_pose, y) * scale(i);
    disp([err err2]);
end
