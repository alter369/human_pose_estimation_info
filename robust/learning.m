% Learning mean pose and bases from training data

addpath(genpath('spams-matlab'));

% Parameters for spams (learning)
param_learn.K = 50;               % Number of bases
param_learn.lambda = 0.01;
param_learn.iter = 200;

load('HumanEva/S1/New_Box_1.mat');
tposes = tposes';

% mean-center the pose
for i=1:size(tposes,2)
    pose = reshape(tposes(:,i), 3, []);
    center = mean(pose,2);
    pose = pose - repmat(center, 1, size(pose, 2));
    tposes(:,i) = pose(:);
end
scale = sqrt(sum(tposes.*tposes,1));
tposes = tposes ./ repmat(scale, size(tposes,1), 1);
mu = zeros(size(tposes,1), 1);
train = tposes - repmat(mu, 1, size(tposes,2));
B = mexTrainDL(train,param_learn);

% intial pose for optimization
init_pose = mean(tposes, 2);

% save learning results for future use
save('model', 'B', 'mu', 'init_pose');
