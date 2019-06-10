function [ output_args ] = display_pose( pose_esti, pose_gnd )
    njoints = length(pose_esti) / 3;
    pose_esti = reshape(pose_esti, 3, []);
    pose_gnd = reshape(pose_gnd, 3, []);
    
    figure; hold on;
    for i=1:njoints
        plot3(pose_esti(1,i), pose_esti(2,i), pose_esti(3,i), 'r');
        plot3(pose_gnd(1,i), pose_gnd(2,i), pose_gnd(3,i), 'g');
    end
    
    parent = [2 1 2 3 4 2 6 7 2 9 10 11 9 13 14];
    for i=1:njoints
        plot3([pose_esti(1,i) pose_esti(1, parent(i))], [pose_esti(2,i) pose_esti(2, parent(i))], [pose_esti(3,i) pose_esti(3, parent(i))], 'r-');
        plot3([pose_gnd(1,i) pose_gnd(1, parent(i))], [pose_gnd(2,i) pose_gnd(2, parent(i))], [pose_gnd(3,i) pose_gnd(3, parent(i))], 'g-');
    end
end

