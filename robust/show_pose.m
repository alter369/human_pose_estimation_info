function show_pose(poses)
    npose = size(poses, 1);
    njoints = size(poses, 2) / 3;
    parent = [2 1 2 3 4 2 6 7 2 9 10 11 9 13 14];
    for i=1:10:npose
        pose = reshape(poses(i, :), 3, njoints);
        for j=1:njoints
            plot3(pose(1, j),pose(2, j),pose(3, j), 'r*','MarkerSize', 15);
            hold on;
        end
        
        for j=1:njoints
            plot3([pose(1, j) pose(1, parent(j))],...
                   [pose(2, j) pose(2, parent(j))],...
                   [pose(3, j) pose(3, parent(j))],'g-');
        end
        
        hold off;
        pause;
    end
end

