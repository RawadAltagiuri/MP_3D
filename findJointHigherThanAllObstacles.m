% Returns the index of the last joint above the obstacles
%
% Input:
%  'current_conf': the current configuration
%  'sp': search problem as indicated in 'MotionPlannerSolution.m'
%
% Output:
%  'index': the index of the last joint above the obstacles


function [index] = findJointHigherThanAllObstacles(sp)
    maxObstacleHeight = sp.plane_z;
    for i = 1:size(sp.obstacles, 1)
        obs = sp.obstacles(i, :);
        if obs(3)-obs(5) < maxObstacleHeight
            maxObstacleHeight = obs(3)-obs(5);
        end
    end

    currDepth = 0;

    % Default index value based on the rotation of the base
    index = 2;
    if sp.baseRotate == true
        index = 1;
    end
    for i = 1:size(sp.design, 1)
        currDepth = currDepth + sp.design(i, 1);
        if currDepth < maxObstacleHeight
            index = i+1;
        end
    end

end