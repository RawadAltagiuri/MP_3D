function intersects = collisionCheck(conf,op)
    obstacles = op.obstacles;
    nObstacles = size(obstacles,1);

    nodes = solveForwardKinematics_3D(conf,op.home_base);
    nUsedLinks = size(conf,1);
    
    nUsedNodes = nUsedLinks + 1;

    intersects = false;
    for i = 1 : nUsedNodes - 1 
        for j = 1 : nObstacles
            if veccol(nodes(i,:),nodes(i+1,:),obstacles(j,:))
                intersects = true;
                return;
            end
        end
    end
end