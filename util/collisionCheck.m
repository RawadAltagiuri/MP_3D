function intersects = collisionCheck(conf, sp)
    nodes = solveForwardKinematics_3D(conf, sp.home_base);

    obstacles = sp.obstacles;
    nObstacles = size(obstacles,1);

    for nUsedLinks = size(conf,1):-1:1
        if conf(nUsedLinks,3)==0
            nUsedLinks = nUsedLinks-1;
            break;
        end
    end
    
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