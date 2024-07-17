function [path, cost, tree, final_child] = searchAlgorithmRRT_star(sp, rrtStarConf, SHOW)
    tree = {};
    
    if SHOW
        drawInit(sp.start_conf, sp.goal_conf, sp);
    end

    prevTreeSize = 0;
    bestPath = [];
    bestCost = 10000;
    graphTree = {sp.start_conf, 0, 0, sp.start_conf};
    for i = 1:rrtStarConf.numOfNodes
        % If size has changed (which means a node is added),
        % then try to create a direct path from this node to the goal.
        if prevTreeSize ~= size(graphTree, 1)
            [path, cost] = directExpansion(sp, 10000, graphTree{end, 1}, sp.goal_conf);
            if ~isempty(path)
                [prevPath, prevCost] = backtrackPath(sp, rrtStarConf, graphTree);
                path = [prevPath, path];
                cost = cost + prevCost;

                if cost < bestCost
                    bestPath = path;
                    bestCost = cost;
                    final_child = graphTree{end, 1};
                end
            end
        end

        if rand(1) > rrtStarConf.pOfGoal
            randomConfig = randomConf(sp);
        else
            randomConfig = sp.goal_conf;
        end

        prevTreeSize = size(graphTree, 1);
        graphTree = updateTree(sp, rrtStarConf, graphTree, randomConfig);

        if SHOW && prevTreeSize ~= size(graphTree, 1)
            drawConfig(graphTree{end, 1}, sp, 'b');
            pause
        end
    end

    if ~isempty(bestPath)
        path = bestPath;
        cost = bestCost;
        tree = graphTree;
        return;
    end

    prevTreeSize = size(graphTree, 1);
    graphTree = updateTree(sp, rrtStarConf, graphTree, sp.goal_conf);
    if prevTreeSize ~= size(graphTree, 1)
        [path, cost] = backtrackPath(sp, rrtStarConf, graphTree);
    else
        path = {};
        cost = -1;
    end

    tree = graphTree;
end

function heuristic = getHeuristicBridge(sp, conf1, conf2)
    sp.goal_conf = conf2;
    heuristic = getHeuristic(sp.typeOfHeuristic, conf1, sp);
end

function [fullPath, totalCost] = backtrackPath(sp, rrtConf, graphTree)
    totalCost = 0;
    curNode = graphTree(end, :);
    fullPath = {};
    paths = {};
    while curNode{2} > 0
        parentNode = graphTree(curNode{2}, :);
        [path, cost] = directExpansion(sp, rrtConf.stepSize, parentNode{1}, curNode{1});

        if isempty(path)
            fullPath = {};
            totalCost = -1;
            return;
        end

        paths = [paths, {path}];
        totalCost = totalCost + cost;
        curNode = parentNode;
    end
    paths = flip(paths, 2);
    for i = 1:size(paths, 2)
        fullPath = [fullPath, paths{i}];
    end
end

function graphTree = updateTree(sp, rrtConf, graphTree, randomConfig)
    neighbours = [];
    for i = 1:size(graphTree, 1)
        if calculateCost(graphTree{i, 1}, randomConfig, sp.home_base) < rrtConf.neighbourSize
            [path, cost] = directExpansion(sp, rrtConf.stepSize, graphTree{i, 1}, randomConfig);
            if ~isempty(path) && size(path, 2) < rrtConf.neighbourSize
                neighbours = [neighbours; i, cost];
            end
        end
    end

    if isempty(neighbours)
        graphTree = updateTreeRRT(sp, rrtConf, graphTree, randomConfig);
        return;
    end

    minNeighbour = neighbours(1, 1);
    minCost = neighbours(1, 2);
    for i = 2:size(neighbours, 1)
        if neighbours(i, 2) < minCost
            minCost = neighbours(i, 2);
            minNeighbour = neighbours(i, 1);
        end
    end

    graphTree = [graphTree; {randomConfig, minNeighbour, minCost + graphTree{minNeighbour, 3}, randomConfig}];
    realNeighbours = [];
    for i = 1:size(neighbours, 1)
        if neighbours(i, 1) == minNeighbour
            continue;
        end
        [path, cost] = directExpansion(sp, rrtConf.stepSize, randomConfig, graphTree{neighbours(i, 1), 1});
        if ~isempty(path)
            realNeighbours = [realNeighbours; neighbours(i, 1), cost + graphTree{end, 3}];
        end
    end

    for i = 1:size(realNeighbours, 1)
        if realNeighbours(i, 2) < graphTree{realNeighbours(i, 1), 3}
            graphTree{realNeighbours(i, 1), 2} = size(graphTree, 1);
        end
    end
end

function graphTree = updateTreeRRT(sp, rrtConf, graphTree, randomConfig)
    closestParent = 1;
    closestDistance = getHeuristicBridge(sp, graphTree{1, 1}, randomConfig);
    for i = 2:size(graphTree, 1)
        distance = getHeuristicBridge(sp, graphTree{i, 1}, randomConfig);
        if distance < closestDistance
            closestParent = i;
            closestDistance = distance;
        end
    end

    [directPath, cost] = directExpansion(sp, rrtConf.stepSize, graphTree{closestParent, 1}, randomConfig);
    if ~isempty(directPath)
        newNodeIndex = 1;
        for j = 1:min(size(directPath, 2) - 1, rrtConf.stepSize)
            newNodeIndex = newNodeIndex + 1;
        end
        newNode = directPath{newNodeIndex};
        graphTree = [graphTree; {newNode, closestParent, cost + graphTree{closestParent, 3}, randomConfig}];
    end
end

function drawInit(startConfig, goalConfig, sp)
    f = figure;

    grayRobotColor = '#569c69';

    home_base = sp.home_base;
    n_obstacles = size(sp.obstacles, 1);

    clf;
    hold on;
    axis equal;
    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    xlim([-400 400]);
    ylim([-400 400]);
    zlim([-100 1000]);
    plot3(home_base(1),home_base(2),home_base(3),'--gs','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','b'); %draw home
    [startConf,~] = solveForwardKinematics_3D(startConfig,home_base,false);
    [goalConf,~] = solveForwardKinematics_3D(goalConfig,home_base,false);

    n_joints = size(startConfig,1);

    for i=2:1:n_joints+1
        plot3([startConf(i-1,1),startConf(i,1)],[startConf(i-1,2),startConf(i,2)],[startConf(i-1,3),startConf(i,3)],'-o','Color','r', 'LineWidth', 1.5);
    end
    for i=2:1:n_joints+1
        plot3([goalConf(i-1,1),goalConf(i,1)],[goalConf(i-1,2),goalConf(i,2)],[goalConf(i-1,3),goalConf(i,3)],'-o','Color','g', 'LineWidth', 1.5);
    end    

    for i = 1:1:n_obstacles
        [X, Y, Z] = cylinder(sp.obstacles(i, 4), 10);
        X = X + sp.obstacles(i, 1);
        Y = Y + sp.obstacles(i, 2);
        Z = Z * -sp.obstacles(i, 5) + sp.obstacles(i, 3);

        surf(X, Y, Z, 'FaceColor', 'w', 'EdgeColor', 'none');
        grayColor = '#778079';
        plot3(X,Y,Z,'Color',grayColor);
        th = 0:pi / 50:2 * pi;
        xunit = sp.obstacles(i, 4) * cos(th) + sp.obstacles(i, 1);
        yunit = sp.obstacles(i, 4) * sin(th) + sp.obstacles(i, 2);
        zunit = 0 * th + sp.obstacles(i, 3);

        plot3(xunit, yunit, zunit,'Color',grayColor);
        plot3(xunit, yunit, (zunit-sp.obstacles(i,5)),'Color',grayColor);
    end
    
    f.CurrentAxes.ZDir = 'Reverse';
    cameratoolbar('SetCoordSys','x');
    view(60, 30)
end

function drawTree(graphTree, sp)
    for i = 2:size(graphTree, 1)
        drawConfig(graphTree{i, 1}, sp, 'b');
        drawConfig(graphTree{i, 4}, sp, 'k');
        pause;
    end
end

function drawConfig(config, sp, color)
    n_joints = size(config, 1);
    [robot_CC,~] = solveForwardKinematics_3D(config, sp.home_base, false); %solve the forward kinematics for a given robot configuration

    % draws the soft robot
    for i=2:1:n_joints+1
        plot3([robot_CC(i-1,1),robot_CC(i,1)],[robot_CC(i-1,2),robot_CC(i,2)],[robot_CC(i-1,3),robot_CC(i,3)],'-o','Color',color, 'LineWidth', 1.5);
    end

    % f.CurrentAxes.ZDir = 'Reverse';
    % cameratoolbar('SetCoordSys','x');
    % view(60, 30)
end



