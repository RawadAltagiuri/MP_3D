function [path, cost] = searchAlgorithmRRT(sp, rrtConf, SHOW)
    if SHOW
        drawInit(sp.start_conf, sp.goal_conf, sp)
    end

    graphTree = {sp.start_conf, 0, sp.start_conf};
    for i = 1:rrtConf.numOfNodes
        if rand(1) > rrtConf.pOfGoal
            randomConfig = randomConf(sp);
            chosenGoal = false;
        else
            randomConfig = sp.goal_conf;
            chosenGoal = true;
        end

        prevTreeSize = size(graphTree, 1);
        graphTree = updateTree(sp, rrtConf, graphTree, randomConfig);
        if SHOW && prevTreeSize ~= size(graphTree, 1)
            drawConfig(graphTree{end, 1}, sp, 'b');
            pause
        end

        if chosenGoal && prevTreeSize ~= size(graphTree, 1) && ...
                getHeuristic(sp.typeOfHeuristic, graphTree{end, 1}, sp) < 1
            [path, cost] = backtractPath(sp, rrtConf, graphTree);
            return;
        end
    end

    prevTreeSize = size(graphTree, 1);
    graphTree = updateTree(sp, rrtConf, graphTree, sp.goal_conf);
    if prevTreeSize ~= size(graphTree, 1) && getHeuristic(sp.typeOfHeuristic, graphTree{end, 1}, sp) < 1
        [path, cost] = backtractPath(sp, rrtConf, graphTree);
    else
        path = {};
        cost = -1;
    end
end

function heuristic = getHeuristicBridge(sp, conf1, conf2)
    sp.goal_conf = conf2;
    heuristic = getHeuristic(sp.typeOfHeuristic, conf1, sp);
end

function [fullPath, totalCost] = backtractPath(sp, rrtConf, graphTree)
    totalCost = 0;
    curNode = graphTree(end, :);
    fullPath = {};
    paths = {};
    while curNode{2} > 0
        parentNode = graphTree(curNode{2}, :);
        [path, cost] = directExpansion(sp, rrtConf.stepSize, parentNode{1}, curNode{1});
        
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
    closestParent = 1;
    closestDistance = getHeuristicBridge(sp, graphTree{1, 1}, randomConfig);
    for j = 2:size(graphTree, 1)
        distance = getHeuristicBridge(sp, graphTree{j, 1}, randomConfig);
        if distance < closestDistance
            closestParent = j;
            closestDistance = distance;
        end
    end

    % Draws random configuration that is sampled.
    % drawConfig(randomConfig, sp, 'k');
    
    [directPath, ~] = directExpansion(sp, rrtConf.stepSize, graphTree{closestParent, 1}, randomConfig);

    if ~isempty(directPath)
        newNodeIndex = 1;
        for j = 1:min(size(directPath, 2) - 1, rrtConf.stepSize)
            newNodeIndex = newNodeIndex + 1;
        end
        newNode = directPath{newNodeIndex};
        graphTree = [graphTree; {newNode, closestParent, randomConfig}];
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
    for i = 1:size(graphTree, 1)
        drawConfig(graphTree{i, 1}, sp, 'r');
        drawConfig(graphTree{i, 3}, sp, 'k');
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

