function [path, cost] = searchAlgorithmRRT_star(sp, rrtStarConf)
    graphTree = {sp.start_conf, 0, 0};
    for i = 1:rrtStarConf.numOfNodes
        if rand(1) > rrtStarConf.pOfGoal
            randomConfig = randomConf(sp);
        else
            randomConfig = sp.goal_conf;
        end

        graphTree = updateTree(sp, rrtStarConf, graphTree, randomConfig);
    end

    prevTreeSize = size(graphTree, 1);
    graphTree = updateTree(sp, rrtStarConf, graphTree, sp.goal_conf);
    if prevTreeSize ~= size(graphTree, 1)
        [path, cost] = backtractPath(sp, graphTree);
    else
        path = {};
        cost = -1;
    end
end

function heuristic = getHeuristicBridge(sp, conf1, conf2)
    sp.goal_conf = conf2;
    heuristic = getHeuristic(sp.typeOfHeuristic, conf1, sp);
end

function [fullPath, totalCost] = backtractPath(sp, graphTree)
    totalCost = 0;
    curNode = graphTree(end, :);
    fullPath = {};
    paths = {};
    while curNode{2} > 0
        parentNode = graphTree(curNode{2}, :);
        [path, cost] = directExpansion(parentNode{1}, curNode{1}, sp);

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
        if totalStep(graphTree{i, 1}, randomConfig, sp) < rrtConf.neighbourSize
            [path, cost] = directExpansion(graphTree{i, 1}, randomConfig, sp);
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

    graphTree = [graphTree; {randomConfig, minNeighbour, minCost + graphTree{minNeighbour, 3}}];
    realNeighbours = [];
    for i = 1:size(neighbours, 1)
        if neighbours(i, 1) == minNeighbour
            continue;
        end
        [path, cost] = directExpansion(randomConfig, graphTree{neighbours(i, 1), 1}, sp);
        if ~isempty(path) && size(path, 2) < rrtConf.neighbourSize
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

    [directPath, cost] = directExpansion(graphTree{closestParent, 1}, randomConfig, sp);
    if ~isempty(directPath)
        newNodeIndex = 0;
        for j = 1:min(size(directPath, 2), rrtConf.stepSize)
            newNodeIndex = newNodeIndex + 1;
        end
        newNode = directPath{newNodeIndex};
        graphTree = [graphTree; {newNode, closestParent, cost + graphTree{closestParent, 3}}];
    end
end

