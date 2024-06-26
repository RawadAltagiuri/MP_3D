function [path, cost] = searchAlgorithmRRT(sp, rrtConf)

    graphTree = {sp.start_conf, 0};
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

        if chosenGoal && prevTreeSize ~= size(graphTree, 1) && ...
                getHeuristic(sp.typeOfHeuristic, graphTree{end, 1}, sp) < 1
            [path, cost] = backtractPath(sp, graphTree);
            return;
        end
    end

    prevTreeSize = size(graphTree, 1);
    graphTree = updateTree(sp, rrtConf, graphTree, randomConfig);
    if chosenGoal && prevTreeSize ~= size(graphTree, 1)
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

    [directPath, ~] = directExpansion(graphTree{closestParent, 1}, randomConfig, sp);
    if ~isempty(directPath)
        newNodeIndex = 0;
        for j = 1:min(size(directPath, 2), rrtConf.stepSize)
            newNodeIndex = newNodeIndex + 1;
        end
        newNode = directPath{newNodeIndex};
        graphTree = [graphTree; {newNode, closestParent}];
    end
end

