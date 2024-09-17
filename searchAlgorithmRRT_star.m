    
function [path, cost, tree, final_child] = searchAlgorithmRRT_star(sp, rrtStarConf, SHOW)
    path = {};
    cost = -1;
    tree = {};
    final_child = [];
    
    if SHOW
        drawInit(sp.start_conf, sp.goal_conf, sp);
    end

    prevTreeSize = 0;
    bestPath = [];
    bestCost = realmax;
    graphTree = {sp.start_conf, 0, {}, 0, sp.start_conf};
    for i = 1:rrtStarConf.numOfNodes
        % If size has changed (which means a node is added),
        % then try to create a direct path from this node to the goal.
        if prevTreeSize ~= size(graphTree, 1)
            [path, cost] = directExpansion(sp, realmax, graphTree{end, 1}, sp.goal_conf);
            if ~isempty(path)
                [prevPath, prevCost] = backtrackPath(sp, graphTree);
                
                path = [prevPath, path];
                cost = prevCost + cost;

                if cost <= bestCost
                    bestPath = path;
                    bestCost = cost;
                    final_child = size(graphTree, 1);
                end
            end
        end

        randomConfig = randomConf(sp);

        prevTreeSize = size(graphTree, 1);
        graphTree = updateTreeRRT_star(sp, rrtStarConf, graphTree, randomConfig);

        if SHOW && prevTreeSize ~= size(graphTree, 1)
            drawConfig(graphTree{end, 1}, sp, 'b');
            pause
        end
    end

    if ~isempty(bestPath)
        path = bestPath;
        cost = bestCost;
    end
    tree = graphTree;
end


function graphTree = updateTreeRRT_star(sp, rrtConf, graphTree, randomConfig)
    neighbours = [];
    for i = 1:size(graphTree, 1)
        % cost = calculateCost(sp, graphTree{i, 1}, randomConfig);
        cost = calculateCost(sp, graphTree{i, 1}, randomConfig);
        if cost < rrtConf.neighbourSize
            neighbours = [neighbours; i, cost + graphTree{i, 4}];
        end
    end

    if isempty(neighbours)
        graphTree = updateTreeRRT(sp, rrtConf, graphTree, randomConfig);
        return;
    end

    neighbours = sortrows(neighbours, 2);
    minNeighbour = 0;
    for i = 1:size(neighbours, 1)
        [path, cost] = directExpansion(sp, realmax, graphTree{neighbours(i, 1), 1}, randomConfig);
        if ~isempty(path)
            graphTree = [graphTree; {path{end}, neighbours(i, 1), path, cost + graphTree{neighbours(i, 1), 4}, randomConfig}];
            minNeighbour = neighbours(i, 1);
            break;
        end
    end

    if minNeighbour == 0
        graphTree = updateTreeRRT(sp, rrtConf, graphTree, randomConfig);
        return;
    end

    realNeighbours = {};
    for i = 1:size(neighbours, 1)
        if neighbours(i, 1) == minNeighbour
            continue;
        end
        [path, cost] = directExpansion(sp, realmax, randomConfig, graphTree{neighbours(i, 1), 1});
        if ~isempty(path)
            realNeighbours(end + 1, :) = {neighbours(i, 1), path, cost + graphTree{end, 4}};
        end
    end

    for i = 1:size(realNeighbours, 1)
        if realNeighbours{i, 3} < graphTree{realNeighbours{i, 1}, 4}
            graphTree{realNeighbours{i, 1}, 2} = size(graphTree, 1);
            graphTree{realNeighbours{i, 1}, 3} = realNeighbours{i, 2};
            graphTree{realNeighbours{i, 1}, 4} = realNeighbours{i, 3};
        end
    end
end

function graphTree = updateTreeRRT(sp, rrtConf, graphTree, randomConfig)
    closestParent = 1;
    closestDistance = getHeuristic(sp, graphTree{1, 1}, randomConfig);
    for i = 2:size(graphTree, 1)
        distance = getHeuristic(sp, graphTree{i, 1}, randomConfig);
        if distance < closestDistance
            closestParent = i;
            closestDistance = distance;
        end
    end

    [directPath, cost] = directExpansion(sp, rrtConf.stepSize, graphTree{closestParent, 1}, randomConfig);
    if ~isempty(directPath)
        newNode = directPath{end};
        graphTree = [graphTree; {newNode, closestParent, directPath, cost + graphTree{closestParent, 4}, randomConfig}];
    end
end




