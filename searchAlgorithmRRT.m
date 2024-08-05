function [path, cost, tree, final_child] = searchAlgorithmRRT(sp, rrtConf, SHOW)
    tree = {};
    
    if SHOW
        drawInit(sp.start_conf, sp.goal_conf, sp)
    end

    %{
Respectively:
- Configuration,
- Parent,
- Random node that is sampled (this is for showing/debugging remove in
production.
    %}
    prevTreeSize = 0;
    graphTree = {sp.start_conf, 0, {}, sp.start_conf};
    for i = 1:rrtConf.numOfNodes
        % If size has changed (which means a node is added),
        % then try to create a direct path from this node to the goal.
        if prevTreeSize ~= size(graphTree, 1)
            [path, cost] = directExpansion(sp, realmax, graphTree{end, 1}, sp.goal_conf);
            if ~isempty(path)
                [prevPath, prevCost] = backtrackPath(sp, graphTree);

                path = [prevPath, path];
                cost = prevCost + cost;

                tree = graphTree;
                final_child = tree{end, 1};
                return;
            end
        end

        if rand(1) > rrtConf.pOfGoal
            randomConfig = randomConf(sp);
            chosenGoal = false;
        else
            randomConfig = sp.goal_conf;
            chosenGoal = true;
        end

        
        prevTreeSize = size(graphTree, 1);
        graphTree = updateTreeRRT(sp, rrtConf, graphTree, randomConfig);

        % Show the plot if SHOW flag is set.
        if SHOW && prevTreeSize ~= size(graphTree, 1)
            drawConfig(graphTree{end, 1}, sp, 'b');
            pause
        end

        % If goal was sampled and tree size has changed,
        % which means a path is found.
        if chosenGoal && prevTreeSize ~= size(graphTree, 1) && ...
                getHeuristic(sp.typeOfHeuristic, graphTree{end, 1}, sp) < 1
            [path, cost] = backtrackPath(sp, graphTree);
            tree = graphTree;
            final_child = tree{end, 1};
            return;
        end
    end


    prevTreeSize = size(graphTree, 1);
    graphTree = updateTreeRRT(sp, rrtConf, graphTree, sp.goal_conf);
    if prevTreeSize ~= size(graphTree, 1) && getHeuristic(sp.typeOfHeuristic, graphTree{end, 1}, sp) < 1
        [path, cost] = backtrackPath(sp, graphTree);
        tree = graphTree;
        final_child = tree{end, 1};
    else
        path = {};
        cost = -1;
    end
end

function heuristic = getHeuristicBridge(sp, conf1, conf2)
    sp.goal_conf = conf2;
    heuristic = getHeuristic(sp.typeOfHeuristic, conf1, sp);
end

function graphTree = updateTreeRRT(sp, rrtConf, graphTree, randomConfig)
    closestParent = 1;
    closestDistance = getHeuristicBridge(sp, graphTree{1, 1}, randomConfig);
    % closestDistance = totalStep(graphTree{1, 1}, randomConfig, sp);

    for j = 2:size(graphTree, 1)
        distance = getHeuristicBridge(sp, graphTree{j, 1}, randomConfig);
        % distance = totalStep(graphTree{j, 1}, randomConfig, sp);
        if distance < closestDistance
            closestParent = j;
            closestDistance = distance;
        end
    end
    
    [directPath, ~] = directExpansion(sp, rrtConf.stepSize, graphTree{closestParent, 1}, randomConfig);

    if ~isempty(directPath)
        % Draws closest parent configuration that is chosen.
        % drawConfig(graphTree{closestParent, 1}, sp, 'k');
        
        newNode = directPath{end};
        graphTree = [graphTree; {newNode, closestParent, directPath, randomConfig}];
    end
end
