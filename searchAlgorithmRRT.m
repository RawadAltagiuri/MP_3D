function [path, cost, tree, final_child] = searchAlgorithmRRT(sp, rrtConf, SHOW)
    path = {};
    cost = -1;
    tree = {};
    final_child = [];
    
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
                final_child = size(tree, 1);
                return;
            end
        end

        randomConfig = randomConf(sp);

        prevTreeSize = size(graphTree, 1);
        graphTree = updateTreeRRT(sp, rrtConf, graphTree, randomConfig);

        if prevTreeSize < size(graphTree, 1) && getHeuristic(sp, graphTree{end, 1}, sp.goal_conf) < sp.goalRegion
            cost = costOfPath(sp, path);
            break;
        end

        % Show the plot if SHOW flag is set.
        if SHOW && prevTreeSize ~= size(graphTree, 1)
            drawConfig(graphTree{end, 1}, sp, 'b');
            pause
        end
    end

    tree = graphTree;
end


function graphTree = updateTreeRRT(sp, rrtConf, graphTree, randomConfig)
    closestParent = 1;
    closestDistance = getHeuristic(sp, graphTree{1, 1}, randomConfig);
    % closestDistance = totalStep(graphTree{1, 1}, randomConfig, sp);

    for j = 2:size(graphTree, 1)
        distance = getHeuristic(sp, graphTree{j, 1}, randomConfig);
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
