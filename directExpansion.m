function [path, cost] = directExpansion(sp, step_size, start_conf, end_conf)
    sp.goal_conf = end_conf;

    node1.path = start_conf;
    node1.g = 0;
    
    % Retract first.
    [path, cost] = pathToRetraction(sp, start_conf, sum(end_conf(:, 3)));

    if (size(path, 2) - 1) >= step_size
        path = path(1:step_size + 1);
        cost = costOfPath(sp, path);
        return;
    end

    node1.path = path{end};
    node1.g = cost;
    node1.h = getHeuristic(sp.typeOfHeuristic, node1.path, sp);
    node1.f = calculateCostBasedOnAlgorithm(node1.g, node1.h, sp.typeOfAlg);
    
    % Does not retract first
    % path = {start_conf};
    % cost = 0;
    % node1.path = start_conf;
    % node1.g = 0;
    % node1.h = getHeuristic(sp.typeOfHeuristic, node1.path, sp);
    % node1.f = calculateCostBasedOnAlgorithm(node1.g, node1.h, sp.typeOfAlg);

    while node1.h > sp.heuristicLimit && (size(path, 2) - 1) < step_size
        node1 = greedyExpand(node1, sp);

        if isempty(node1) || collisionCheck(node1.path(:, end - 2:end), sp) 
            
           % Controls if obstacles in front of the random node is considered or not (Fix this). 
           path = {};
           cost = -1;
           return;
        end

        node1.path = node1.path(:, end - 2:end);
        cost = node1.g;
        path = [path, node1.path];
    end

    cost = cost + calculateCost(path{end}, end_conf, sp.home_base);
    if ~isequal(path{end}, end_conf)
        path = [path, end_conf];
    end
end
