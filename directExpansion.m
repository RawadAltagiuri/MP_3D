function [path, cost] = directExpansion(start_conf, end_conf, sp)
    sp.goal_conf = end_conf;

    node1.path = start_conf;
    node1.g = 0;
    
    % Retract first.
    [path, cost] = pathToRetraction(sp, start_conf, sum(end_conf(:, 3)));

    node1.path = path{end};
    node1.g = cost;
    node1.h = getHeuristic(sp.typeOfHeuristic, node1.path, sp);
    node1.f = calculateCostBasedOnAlgorithm(node1.g, node1.h, sp.typeOfAlg);
    
    % Does not retract first
    % path = {start_conf};
    % node1.path = start_conf;
    % node1.g = 0;
    % node1.h = getHeuristic(sp.typeOfHeuristic, node1.path, sp);
    % node1.f = calculateCostBasedOnAlgorithm(node1.g, node1.h, sp.typeOfAlg);

    while node1.h > 1
        node1 = greedyExpand(node1, sp);

        if isempty(node1) || collisionCheck(node1.path, sp) 
            
           % Controls if obstacles in front of the random node is considered or not (Fix this). 
           % path = {};
           % cost = -1;
           return;
        end

        cost = node1.g;
        node1.path = node1.path(:, end - 2:end);
        path = [path, node1.path];
    end
end
