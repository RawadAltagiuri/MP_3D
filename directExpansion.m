function [path, cost] = directExpansion(start_conf, end_conf, sp)
    sp.goal_conf = end_conf;

    node1.path = start_conf;
    node1.g = 0;
    node1.h = getHeuristic(sp.typeOfHeuristic, node1.path, sp);
    node1.f = calculateCostBasedOnAlgorithm(node1.g, node1.h, sp.typeOfAlg);
    
    path = {node1.path};
    cost = 0;
    while node1.h > 1
        if collisionCheck(node1.path, sp)
           path = {};
           cost = -1;
           return;
        end
        
        node1 = greedyExpand(node1, sp);
        if isempty(node1)
            path = {};
            cost = -1;
            return;
        end

        cost = node1.g;
        node1.path = node1.path(:, end - 2:end);
        path = [path, node1.path];
    end
end
