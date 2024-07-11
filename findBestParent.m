function [BestParent, child] = findBestParent(child, nearest_node, nodes_map, sp)
    potential_parents = [];
    lowest_g = Inf;
    BestParent = [];

    keys = nodes_map.keys;
    for i = 1:length(keys)
        parent_key = keys{i};
        grandparent_node = nodes_map(parent_key);
        parent_node.path = eval(parent_key);
        parent_node.g = grandparent_node.g + calculateCost(parent_node.path(:, end-2:end), grandparent_node.path(:, end-2:end), sp.home_base);
        parent_node.h = getHeuristic(sp.typeOfHeuristic, parent_node.path(:, end-2:end), sp);
        parent_node.f = calculateCostBasedOnAlgorithm(parent_node.g, parent_node.h, sp.typeOfAlg);
        
        % Temporarily set sp.random_conf to the parent_node's configuration to use greedyExpand
        original_random_conf = sp.random_conf;
        sp.random_conf = parent_node.path(:, end-2:end);
        
        greedyChildren = greedyExpand(child, sp);
        
        % Restore the original random_conf
        sp.random_conf = original_random_conf;
        
        for j = 1:length(greedyChildren)
            if isequal(greedyChildren(j).path(:, end-2:end), parent_node.path(:, end-2:end))
                potential_parents = [potential_parents, parent_node];
                if parent_node.g < lowest_g
                    lowest_g = parent_node.g;
                    BestParent = parent_node;
                end
                break; % Found a match, no need to check other children
            end
        end
    end
    
    if isempty(BestParent)
        BestParent = nearest_node; % Fallback to nearest_node if no better parent is found
    else
        %recalculate the g, h, and f values of the child
        child.g = BestParent.g + calculateCost(child.path(:, end-2:end), BestParent.path(:, end-2:end), sp.home_base);
        child.h = getHeuristic(sp.typeOfHeuristic, child.path(:, end-2:end), sp);
        child.f = calculateCostBasedOnAlgorithm(child.g, child.h, sp.typeOfAlg);
    end
end