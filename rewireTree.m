function [nodes_map] = rewireTree(nodes_map, sp)
    keys = nodes_map.keys;
    for i = 1:length(keys)
        child_key = keys{i};
        child_node.path = eval(child_key);
        child_node.g = nodes_map(child_key).g + calculateCost(child_node.path(:, end-2:end), nodes_map(child_key).path(:, end-2:end), sp.home_base);
        child_node.h = getHeuristic(sp.typeOfHeuristic, child_node.path(:, end-2:end), sp);
        child_node.f = calculateCostBasedOnAlgorithm(child_node.g, child_node.h, sp.typeOfAlg);
        % nearest_node is the parent of the child_node
        nearest_node = nodes_map(mat2str(child_node.path(:, end-2:end)));
        
        [BestParent, child_node] = findBestParent(child_node, nearest_node, nodes_map, sp);

        if isequal(BestParent.path, nearest_node.path)
            continue; % Skip if the parent is the same as the nearest node
        end
        
        % Check if BestParent is an ancestor of child_node to prevent cycles
        if ~isAncestor(BestParent, child_node, nodes_map, sp) && ~isAncestor(child_node, BestParent, nodes_map, sp)
            % Update the child node in the map if the parent has changed and no cycle is created.
            if ~isequal(BestParent.path, eval(child_key)) || ~isequal(BestParent.path, nearest_node.path)
                disp("Rewiring")
                nodes_map(child_key) = BestParent;
            end
        end
    end
    disp("Rewiring Done");
end

function is_ancestor = isAncestor(possible_ancestor, child_node, nodes_map, sp)
    %we iterate from the child node to the root node, if we find the possible ancestor, then it is an ancestor
    path = child_node.path;
    counter = 0;
    while ~isequal(path, sp.start_conf)
        if isequal(path, possible_ancestor.path)
            is_ancestor = true;
            return;
        end
        parent = nodes_map(mat2str(path(:, end-2:end)));
        %if ~isempty(path) && isequal(path, parent.path)
         %   is_ancestor = true;
        %end
        path = parent.path;
        counter = counter + 1;
    end
    is_ancestor = false;
    
end