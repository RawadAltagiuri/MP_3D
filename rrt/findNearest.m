function [treeIndex, minCost] = findNearest(sp, graphTree, config)
    % findNearest
    % Finds the node that is nearest in the tree to 'config' and returns its
    % index.
    % As with other cost calcualtions, the distance metric is given in 'sp'.
    %
    % Inputs:
    % - graphTree (Nxk) cell array where N is the number of nodes
    %  and k is the number of attributes of each node.
    % - config (sp.j x 3) target configuration
    %
    % Outputs:
    % - treeIndex (1x1) index into the $graphTree identifying the closest node.
    % - minCost (1x1) a cost to go from closest to $config.
    if isempty(graphTree)
        treeIndex = -1;
        minCost = -1;
        return;
    end

    treeIndex = 1;
    minCost = calculateCost(sp, graphTree{1, 1}, config);
    for i = 2:size(graphTree, 1)
        cost = calculateCost(sp, graphTree{i, 1}, config);
        if cost < minCost
            treeIndex = i;
            minCost = cost;
        end
    end
end
