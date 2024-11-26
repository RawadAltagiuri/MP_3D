function [neighbours] = findNeighbours(sp, graphTree, config, radius)
    % findNeighbours
    % Finds the neighbours of a given configuration in a tree that is
    % given with a specified radius.
    % Output is a set of indicies into the tree, of neighbours.
    %
    % Inputs:
    % - graphTree (Nxk) cell array where N is the number of nodes
    %  and k is the number of attributes of each node.
    % - config (sp.j x 3) target configuration.
    % - radius (1x1) a radius of neighbours to search for. Its unit is as given in
    % sp.metric.
    %
    % Outputs:
    % - neighbours (nx2) where n is the nodes that are less than,
    %  or equal to $radius amount of distance from $config. C1 is the
    % index and C2 is the cost.
    neighbours = cell(size(graphTree, 1), 2);
    curElem = 1;
    for i = 1:size(graphTree, 1)
        cost = calculateCost(sp, graphTree{i, 1}, config);
        if cost < radius
            neighbours(curElem, :) = {i, cost};
            curElem = curElem + 1;
        end
    end
    neighbours(curElem:end) = [];
end

