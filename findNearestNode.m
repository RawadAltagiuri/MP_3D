function [nearest_node] =  findNearestNode(nodes_map, random_conf, sp) 
    %initialize the minimum distance to be infinity
    min_distance = inf;
    %initialize the nearest node to be empty
    nearest_node = [];
    %get the keys of the nodes_map
    KeysSet = keys(nodes_map);
    for i = 1:length(KeysSet)
        %get the configuration matrix of the node, the configuration matrix is the key of the nodes_map
        node_conf.path = eval(KeysSet{i});
        node_conf.h = getHeuristicRrt(sp.typeOfHeuristic, node_conf.path(:, end-2:end), sp, random_conf);
        node_conf.g = calculateCost(nodes_map(KeysSet{i}).path(:, end-2:end), node_conf.path, sp.home_base);
        node_conf.f = calculateCostBasedOnAlgorithm(node_conf.g, node_conf.h, sp.typeOfAlg);
        %check if the distance is less than the minimum distance
        if node_conf.h < min_distance
            %update the minimum distance
            min_distance = node_conf.h;
            %update the nearest node
            nearest_node = node_conf;
        end
    end
end
    
