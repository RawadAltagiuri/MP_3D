function [path, cost] = directExpansion(sp, step_size, start_conf, end_conf)    
    config = start_conf;
    path = {config};
    cost = -1;
    while getHeuristic(sp, config, end_conf) > sp.heuristicLimit && (size(path, 2) - 1) < step_size
        config = greedyExpand(sp, config, end_conf);

        if isempty(config)
            break;
        end

        if collisionCheck(config, sp) 
            % Controls if obstacles in front of the random node is considered or not (Fix this).
            path = {};
            cost = -1;
            return;
        end

        path = [path, config];
    end
    
    cost = costOfPath(sp, path);
end
