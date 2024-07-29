function [fullPath, totalCost] = backtrackPath(sp, graphTree)
    totalCost = 0;
    curNode = graphTree(end, :);
    fullPath = {};
    paths = {};
    while curNode{2} > 0
        parentNode = graphTree(curNode{2}, :);
        [path, cost] = directExpansion(sp, realmax, parentNode{1}, curNode{1});

        if isempty(path)
            fullPath = {};
            totalCost = -1;
            return;
        end

        % path(1) = [];

        paths = [paths, {path}];
        totalCost = totalCost + cost;
        curNode = parentNode;
    end
    paths = flip(paths, 2);
    for i = 1:size(paths, 2)
        fullPath = [fullPath, paths{i}];
    end

    fullPath = [sp.start_conf, fullPath];
end