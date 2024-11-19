function [fullPath, totalCost] = backtrackPath(sp, graphTree)
    totalCost = 0;
    curNode = graphTree(end, :);
    fullPath = {};
    paths = {};
    while curNode{2} > 0
        parentNode = graphTree(curNode{2}, :);
        path = curNode{3};
        
        if isempty(path)
            fullPath = {};
            totalCost = -1;
            return;
        end
        
        if ~isempty(paths) && isequal(path{end}, paths{1})
            path(end) = [];
        end

        paths = [path, paths];
        curNode = parentNode;
    end

    fullPath = paths;
    totalCost = costOfPath(sp, fullPath);
end