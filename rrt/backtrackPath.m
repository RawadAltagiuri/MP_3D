%{
Forms a path from the root node to node in the tree with a
given index. The path is formed by backtracking from the indexed node
to the root in the tree.

@omerk
%}
function [fullPath, totalCost] = backtrackPath(sp, graphTree, index)
    totalCost = 0;
    curNode = graphTree(index, :);
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