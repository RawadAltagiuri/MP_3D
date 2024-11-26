function [trees, bestPath] = randomVertexContraction(sp, trees, bestPath)
    % randomVertexContraction
    % Implements a random vertex contraction where in a given path,
    % two random vertices are selected and attempted to be connected.
    % If such a connection is feasible (collusion free), then a new path
    % is formed with such a connection.
    %
    % Inputs:
    % - trees (1x2) trees.
    % - bestPath (1x1 struct) a struct with fields:
    % + nodes, nodes of respective trees.
    % + connection, the path that has only two endpoints that are member of any
    % tree.
    % + cost, cost of the best path.

    if rand < 0.5
        t = 1;
    else
        t = 2;
    end

    range = size(trees{t}, 1);
    firstNode = randi(range);
    secondNode = randi(range);

    if firstNode == secondNode
        return;
    elseif secondNode < firstNode
        temp = firstNode;
        firstNode = secondNode;
        secondNode = temp;
    end

    [path, cost] = directExpansion(sp, realmax, trees{t}{firstNode, 1}, trees{t}{secondNode, 1});
    if ~isempty(path)
        trees{t}(secondNode, 2:4) = {firstNode, path, cost + trees{t}{firstNode, 4}};
        bestPath.cost = cost;
    end
end

