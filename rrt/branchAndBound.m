function [trees] = branchAndBound(sp, trees, c_best)
    % BRANCHANDBOUND Implements branch-and-bound algorithm.
    %
    % After this function completes its execution, nodes that does not
    % add more to the optimality are removed from both the trees.
    % That is, V' is the number of vertices to be removed by this function
    % and that V' = {x E V | Cost(x) + HeuristicCost(x, x_goal) > $c_best}
    %
    % Inputs:
    % - trees (1x2) cell array of two trees where:
    % trees(1) is the initial configuration tree.
    % trees(2) is the goal configuration tree.
    % - bestCost (1x1) cost of the best solution.
    %
    % Outputs:
    % - trees, (1x2) updated trees.

    % For the first tree.
    removePos = false(size(trees{1}, 1));
    for i = 1:size(trees{1}, 1)
        node = trees{1}(i, :);
        if node{4} + calculateCost(sp, node{1}, sp.goal_conf) > c_best
            removePos(i) = true;
        end
    end
    trees{1} = trees{1}(removePos);
    
    % For the second tree.
    removePos = false(size(trees{2}, 1));
    for i = 1:size(trees{2}, 1)
        node = trees{2}(i, :);
        if node{4} + calculateCost(sp, node{1}, sp.start_conf) > c_best
            removePos(i) = true;
        end
    end
    trees{2} = trees{2}(removePos);
end

