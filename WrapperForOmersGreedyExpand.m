function [child] = WrapperForOmersGreedyExpand(fringeNode, sp)
    newStep = greedyExpand(sp, fringeNode.path(:,end-2:end), sp.goal_conf);
    child.path = [fringeNode.path newStep];
    child.g = fringeNode.g + calculateCost(fringeNode.path(:,end-2:end), child.path(:, end-2:end), sp.home_base);
    child.h = getHeuristic(sp.typeOfHeuristic, child.path(:,end-2:end), sp);
    child.f = calculateCostBasedOnAlgorithm(child.g, child.h, sp.typeOfAlg);
end