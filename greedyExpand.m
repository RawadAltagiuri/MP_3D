%Produces 'greedy' children of the given node (configuration), the
%children are one for eversion and one for steering
%
%Input:
% 'node': The configuration matrix of the parent
% 'sp': The search problem as described in 'MotionPlannerSolution.m'

function [children] = greedyExpand(node, searchProblem)
    children = [];

    [greedyChild, isValid] = steeringChild(node, searchProblem); % Produce the steering child
    % check if the steering child is colliding with the obstacles
    if isValid % only check for collision if the child was initially considered valid
        [isColliding, ~] = collisionCheck(greedyChild.path(:,end-2:end), searchProblem);
        if ~isColliding
            children = [children ; greedyChild];
        end
    end

    [greedyChild, isValid] = eversionChild(node, searchProblem); % Produce the eversion child
    % check if the eversion child is colliding with the obstacles
    if isValid % only check for collision if the child was initially considered valid
        [isColliding, ~] = collisionCheck(greedyChild.path(:,end-2:end), searchProblem);
        if ~isColliding
            children = [children ; greedyChild];
        end
    end

    %sort the children based on the heuristic value
    if size(children, 1) > 1 && children(2).h < children(1).h
        temp = children(1);
        children(1) = children(2);
        children(2) = temp;
    end
end

%Produces the eversion child
%Input:
% 'node': The configuration matrix of the parent
% 'sp': The search problem as described in 'MotionPlannerSolution.m'
% Output:
% 'greedyChild': The generated child node
% 'isValid': Flag indicating whether the child is a valid node

function [greedyChild, isValid] = eversionChild(node, searchProblem)
    %Extract the configuration of the parent node
    parent_conf = node.path(:, end-2:end);
    child_conf = parent_conf;
    %Since a link cannot expand unless the previous link is full expanded,
    %we can calculate the eversion of the next link by sum of current links
    %+ amount of eversion which is the difference between the goal and the
    %parent
    amountOfEversion = (sign(sum(searchProblem.random_conf(:, 3)) - sum(parent_conf(:, 3))) * searchProblem.stepSize(2));
    totaleversion = sum(parent_conf(:, 3)) + amountOfEversion;
    
    %distributing the eversion among links
    for r = 1 : searchProblem.j
        child_conf(r, 3) = totaleversion;
        if child_conf(r, 3) < searchProblem.random_conf(r, 3) && amountOfEversion < 0
            child_conf(r, 3) = searchProblem.random_conf(r, 3);
        elseif child_conf(r, 3) > searchProblem.random_conf(r, 3) && amountOfEversion > 0
            child_conf(r, 3) = searchProblem.random_conf(r, 3);
        end
        if child_conf(r, 3) > searchProblem.design(r)
            child_conf(r, 3) = searchProblem.design(r);
        elseif child_conf(r, 3) < 0
            child_conf(r, 3) = 0;
            break;
        end
        totaleversion = totaleversion - child_conf(r, 3);
    end
    
    %check if the child configuration is the same as the parent's configuration
    if isequal(child_conf, parent_conf)
        greedyChild = [];
        isValid = false;
        return;
    end
    isValid = true;
    greedyChild.label = 'eversion';
    greedyChild.g = node.g + calculateCost(node.path(:,end-2:end), child_conf, searchProblem.home_base);
    greedyChild.h = getHeuristic(searchProblem.typeOfHeuristic, child_conf, searchProblem);
    greedyChild.f = calculateCostBasedOnAlgorithm(greedyChild.g, greedyChild.h, searchProblem.typeOfAlg);
    greedyChild.path = [node.path , child_conf];
end

%Produces the eversion child
%Input:
% 'node': The configuration matrix of the parent
% 'sp': The search problem as described in 'MotionPlannerSolution.m'
%Output:
% 'greedyChild': The generated child node
% 'isValid': Flag indicating whether the child is a valid node
function [greedyChild, isValid] = steeringChild(node, searchProblem)
    % Extract the configuration of the parent node
    parent_conf = node.path(:, end-2:end);
    child_conf = parent_conf;

     %update the x and y coordinates of the child based on steering towards the goal
    child_conf(:, 1:2) = parent_conf(:, 1:2) + (sign(searchProblem.random_conf(:, 1:2) - parent_conf(:, 1:2)) * searchProblem.stepSize(1));
    
%   Make sure we don't overshoot to our taget due to the step size
    for ConfCount = 1:searchProblem.j
        if searchProblem.random_conf(ConfCount, 1) > parent_conf(ConfCount, 1) && searchProblem.random_conf(ConfCount, 1) < child_conf(ConfCount, 1)
            child_conf(ConfCount,1) = searchProblem.random_conf(ConfCount, 1);
        elseif searchProblem.random_conf(ConfCount, 1) < parent_conf(ConfCount, 1) && searchProblem.random_conf(ConfCount, 1) > child_conf(ConfCount, 1)
            child_conf(ConfCount,1) = searchProblem.random_conf(ConfCount, 1);
        end
        if searchProblem.random_conf(ConfCount, 2) > parent_conf(ConfCount, 2) && searchProblem.random_conf(ConfCount, 2) < child_conf(ConfCount, 2)
            child_conf(ConfCount,2) = searchProblem.random_conf(ConfCount, 2);
        elseif searchProblem.random_conf(ConfCount, 2) < parent_conf(ConfCount, 2) && searchProblem.random_conf(ConfCount, 2) > child_conf(ConfCount, 2)
            child_conf(ConfCount,2) = searchProblem.random_conf(ConfCount, 2);
        end
    end
    %Check if the resulting child violates the minimum length constraint
    for r = 1 : searchProblem.j
        if child_conf(r, 3) < searchProblem.lengthMin
            child_conf(r:end, 1:2) = parent_conf(r:end, 1:2);
            break;
        end
    end

    %check if the child configuration is the same as the parent's configuration
    if isequal(child_conf, parent_conf)
        greedyChild = [];
        isValid = false;
        return;
    end
    isValid = true;
    greedyChild.label = 'steering';
    greedyChild.g = node.g + calculateCost(node.path(:,end-2:end), child_conf, searchProblem.home_base);
    greedyChild.h = getHeuristic(searchProblem.typeOfHeuristic, child_conf, searchProblem);
    greedyChild.f = calculateCostBasedOnAlgorithm(greedyChild.g, greedyChild.h, searchProblem.typeOfAlg);
    greedyChild.path = [node.path , child_conf];
end