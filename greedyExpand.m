%Produces 'greedy' children of the given node (configuration), the
%children are one for eversion and one for steering
%
%Input:
% 'node': The configuration matrix of the parent
% 'sp': The search problem as described in 'MotionPlannerSolution.m'

function [children] = greedyExpand(node, sp)
    children = [];

    [greedyChild, isValid] = steeringChild(node, sp); %Produce the steering child
    if isValid == true %if the child is not valid then we do not account for it
        children = [children ; greedyChild];
    end

    if ~isValid %if we have a steering child we do not generate an eversion child, this way we prioritize steering first
        [greedyChild, isValid] = eversionChild(node, sp); %Produce the eversion child
        if isValid == true %if the child is not valid then we do not account for it
            children = [children ; greedyChild];            
        end
    end
end

%Produces the eversion child
%Input:
% 'node': The configuration matrix of the parent
% 'sp': The search problem as described in 'MotionPlannerSolution.m'
% Output:
% 'greedyChild': The generated child node
% 'isValid': Flag indicating whether the child is a valid node

function [greedyChild, isValid] = eversionChild(node, sp)
    %Extract the configuration of the parent node
    parent_conf = node.path(:, end-2:end);
    child_conf = parent_conf;
    %Since a link cannot expand unless the previous link is full expanded,
    %we can calculate the eversion of the next link by sum of current links
    %+ amount of eversion which is the difference between the goal and the
    %parent

    amountOfEversion = sum(sp.goal_conf(:, 3)) - sum(parent_conf(:, 3));
    if amountOfEversion > 0
        amountOfEversion = min(amountOfEversion, sp.stepSize(2));
    else
        amountOfEversion = max(amountOfEversion, sp.stepSize(2));
    end

    totaleversion = sum(parent_conf(:, 3)) + amountOfEversion;
    
    %distributing the eversion among links
    for r = 1 : sp.j
        child_conf(r, 3) = totaleversion;
        if child_conf(r, 3) < sp.goal_conf(r, 3) && amountOfEversion < 0
            child_conf(r, 3) = sp.goal_conf(r, 3);
        elseif child_conf(r, 3) > sp.goal_conf(r, 3) && amountOfEversion > 0
            child_conf(r, 3) = sp.goal_conf(r, 3);
        end
        if child_conf(r, 3) > sp.design(r)
            child_conf(r, 3) = sp.design(r);
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
    % greedyChild.g = node.g + calculateCost(searchProblem, node.path(:,end-2:end), child_conf);
    greedyChild.g = node.g + calculateCost_old(node.path(:,end-2:end), child_conf, sp.home_base);
    greedyChild.h = getHeuristic(sp.typeOfHeuristic, child_conf, sp);
    greedyChild.f = calculateCostBasedOnAlgorithm(greedyChild.g, greedyChild.h, sp.typeOfAlg);
    greedyChild.path = [node.path , child_conf];
end

%Produces the eversion child
%Input:
% 'node': The configuration matrix of the parent
% 'sp': The search problem as described in 'MotionPlannerSolution.m'
%Output:
% 'greedyChild': The generated child node
% 'isValid': Flag indicating whether the child is a valid node
function [greedyChild, isValid] = steeringChild(node, sp)
    % Extract the configuration of the parent node
    parent_conf = node.path(:, end-2:end);
    child_conf = parent_conf;

     %update the x and y coordinates of the child based on steering towards the goal
    child_conf(:, 1:2) = parent_conf(:, 1:2) + (sign(sp.goal_conf(:, 1:2) - parent_conf(:, 1:2)) * sp.stepSize(1));
    
%   Make sure we don't overshoot to our taget due to the step size
    for ConfCount = 1:sp.j
        if sp.goal_conf(ConfCount, 1) > parent_conf(ConfCount, 1) && sp.goal_conf(ConfCount, 1) < child_conf(ConfCount, 1)
            child_conf(ConfCount,1) = sp.goal_conf(ConfCount, 1);
        elseif sp.goal_conf(ConfCount, 1) < parent_conf(ConfCount, 1) && sp.goal_conf(ConfCount, 1) > child_conf(ConfCount, 1)
            child_conf(ConfCount,1) = sp.goal_conf(ConfCount, 1);
        end
        if sp.goal_conf(ConfCount, 2) > parent_conf(ConfCount, 2) && sp.goal_conf(ConfCount, 2) < child_conf(ConfCount, 2)
            child_conf(ConfCount,2) = sp.goal_conf(ConfCount, 2);
        elseif sp.goal_conf(ConfCount, 2) < parent_conf(ConfCount, 2) && sp.goal_conf(ConfCount, 2) > child_conf(ConfCount, 2)
            child_conf(ConfCount,2) = sp.goal_conf(ConfCount, 2);
        end
    end

    %Check if the resulting child violates the minimum length constraint
    % MODIFIED:
    % If there is a min length constraint violation, then grow.
    for r = 1 : sp.j
        if child_conf(r, 3) < sp.lengthMin
            child_conf(r:end, 1:2) = parent_conf(r:end, 1:2);
            break;
        end
    end

    %check if the child configuration is the same as the parent's configuration
    if isequal(child_conf, parent_conf)
        if abs(sum(parent_conf(:, 3)) - sum(sp.goal_conf(:, 3))) > 0.0001
            greedyChild = [];
            isValid = false;
        else
            growConf = parent_conf;
            growConf(r, 3) = sp.lengthMin;
            modSp = sp;
            modSp.goal_conf = growConf;
            [greedyChild, isValid] = eversionChild(node, modSp);
            greedyChild.h = getHeuristic(sp.typeOfHeuristic, parent_conf, sp);
            greedyChild.f = calculateCostBasedOnAlgorithm(greedyChild.g, greedyChild.h, sp.typeOfAlg);
        end
        
        return;
    end

    isValid = true;
    greedyChild.label = 'steering';
    % greedyChild.g = node.g + calculateCost(searchProblem, node.path(:,end-2:end), child_conf);
    greedyChild.g = node.g + calculateCost_old(node.path(:,end-2:end), child_conf, sp.home_base);
    greedyChild.h = getHeuristic(sp.typeOfHeuristic, child_conf, sp);
    greedyChild.f = calculateCostBasedOnAlgorithm(greedyChild.g, greedyChild.h, sp.typeOfAlg);
    greedyChild.path = [node.path , child_conf];
end