%Produces children according to the current algorithm in sp
%the children are [Eversion-, Eversion+, and then a combination of steering
%of either steering a join in either x or y or both for every join capable
%of steering]
%
%Input:
% 'node': the parent node in the search tree
% 'sp': search problem as described in 'MotionPlannerSolution.m' file
%
%Output:
% 'children': the children of the given node and their information
% [heuristic, cost, total] according to the current algorithm in sp
%

function [children] = FullExpand(node, sp)
    children = [];
    %producde the eversion+ and eversion- children
    [child, isValid] = doEversion(node, 0, sp);
    if isValid == true %if the child is valid (within constrains and not identical to the parent) then we add it to the children
        children = [children ; child];
    end
    [child, isValid] = doEversion(node, 1, sp);
    if isValid == true %if the child is valid (within constrains and not identical to the parent) then we add it to the children
        children = [children ; child];
    end
    
    %if the joint at the base can rotate we start the rotation from the
    %first index, otherwise we start from the second
    if sp.baseRotate == true
        row = 1;
    else
        row = 2;
    end
    %Constrain to allow only the joints higher than the obstacles to
    %rotate
    lastIndexToExpand = findJointHigherThanAllObstacles(sp);
    
    %Generate the steering children of steering only x or y
    for r = row: lastIndexToExpand
        for c = 1:2
            [child, isValid] = doSteering(node, 0, r,c, sp);
            if isValid == true %if the child is valid (within constrains and not identical to the parent) then we add it to the children
                children = [children ; child];
            end
            [child, isValid] = doSteering(node, 1, r,c, sp);
            if isValid == true %if the child is valid (within constrains and not identical to the parent) then we add it to the children
                children = [children ; child];
            end
        end
    end

    %Generate the combined steering children, that steer both x and y
    for r = row:lastIndexToExpand
           [child, isValid] = combineSteering(node, 0, 0, r, sp);
           if isValid == true
               children = [children ; child];
           end
           [child, isValid] = combineSteering(node, 1, 1, r, sp);
           if isValid == true %if the child is valid (within constrains and not identical to the parent) then we add it to the children
               children = [children ; child];
           end
           [child, isValid] = combineSteering(node, 1, 0, r, sp);
           if isValid == true %if the child is valid (within constrains and not identical to the parent) then we add it to the children
               children = [children ; child];
           end
           [child, isValid] = combineSteering(node, 0, 1, r, sp);
           if isValid == true %if the child is valid (within constrains and not identical to the parent) then we add it to the children
               children = [children ; child];
           end
    end
end

% Function to perform eversion operation (grow or retract) on a node
% Input:
%  'node': the parent node in the search tree
%  'operation': 0 for grow, 1 for retract
%  'sp': search problem
% Output:
%  'child': the generated child node
%  'isValid': flag indicating whether the child is a valid node
function [child, isValid] = doEversion(node, operation, sp)
    conf = node.path(:,end-2:end);

    % Check if the operation is valid based on the current configuration
    if operation == 0 % grow
        if sum(conf(:,3)) == sum(sp.design)
            child=[];
            isValid = false;
            return;
        end
    else % retract
        if sum(conf(:,3)) == 0
            child=[];
            isValid = false;
            return;
        end
    end

    row = 1;
    %Find the row where the eversion operation should be performed
    while row <= size(conf, 1) && conf(row, 3) == sp.design(row)
        row = row+1;
    end
    if row > size(conf,1)
        row = size(conf,1);
    end
    %Perform the eversion operation based on the specified operation type
    if operation == 0
        conf(row, 3) = conf(row, 3) + sp.stepSize(1, 2);
        if conf(row, 3) > sp.design(row)
            if(row < size(conf,1))
                conf(row+1, 3) = conf(row, 3) - sp.design(row);
                conf(row, 3) = sp.design(row);
            else
                child = [];
                isValid = false;
                return;
            end
        end
    elseif operation == 1
        conf(row, 3) = conf(row, 3) - sp.stepSize(1, 2);
        if conf(row, 3) < 0
            if row > 1
                conf(row-1, 3) = conf(row-1, 3) + conf(row, 3);
                conf(row, 3) = 0;
            else
                child = [];
                isValid = false;
                return;
            end
        end
    end
    % ---- calculate cost -----
    child.g = node.g + calculateCost(sp, node.path(:,end-2:end), conf);
    if(operation==0)
        child.label = 'e+';
    else
        child.label = 'e-';
    end
    isValid = true;
    child.h = getHeuristic(sp.typeOfHeuristic, conf, sp);
    child.f = calculateCostBasedOnAlgorithm(child.g, child.h, sp.typeOfAlg);
    child.path = [node.path , conf]; % to get all configurations of a path, divide the path by 3 columns
end



% Function to perform steering operation on a node
% Input:
% 'node': the parent node in the search tree
% 'operation': 0 for increment, 1 for decrement
% 'row': row index of the configuration joint to be modified
% 'col': column index of the configuration joint to be modified
%  'sp': Search problem
% Output:
%  'child': the generated child node
%  'isValid': flag indicating whether the child is a valid node
function [child, isValid] = doSteering(node, operation, row, col, sp)
    % ---- generate new configuration ----
  
    conf = node.path(:,end-2:end);
    if conf(row, col) == sp.steerBounds(1) && operation ==1  || conf(row, col) == sp.steerBounds(2) && operation ==0
        child=[];
        isValid = false;
        return;
    end

    if(conf(row, 3)<sp.lengthMin)
        child.h = 0;
        isValid = false;
        return;
    end
    if(operation == 0)
        conf(row, col) = conf(row, col) + sp.stepSize(1, 1);
        if conf(row, col) > sp.steerBounds(2)
            conf(row, col) = sp.steerBounds(2);
        end
    else
        conf(row, col) = conf(row, col) - sp.stepSize(1, 1);
        if conf(row, col) < sp.steerBounds(1)
            conf(row, col) = sp.steerBounds(1);
        end
    end
    if col == 1
        if operation == 0
            child.label = "a" + row + "+";
        else
            child.label = "a" + row + "-";
        end
    else
        if operation == 0
            child.label = "b" + row + "+";
        else
            child.label = "b" + row + "-";
        end
    end
    isValid = true;
    child.g = node.g + calculateCost(sp, node.path(:,end-2:end), conf);
%     child.h = calculateHeuristic(conf, sp);
    child.h = getHeuristic(sp.typeOfHeuristic, conf, sp);
    child.f = calculateCostBasedOnAlgorithm(child.g, child.h, sp.typeOfAlg);
    child.path = [node.path , conf];
end


% Function to combine steering operations on two columns of a node's configuration
% Input:
%   'node': the parent node in the search tree
%   'operationFirstCol': 0 for increment, 1 for decrement on the first column
%   'operationSecondCol': 0 for increment, 1 for decrement on the second column
%   'row': row index of the configuration joint to be modified
%   'sp': search problem
% Output:
%   'child': the generated child node
%   'isValid': flag indicating whether the child is a valid node
function [child, isValid] = combineSteering(node, operationFirstCol, operationSecondCol, row, sp)
    conf = node.path(:,end-2:end);


    %if any of the columns is at its limit then this child is invalid
    if conf(row, 1) == sp.steerBounds(1) && operationFirstCol ==1  || conf(row, 1) == sp.steerBounds(2) && operationFirstCol ==0
        child=[];
        isValid = false;
        return;
    end
    if conf(row, 2) == sp.steerBounds(1) && operationSecondCol ==1  || conf(row, 2) == sp.steerBounds(2) && operationSecondCol ==0
        child=[];
        isValid = false;
        return;
    end

    %if the link is smaller than the minimum then we can't steer
    if(conf(row, 3)<sp.lengthMin)
        child.h = 0;
        isValid = false;
        return;
    end

    %Perform steering on the first column based on the specified operation type
    if(operationFirstCol == 0) %Increment
        conf(row, 1) = conf(row, 1) + sp.stepSize(1, 1);
        if conf(row, 1) > sp.steerBounds(2)
            conf(row, 1) = sp.steerBounds(2);
        end
    else %Decrement
        conf(row, 1) = conf(row, 1) - sp.stepSize(1, 1);
        if conf(row, 1) < sp.steerBounds(1)
            conf(row, 1) = sp.steerBounds(1);
        end
    end

    %Perform steering on the second column based on the specified operation type
    if(operationSecondCol == 0) %Increment
        conf(row, 2) = conf(row, 2) + sp.stepSize(1, 1);
        if conf(row, 2) > sp.steerBounds(2)
            conf(row, 2) = sp.steerBounds(2);
        end
    else %Decrement
        conf(row, 2) = conf(row, 2) - sp.stepSize(1, 1);
        if conf(row, 2) < sp.steerBounds(1)
            conf(row, 2) = sp.steerBounds(1);
        end
    end
    
    opArray = ['+' '-'];
    %Calculate the cost and fill the child information
    child.label = "a" + row + opArray(1, operationFirstCol+1) + ", b" + row + opArray(1, operationSecondCol+1);
    isValid = true;
    child.g = node.g + calculateCost(sp, node.path(:,end-2:end), conf);
    child.h = getHeuristic(sp.typeOfHeuristic, conf, sp);
    child.f = calculateCostBasedOnAlgorithm(child.g, child.h, sp.typeOfAlg);
    child.path = [node.path , conf];

end


