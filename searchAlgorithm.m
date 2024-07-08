%Search algorithm which finds the optimal path between the start
%configuration and the goal configuration
%
%Input:
%'sp': As described in the file 'MotionPlannerSolution.m'
%
%
%Output:
%
%'solution': The solution for the search algorithm which contains the
%optimal path for the search algorithm, the heuristic, the cost, and f which is the priority for Astar [path, h, g, f]
%
%'expandedNodes': the number of expanded nodes to find the optimal solution
%

%the function uses RRT to find a path
function [solution, exapndedNodes] = searchAlgorithm(sp)
    finalChild = [];
    root.path = sp.start_conf;
    root.g = 0;
    root.h = getHeuristic(sp.typeOfHeuristic, root.path(:, end-2:end), sp);
    root.f = calculateCostBasedOnAlgorithm(root.g, root.h, sp.typeOfAlg);
    %create a map to store the nodes, the key is the configuration matrix and the value is the parent node
    nodes_map = containers.Map({mat2str(root.path(:, end-2:end))}, {root});

    for i = 1:sp.iterations
        %generate a random configuriation
        random_conf = RrtExpand(sp);
        %find the nearest node to the random configuration
        nearest_node = findNearestNode(nodes_map, random_conf, sp);
        sp.random_conf = random_conf;
        greedyChildren = greedyExpand(nearest_node, sp);
        if isempty(greedyChildren)
            continue; % Skip the rest of the current iteration
        else
            child = greedyChildren(1);
        end
        if ~nodes_map.isKey(mat2str(child.path(:, end-2:end))) %check if the child is already in the tree
            [isColliding, ~] = collisionCheck(child.path(:,end-2:end), sp);%Checking if the greedy children are valid (not colliding)
            if isColliding == false
                nodes_map(mat2str(child.path(:, end-2:end))) = nearest_node;

                %check if the the difference between all cells in the child configuration and the goal configuration is less than 100
                if calculateCost(child.path(:, end-2:end), sp.goal_conf, sp.home_base) < sp.AcceptedEuclideanDistance
                    finalChild = child;
                    break;
                end
            else
                continue;
            end
        end
    end

    if i == 100000 && isempty(finalChild)
        solution = [];
        exapndedNodes = i;
        return;
    end
        

    exapndedNodes = nodes_map.Count;
    solution.map = nodes_map;

    %create the path beginning from the final child to the root, then reverse it, we retrieve the parent of the last (:, end-2:end) from the map and then add it to the right of the path
    path = finalChild.path(:, end-2:end);
    while ~isequal(path(:, end-2:end), sp.start_conf)
        parent = nodes_map(mat2str(path(:, end-2:end)));
        path = [path, parent.path(:, end-2:end)];
    end
    solution.path = path;
    solution.g = 0;
    %calculate the cost of the path using the cost function, every three columns represent a configuration matrix
    for i = 1:size(path, 2)/3
        solution.g = solution.g + calculateCost(path(:, (i-1)*3 + 1 : i*3), sp.goal_conf, sp.home_base);
    end
    solution.h = getHeuristic(sp.typeOfHeuristic, path(:, 1:3), sp);
    solution.f = calculateCostBasedOnAlgorithm(solution.g, solution.h, sp.typeOfAlg);
    solution.final_child = child;
    


end


