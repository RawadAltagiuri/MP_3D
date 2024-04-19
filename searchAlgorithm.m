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

function [solution, exapndedNodes] = searchAlgorithm(sp)
    exapndedNodes = 0;
    root.g = 0;
    root.h = getHeuristic(sp.typeOfHeuristic, sp.start_conf, sp);
    root.f = calculateCostBasedOnAlgorithm(root.g, root.h, sp.typeOfAlg);
    root.path = sp.start_conf;

    instanceId = PDQ_test("init", 10000);
    PDQ_test("insertAny", instanceId, root.path, [root.g, root.h, root.f]);
    set = java.util.HashSet; %A hashset used from the Java library to prevent cycles
    set.add(mat2str(root.path(:,end-2:end)));
    while ~(PDQ_test("size", instanceId) == 0)
        [path, priority] = PDQ_test("peek", instanceId);
        fringeNode.g = priority(1);
        fringeNode.h = priority(2);
        fringeNode.f = priority(3);
        fringeNode.path = path;

        % set.add(mat2str(fringeNode.path(:,end-2:end))); %Getting the last configuration from the polled path
        
        exapndedNodes = exapndedNodes + 1;
        if(fringeNode.h < 1)
            if size(sp.goals, 1) ~= 0
                sp.goals(1:sp.j, :) = [];
            end
            if size(sp.goals, 1) == 0 %Found the optimal path
                [path, costs] = PDQ_test("extractHead", instanceId);
                solution.path = [];
                for config = path
                    solution.path = [solution.path, config{1}];
          
                    solution.g = costs(1);
                    solution.h = costs(2);
                    solution.f = costs(3);
                end
                break;
            else 
                
                % PDQ('clear', fringe);
                % set.clear;
                % sp.goal_conf = sp.goals(1:sp.j, 1:3);
                % fringeNode.h = getHeuristic(sp.typeOfHeuristic, sp.start_conf, sp);
            end
        end

        nextChildren = {};

        [greedyChildren] = greedyExpand(fringeNode, sp); %Generating children of the current node according to greedy algorithm
        validGreedyFound = false;
        for i = 1 :size(greedyChildren, 1)
            child = greedyChildren(i);
            child.path = child.path(:, end-2:end);

            [isColliding, ~] = collisionCheck(child.path(:,end-2:end), sp);%Checking if the greedy children are valid (not colliding), if not we will generate children according to the current algorithm [Astar, UCS, greedy]
            if ~set.contains(mat2str(child.path(:,end-2:end)))
                if isColliding == false
                    nextChildren(size(nextChildren, 1) + 1, 1) = {{child.path, [child.g, child.h, child.f]}};

                    set.add(mat2str(child.path(:,end-2:end)));
                    
                    validGreedyFound = true;
                else
                    set.add(mat2str(child.path(:,end-2:end)));
                end
            end
        end
        if validGreedyFound == false %If no valid greedy children were found, we generate children according to the current algorithm [Astar, UCS, Greedy]
            children = FullExpand(fringeNode, sp);
            for i = 1 :size(children, 1)
                child = children(i);
                child.path = child.path(:, end-2:end); 

                [isColliding, ~] = collisionCheck(child.path(:,end-2:end), sp);
                if ~set.contains(mat2str(child.path(:,end-2:end)))
                    if isColliding == false
                        nextChildren(size(nextChildren, 1) + 1, 1) = {{child.path, [child.g, child.h, child.f]}};
                        set.add(mat2str(child.path(:,end-2:end)));
                    else
                        set.add(mat2str(child.path(:,end-2:end)));
                    end
                end
            end
        end

        PDQ_test("expandHead", instanceId, nextChildren);

        if PDQ_test("size", instanceId) == 0
            disp('no path found');
            solution = [];
        end
    end
    PDQ_test("destroyAll", instanceId);
end


