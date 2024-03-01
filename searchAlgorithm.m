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
    fringe = PDQ('init');
    PDQ('add', fringe, {[root.f, root.g, root.h], root.path}); %Initiating the the priority deque which is a custom class implemented in c++ and used with a wrapper in matlab
    PDQ('setMaxSize', fringe, 10000); %Set the max size of the priority deque
    set = java.util.HashSet; %A hashset used from the Java library to prevent cycles
    while ~PDQ('empty', fringe)
        [priority, path] = PDQ('poll', fringe);
        fringeNode.f = priority(1);
        fringeNode.g = priority(2);
        fringeNode.h = priority(3);
        fringeNode.path = path;
        set.add(mat2str(fringeNode.path(:,end-2:end))); %Getting the last configuration from the polled path
        exapndedNodes = exapndedNodes +1;
        if(fringeNode.h < 1)
            if size(sp.goals, 1) ~= 0
                sp.goals(1:sp.j, :) = [];
            end
            if size(sp.goals, 1) == 0 %Found the optimal path
                solution = fringeNode;
                return;
            else 
                PDQ('clear', fringe);
                set.clear;
                sp.goal_conf = sp.goals(1:sp.j, 1:3);
                fringeNode.h = getHeuristic(sp.typeOfHeuristic, sp.start_conf, sp);
            end
        end
        [greedyChildren] = greedyExpand(fringeNode, sp); %Generating children of the current node according to greedy algorithm
        validGreedyFound = false;
        for i = 1 :size(greedyChildren, 1)
            child = greedyChildren(i);
            [isColliding, ~] = collisionCheck(child.path(:,end-2:end), sp);%Checking if the greedy children are valid (not colliding), if not we will generate children according to the current algorithm [Astar, UCS, greedy]
            if ~set.contains(mat2str(child.path(:,end-2:end)))
                if isColliding == false
                    validGreedyFound = true;
                    PDQ('add', fringe, {[child.f child.g child.h], child.path});
                    set.add(mat2str(child.path(:,end-2:end)));
                else
                    set.add(mat2str(child.path(:,end-2:end)));
                end
            end
        end
        if validGreedyFound == false %If no valid greedy children were found, we generate children according to the current algorithm [Astar, UCS, Greedy]
            children = FullExpand(fringeNode, sp);
            for i = 1 :size(children, 1)
                child = children(i);
                [isColliding, ~] = collisionCheck(child.path(:,end-2:end), sp);
                if ~set.contains(mat2str(child.path(:,end-2:end)))
                    if isColliding == false
                        PDQ('add', fringe, {[child.f child.g child.h], child.path});
                        set.add(mat2str(child.path(:,end-2:end)));
                    else
                        set.add(mat2str(child.path(:,end-2:end)));
                    end
                end
            end
        end
        if PDQ('empty', fringe)
            disp('no path found');
            solution = [];
        end
    end
    PDQ('delete', fringe);
end


