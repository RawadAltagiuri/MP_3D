% this function checks which algorithm we are using exactly so it
% calculates the correct value of heuristic 
% INPUT:
% 'g' : the heuristic 
% 'h' : the direct path distance 
% 'typeOfAlg': a string value to represent the type of the algorithm
% OUTPUT:
% 'f': the final heuristic value depending on the algorithm
function [f] = calculateCostBasedOnAlgorithm(g, h, typeOfAlg)
    switch typeOfAlg
        case 'astar' % if the algorithm is A star, then the final heuristic will be the sum between g and h
            f = g + h;
        case 'ucs' % if the algorithm is ucs, then the final heuristic value will be the g, the heuristic value between the move and the next one
            f = g;
        case 'greedy' % if the algorithm is greedy, then the final heuristic will be h, which is the direct way to the goal
            f = h;
    end
end