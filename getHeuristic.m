% This function gets the specific calculation method for the heuristic
% according to the given type of heuristic
% INPUT:
% 'typeOfHeuristic': string value that represents if the heuristic is discrete or continue  
% 'currentMat': n x 3 matrix that represents the current configurations of the robot 
% 'searchProblem': a structure of the motion details that is going to get
%                  passed to the calculate heuristic function
function [h] = getHeuristic(sp, config, goal)
        h = calculateCost(sp, config, goal);
end