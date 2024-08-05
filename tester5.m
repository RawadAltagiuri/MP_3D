%Authors:
% •Fabio Stroppa (Algorithms, Concept)
% •Rawad E. H. Altagiuri (Algorithms)
% •Omar H. A. Zaghloul (Algorithms)
% •Ömer Kalafatlar

%Start of the motion planner
%
% 'SP' is the search problem and its attributes are:
%
% .problemName: Presets we've created which are [wall, wallWithEntrance,
% hole, GrabbingTest] Switch the problem name to see different presets
% .typeOfAlg: switch the algorithm between [astar, ucs, greedy]
% .typeOfHeuristic: switch the heuristic between [continue, discrete]
% .steerBounds: set the steering limits of the joints in [xdimension,
% ydimension]
% .lengthMin: minimum required link length for its joint to be able to
% steer
% .plane_z: the plane's y dimension
% .costArray: cost of [steer, growth, retract]
% .stepSize: the size of each step of [steering, eversion]
% .obstacles: Each row is an obstacle and the columns are [x, y, zplane base,
% radius, height]
% .goals: each element is an N*3 matrix, each row corresponding to a joint and link 
% .design: an N rows matrix, containing the max length of each link for
% the general design of the robot
% .baseRotate: True if the home base of the robot can rotate
% .start_conf: A N*3 matrix which is the starting position of the robot
% .home_base: Coordinates of the home base of the robot [x, y, z]
%
%  Start is the starting node in the algorithm, you set the configuration
%  of the robot in the start, its attributes
% .design: an N rows matrix, containing the max length of each link for
%  the general design of the robot
% .matrix: the start configuration of the robot, the columns are [x, y, z]
%
%
% Running this file correctly will result in:
%
% 'formattedPathForAnimation': A 3D matrix containing every configuration
% for the robot to transform from the start configuration to the goal
% configuration
% 
% 'solution': Contains the path matrix, and the total cost of the path
% [path, h, g, f]
% 


clear, clc, close;

load envs
sp = envs{2};
sp.baseRotate = false;
sp.heuristicLimit = 0.1;

% sp.start_conf = [0 0 50; 0 0 150; 0 30 50; 0 0 0; 0 0 0];
% sp.goal_conf = [0 0 50; 0 0 150; 0 0 0; 0 0 0; 0 0 0];

rrtConf.pOfGoal = 0.1;
rrtConf.numOfNodes = 250;
rrtConf.stepSize = 1;
rrtConf.neighbourSize = calculateNeighbourSize(sp.start_conf, sp.goal_conf, sp);
tic
[path, cost, tree, final_child] = searchAlgorithmRRT_star(sp, rrtConf, false);
% [path, cost] = directExpansion(sp, 10000, sp.start_conf, sp.goal_conf);
% [solution, exapndedNodes] = searchAlgorithm_Sto(sp, 100000);
time = toc;
solution.path = pathConversion1(path);
solution.g = cost;
solution.f = solution.g;
solution.h = 0;


rrtConf.pOfGoal = 0.1;
rrtConf.numOfNodes = 100;
rrtConf.stepSize = 5;

% testWriterRRTs(rrtConf, envs{2}, 1, "results.xls");

% Calculate the number of submatrices you will create
numSubMatrices = size(solution.path, 2) / 3;

% Preallocate the 3D array to store the submatrices
formattedPathForAnimation = zeros(sp.j, 3, numSubMatrices);

% Extract the submatrices and store them in the 3D array
for i = 1:numSubMatrices
    formattedPathForAnimation(:, :, i) = solution.path(:, (i-1)*3 + 1 : i*3);
end

growthCount = 0;
retractCount = 0;
steerCount = 0;
for i=2:size(formattedPathForAnimation,3)
    [growthCount, retractCount, steerCount] = actionCounter(formattedPathForAnimation(:, :, i), formattedPathForAnimation(:, :, i-1), growthCount, retractCount, steerCount);
end

 softRobot_animation(formattedPathForAnimation, [0,0,0,0,0], true, sp);