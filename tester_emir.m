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
sp.times = [1, 0.3, 0.3];
sp.metric = "time";
paddingAmount = 5;
sp.obstacles(:, 4:5) = sp.obstacles(:, 4:5) + paddingAmount;

sp.baseRotate = false;
sp.heuristicLimit = 0.1;
sp.goalRegion = 50;

rrtConf.numOfNodes = 1000;
rrtConf.stepSize = 5;
rrtConf.neighbourSize = calculateNeighbourSize(sp) * 10;


[path, cost, tree, final_child] = searchAlgorithmRRT(sp, rrtConf, false);

minCost = realmax;
for i = 1:size(tree, 1)
    cost = calculateCost(sp, tree{i, 1}, sp.goal_conf);
    if cost < minCost
        minCost = cost;
    end
end

solution.g = cost;
solution.f = solution.g;
solution.h = 0;

modSp = sp;
modSp.obstacles = [];
animate(sp, path);







