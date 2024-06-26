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

sp.problemName = "wall";
sp.typeOfAlg = 'astar';
sp.typeOfHeuristic = 'continue';

switch sp.problemName
    case "wall"
        start.design = [50; 150; 150; 150; 150];
        start.matrix = [0 0 50;
                    0  0 150;
                    0 0 150;
                    0 0 150;
                    0 0 150];
        sp.steerBounds = [-30 30];
        sp.lengthMin = 5;
        sp.plane_z = 1000;
        sp.costArray = [1, 1, 1];
        sp.stepSize = [5,40];
        sp.obstacles = [
        -50	-300 sp.plane_z	25	650
        -50	-250 sp.plane_z	25	650
        -50	-200 sp.plane_z	25	650
        -50	-150 sp.plane_z	25	650
        -50	-100 sp.plane_z	25	650
        -50	-50	sp.plane_z	25	650
        -50	0	sp.plane_z	25	650
        -50	50	sp.plane_z	25	650
        -50	100	sp.plane_z	25	650
        -50	150	sp.plane_z	25	650
        -50	200	sp.plane_z	25	650
        -50	250	sp.plane_z	25	650
        -50	300	sp.plane_z	25	650
        ];
        sp.goals =[
            [0 0 50; 0 -25 150; 0 -20 150; 0 20 150; 0 0 150]
            ];


    case "wallWithEntrance"
        start.design = [50; 150; 150; 150; 150];
        start.matrix = [0 0 50; 10 0 150; 10 0 150; 0 0 150; 0 0 150];
        sp.steerBounds = [-30 30];
        sp.lengthMin = 5;
        sp.plane_z = 1000;
        sp.costArray = [1, 1, 1];
        sp.stepSize = [2.5, 30];
        sp.obstacles = [
        -50	-300 sp.plane_z	25	650
        -50	-250 sp.plane_z	25	650
        -50	-200 sp.plane_z	25	650
        -50	-150 sp.plane_z	25	650
        -50	0	sp.plane_z	25	650
        -50	50	sp.plane_z	25	650
        -50	100	sp.plane_z	25	650
        -50	150	sp.plane_z	25	650
        -50	200	sp.plane_z	25	650
        -50	250	sp.plane_z	25	650
        -50	300	sp.plane_z	25	650
        ];
        sp.goals =[
            [0 0 50; 0 -25 150; 0 -20 150; 0 20 150; 0 0 150]
            ];
    case "hole"
        start.design = [50; 150; 175; 150; 200];
        start.matrix = [0 0 50; 0 0 150; 0 0 175; 0 0 150; 0 0 150];
        sp.steerBounds = [-40 30];
        sp.lengthMin = 5;
        sp.plane_z = 1000;
        sp.costArray = [1, 1, 1];
        sp.stepSize = [2.5, 30];
        sp.obstacles = [
           0  100   450    25   100
         -50  100   450    25   100
          50  100   450    25   100
          50 100   sp.plane_z    25   430
         -50 100   sp.plane_z    25   430
          0  100   sp.plane_z    25   430
        -125 100   sp.plane_z    50   650
         125 100   sp.plane_z    50   650
        ]; 
        sp.goals =[
            [0 0 50; 0 0 150; 0 0 175; -30 0 150; -40 0 200]
            ];
    case 'GrabbingTest'
        start.design =[50; 100; 150; 150; 150];
        start.matrix = [0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0];
        sp.steerBounds = [-40 40];
        sp.lengthMin = 5;
        sp.plane_z = 1000;
        sp.costArray = [1, 1, 1];
        sp.stepSize = [2.5, 30];
        sp.obstacles = [
           0  -200   sp.plane_z    25   450
           0  200   sp.plane_z    25   450
        ]; 
        sp.goals =[
            [0 0 50; 0 0 100; 5 0 150; 20 0 150; 35 0 140]
            [0 0 50; 0 0 100; -5 0 150; -20 0 150; -35 0 140]
        ];
        
end




sp.design = start.design;
sp.baseRotate = false;
sp.start_conf = start.matrix;
sp.j = size(sp.start_conf, 1);
sp.goal_conf = sp.goals(1:sp.j, 1:3);
sp.home_base = [0,0,0];


% timeSize = 5;
% times = zeros(20, 1);
% 
% sizes = [50, 100, 500, 1000];
% sizes = 1000;
% 
% solutions = zeros(timeSize, 2);
% 
% for size = sizes
%     for i = 1:timeSize
%         tic
%         [solution, expandedNodes] = searchAlgorithm(sp, size);
%         time = toc;
% 
%         solutions(i, 1) = solution.f;
%         solutions(i, 2) = time;
% 
%         disp("Size: " + size);
%         disp("Iteration: " + i);
%         disp("Time: " + time);
%         disp(" ");
%         if isempty(solution)
%             disp("No Solution");
%             return;
%         end 
%     end
% end

% sp.obstacles = [];
% [path, cost] = directExpansion(sp.start_conf, sp.goal_conf, sp);
% solution.path = pathConversion1(path);
% solution.g = 0;
% solution.f = 0;
% sollution.h = 0;


% lengthSum = 0;
% for i = 1:size(sp.design)
%     length = sp.design(i);
%     lengthSum = lengthSum + length;
% end
% 
% randConfig = randomConf(sp, rand(1) * lengthSum);
% 
% [solution, expandedNodes] = searchAlgorithm(sp, 1000);
% 
% expandedNodes
% solution.g

sp.obstacles = [10000, 10000, 0.1, 0.1, 0.1];

pathsSteps = {};
pathsSteps{1, 1} = size(directExpansion(sp.start_conf, sp.goal_conf, sp), 2) - 1;
pathsSteps{2, 1} = totalStep(sp.start_conf, sp.goal_conf, sp);
for i = 2:100
    randConf1 = randomConf(sp);
    randConf2 = randomConf(sp);
    
    [path, ~] = directExpansion(randConf1, randConf2, sp);
    pathsSteps{1, i} = size(path, 2) - 1;
    pathsSteps{2, i} = totalStep(randConf1, randConf2, sp);
    if pathsSteps{1, i} ~= pathsSteps{2, i}
        x = 10;
    end
end


sp.obstacles = [
    -150, 0, 500, 25, 100;
    -150, 50, 500, 25, 100;
    -150, 100, 500, 25, 100;

    -150, -50, 500, 25, 100;
    -150, -100, 500, 25, 100;
    ];




rrtConf.pOfGoal = 0.95;
rrtConf.numOfNodes = 1000;
rrtConf.stepSize = 10;
[path, cost] = searchAlgorithmRRT(sp, rrtConf);
solution.path = pathConversion1(path);
solution.g = cost;
solution.f = solution.g;
solution.h = 0;

% solution = searchAlgorithm(sp, 1000);

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

 softRobot_animation(formattedPathForAnimation, [0,0,0], true, sp);








