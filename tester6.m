clear, clc, close;

load envs
sp = envs{1};
sp.baseRotate = false;
sp.heuristicLimit = 1;


rrtConf.pOfGoal = 1;
rrtConf.numOfNodes = 100;
rrtConf.stepSize = 1;
rrtConf.neighbourSize = calculateNeighbourSize(sp.start_conf, sp.goal_conf, sp);

tic;
[path, cost, tree, final_child] = searchAlgorithmRRT_star(sp, rrtConf, false);
time = toc
solution.path = pathConversion1(path);
solution.g = cost;
solution.f = solution.g;
solution.h = 0;


% solution.path = pathToRetraction(sp, sp.goal_conf, 125);
% solution.path = pathConversion1(solution.path);

sp.start_conf = [
    0	0	50
-45	45	175
0	0	175
25	-25	175
20	0	10
    ];
sp.goal_conf = [
    0	0	50
-45	45	175
0	0	175
25	-25	175
-20	20	20
    ];

[path, cost] = directExpansion(sp, realmax, sp.start_conf, sp.goal_conf);
solution.path = pathConversion1(path);

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

