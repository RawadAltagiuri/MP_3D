clear, clc, close;

load envs
sp = envs{5};
sp.baseRotate = false;
sp.heuristicLimit = 1;


rrtConf.pOfGoal = 0.1;
rrtConf.numOfNodes = 1000;
rrtConf.stepSize = 1;

tic;
[path, cost, tree, final_child] = searchAlgorithmRRT_star(sp, rrtConf, false);
time = toc
solution.path = pathConversion1(path);
solution.g = cost;
solution.f = solution.g;
solution.h = 0;

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
