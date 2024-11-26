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
rrtConf.iteration = 1000;
rrtConf.stepSize = 5;
rrtConf.neighbourSize = calculateNeighbourSize(sp) * 10;
rrtConf.pOfGoal = 0.2;
rrtConf.pOfVC = 0.2;


[rrtSol] = searchAlgorithmBiRRT_star(sp, rrtConf, false);


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