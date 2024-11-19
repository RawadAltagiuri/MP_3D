clear

load envs.mat
sp = envs{2};

% sp.start_conf = [0	0	50
% -45	-45	175
% -35	-30	150
% 0	0	0
% 0	0	0];
% 
% for i = 1:size(sp.obstacles, 1)
%     sp.obstacles(i, end) = sp.obstacles(i, end) - 50;
%     sp.obstacles(i, end - 1) = max(sp.obstacles(i, end - 1) - 25, 5);
% end

% sp.design = [50
% 175
% 175
% 175
% 175
% 150];

% sp.j = 6;
% 
% sp.start_conf = [0 0 50
% -45	45	175
% 0	0	175
% 25	-10	175
% 20	-15	100
% 0 0 0];
% 
% sp.goal_conf = [0	0	50
% 45	-45	175
% 0	0	175
% -25	10	175
% -20	15	175
% 0 0 90];

sp.times = [1, 0.2, 0.2];
sp.metric = "time";
paddingAmount = 5;
sp.obstacles(:, 4:5) = sp.obstacles(:, 4:5) + paddingAmount;

orgSp = sp;
sp = preMotion(sp);

[prePath, preCost] = directExpansion(sp, realmax, orgSp.start_conf, sp.start_conf);
[postPath, postCost] = directExpansion(sp, realmax, sp.goal_conf, orgSp.goal_conf);

tic
solution = searchAlgorithmA_star(sp, 10000, false);
time = toc;

solution.path = [pathConversionC_M(prePath), solution.path, pathConversionC_M(postPath)];
cost = solution.g + preCost + postCost;

animate(sp, solution.path);







