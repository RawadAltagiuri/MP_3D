clc, clear
load envs

% inputs: sp, number of iterations in each combination, number of combinations that would be tested

% sp = envs{3};
% sp.baseRotate = false;
run_num = 2;

for i=1:size(envs, 2)
    sp = envs{i};
    tabulation(sp, run_num, 2);
end