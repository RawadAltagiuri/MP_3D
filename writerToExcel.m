clc, clear
load envs

% inputs: sp, number of iterations in each combination, number of combinations that would be tested
run_num = 10;

% tabulation(envs{3}, run_num);


for i=1:size(envs, 2)
    sp_counter = i      
    sp = envs{i};
    tabulation(sp, run_num);
end