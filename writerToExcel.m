clc, clear
load envs

% inputs: sp, number of iterations in each combination, number of combinations that would be tested
run_num = 10;

% tabulation(envs{3}, run_num);

for i=1:size(envs, 2)
    sp_counter = i      
    sp = envs{i};

    
    modSp = sp;
    paddingAmount = 5;
    modSp.obstacles(:, 4:5) = modSp.obstacles(:, 4:5) + paddingAmount;

    tabulation(modSp, run_num);
end