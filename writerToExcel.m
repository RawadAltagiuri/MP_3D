clear

load scenarios
sp = scenarios{3};
sp.baseRotate = false;
run_num = 2;

% inputs: sp, number of iterations in each combination, number of combinations that would be tested
tabulation(sp, run_num, 3);