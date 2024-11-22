function setCombinations(pOfGoal_start, pOfGoal_end, pOfGoal_increase, step_start, step_end, step_increase)
    combinations_RRT = {};
    pOfGoal_num = ((pOfGoal_end - pOfGoal_start)/pOfGoal_increase)+1;
    step_num = ((step_end - step_start)/step_increase)+1;

    for i=0:(pOfGoal_num*step_num)-1
        combinations_RRT{i+1}.pOfGoal = pOfGoal_start + ((mod(i, pOfGoal_num)) * pOfGoal_increase);
        combinations_RRT{i+1}.stepSize = step_start + (floor(i / step_num) * step_increase);
    end
    save('combinations_RRT.mat', 'combinations_RRT')
end