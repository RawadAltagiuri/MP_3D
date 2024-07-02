function [stepsAreCorrects] = checkIfStepsAreCorrects(formattedPathForAnimation, sp)
    stepsAreCorrects = true
    for i=2:size(formattedPathForAnimation,3)
        %check the absolute between the matrixes, if any of the values in the first two columns are greater than sp.stepSize(1) or any in the third column is greater than sp.stepSize(2) then the steps are not correct
        if any(abs(formattedPathForAnimation(:,1,i) - formattedPathForAnimation(:,1,i-1)) > sp.stepSize(1)) || any(abs(formattedPathForAnimation(:,2,i) - formattedPathForAnimation(:,2,i-1)) > sp.stepSize(1)) || any(abs(formattedPathForAnimation(:,3,i) - formattedPathForAnimation(:,3,i-1)) > sp.stepSize(2))
            stepsAreCorrects = false
            break
        end
    end
end