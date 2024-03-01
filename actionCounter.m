% This function updates the number of growths, retracts, and steers that
% are done so far during the motion
% INPUT:
% 'newConf' jx3 contains the configuration of a soft robot in polar coordinates, for each joint: rotation on x, rotation on y, length of the link
% 'prevConf' jx3 contains the configuration of a soft robot in polar coordinates, for each joint: rotation on x, rotation on y, length of the link
% 'growth' the current number of growths that are done so far
% 'retract' the current number of retractions so far
% 'steer' the current number of steerings so far
% OUTPUT: 
% 'growth' the current number of growths that are done after updating
% 'retract' the current number of retractions after updating
% 'steer' the current number of steerings after updating
function [growth, retract, steer] = actionCounter(newConf, prevConf, growth, retract, steer)
    if sum(newConf(:, 3)) > sum(prevConf(:, 3)) % if there is an increase in the third column value between the new and the previous configurations, then it increases the growth count 
        growth = growth+1;
        steer= steer;
        retract = retract;
    elseif sum(newConf(:, 3)) < sum(prevConf(:, 3)) % if there is a decrease in the third column value between the new and the previous configurations, then it increases the retract count
        growth = growth;
        retract = retract +1;
        steer= steer;
    end
    if ~isequal(newConf(:, 1:2), prevConf(:, 1:2)) % if there is an change in the first or the second column value between the new and the previous configurations, then it increases the steer count
        growth = growth;
        retract = retract;
        steer = steer + 1;
    end
end