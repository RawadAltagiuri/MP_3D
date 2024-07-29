function [path, cost] = pathToRetraction(sp, conf, finalLength)
    path = {conf};
    cost = 0;
    length = sum(conf(:, 3));
    if finalLength >= length
        return;
    end

    numOfIter = floor((length - finalLength) / sp.stepSize(2));
    reminder = mod(length - finalLength, sp.stepSize(2));
    for i = 1:numOfIter
        conf = retract(conf, sp.stepSize(2));
        
        % Alternative cost:
        % cost = cost + calculateCost(sp, path{end}, conf);
        cost = cost + calculateCost_old(path{end}, conf, sp.home_base);
        path = [path, conf];
    end

    if reminder ~= 0
        conf = retract(conf, reminder);
        % cost = cost + calculateCost(sp, path{end}, conf);
        cost = cost + calculateCost_old(path{end}, conf, sp.home_base);
        path = [path, conf];
    end

    % True cost:
    % cost = (length - finalLength) * sp.costArray(3);
end

% Assumes that minimum link length of the design is larger than amount of
% retraction and first joint of the configuration is fully extended.
function conf = retract(conf, amountOfRet)
    for i = 1:size(conf, 1)
        if i == size(conf, 1) || conf(i+1, 3) == 0
            conf(i, 3) = conf(i, 3) - amountOfRet;
            if conf(i, 3) <= 0
                conf(i - 1, 3) = conf(i - 1, 3) + conf(i, 3);
                conf(i, :) = 0;
            end
            return;
        end
    end
end
