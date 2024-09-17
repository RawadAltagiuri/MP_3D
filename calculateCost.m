% This function calculate the cost to go from a configuration A to a configuration B.
% It can be used as heuristic if configuration B is the goal.
%
% INPUT:
% 'conf_a' jx3 contains the configuration of a soft robot in polar coordinates, for each joint: rotation on x, rotation on y, length of the link
% 'conf_b' jx3 contains the configuration of a soft robot in polar coordinates, for each joint: rotation on x, rotation on y, length of the link
% 'home_base' 1x3 contains the coordinates x,y,z of the container. robot will grow in z direction
%
% OUTPUT:
% 'cost' the cost, expressed as the sum of euclidean distances between each joint (including the end effector)
function [cost] = calculateCost(sp, conf_a, conf_b)
    switch sp.metric
        case "space"
            cost = calculateCostSpace(sp, conf_a, conf_b);
        case "time"
            cost = calculateCostTime(sp, conf_a, conf_b);
        otherwise
            error("'" + sp.metric + "' is not a valid metric.");
    end
end

function [cost] = calculateCostSpace(sp, conf_a, conf_b)
    home_base = sp.home_base;
    j = size(conf_a,1);
    conf_a_cc = solveForwardKinematics_3D(conf_a, home_base, false);
    conf_b_cc = solveForwardKinematics_3D(conf_b, home_base, false);
    
    % find the index of the end effector
    ee_a = j;
    for i=1:1:j
        if conf_a(i,3) == 0
            ee_a = i-1;
            break;
        end
    end
    
    ee_b = j;
    for i=1:1:j
        if conf_b(i,3) == 0
            ee_b = i-1;
            break;
        end
    end
    ee_a = ee_a+1;
    ee_b = ee_b+1;
    ee_max = max(ee_a,ee_b);    % this is to repeat the coordinate of the end effector over the last joints that are still rolled inside the robot if one configuration is longer than the other
    
    % cut after end effector
    conf_a_cc(ee_max+1:end,:) = [];
    conf_b_cc(ee_max+1:end,:) = [];
    
    % calculate cost
    dist = zeros(ee_max,1);
    for i=1:1:ee_max
        dist(i,:) = norm(conf_a_cc(i,:)-conf_b_cc(i,:));
    end
    cost = sum(dist);
    
end

function timeCost = calculateCostTime(sp, conf_a, conf_b)
    times = sp.times; % Steering: rad/time, Retraction: m/time, Growth: m/time

    angles1 = conf_a(:, 1:2);
    angles2 = conf_b(:, 1:2);

    diffAngles = abs(angles1 - angles2);

    totalAngleDiff = sum(sum(diffAngles));
    timeCost = totalAngleDiff / times(1);

    length1 = sum(conf_a(:, 3));
    length2 = sum(conf_b(:, 3));

    diffLength = length1 - length2;
    if diffLength > 0
        timeCost = timeCost + diffLength / times(3);
    else
        timeCost = timeCost + (-diffLength / times(2));
    end
end



