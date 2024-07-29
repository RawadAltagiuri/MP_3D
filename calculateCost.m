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
function cost = calculateCost(sp, conf_a, conf_b)
    cost = 0;   
    a_link_num = 0;
    for i=2:1:size(conf_a, 1) % get the link number of conf_a
        if conf_a(i, 3) == 0
            a_link_num = i-1;
            break;
        end
        a_link_num = a_link_num+1;
    end

    b_link_num = 0;
    for i=2:1:size(conf_b, 1) % get the link number of conf_b
        if conf_b(i, 3) == 0
            b_link_num = i-1;
            break;
        end
        b_link_num = b_link_num+1;
    end

    if a_link_num > b_link_num % if the length is decreasing
        for i=a_link_num:-1:b_link_num+1
            cost = cost + conf_a(i, 3);
            conf_a(i, :) = [0 0 0];
        end
    elseif a_link_num < b_link_num % if the length is increasing
        % there are two scenarios: Link length is bigger or smaller than the sp.lenthMin
        if conf_a(a_link_num, 3) < sp.lengthMin
            for i=a_link_num:1:b_link_num
                cost = cost + sp.lengthMin - conf_a(i, 3);  % get the cost of eversion till the sp.lengthMin
                conf_a(i, 3) = sp.lengthMin;                % update the conf_a as it has been longated
                temp_conf = conf_b;                         % create a temp config with the same joint number as conf_a (as the conf_b is shortened)
                temp_conf(i+1:end, :) = 0;
                cost = cost + heuristic(conf_a, temp_conf, sp.home_base);   % get the heuristic of the movement and update the conf_a
                conf_a = temp_conf;
            end
        else
            % if the last link length is bigger than the sp.lengthMin, we just
            % get the heuristic of the movement first, without considering the eversion
            temp_conf = conf_b;
            temp_conf(a_link_num+1:end, :) = 0;
            cost = cost + heuristic(conf_a, temp_conf, sp.home_base);
            conf_a = temp_conf;
            for i=a_link_num+1:1:b_link_num
                cost = cost + sp.lengthMin - conf_a(i, 3);
                conf_a(i, 3) = sp.lengthMin;
                temp_conf = conf_b;
                temp_conf(i+1:end, :) = 0;
                cost = cost + heuristic(conf_a, temp_conf, sp.home_base);
                conf_a = temp_conf;
            end
        end
    else    % if both configurations have same joint number
        if conf_a == conf_b % if the configs are same, cost = 0
            cost = 0;
            return;
        end
        if conf_a(a_link_num, 3) < sp.lengthMin     % first, control the last link length if it is smaller than the sp.lengthMin
            cost = cost + sp.lengthMin - conf_a(i, 3);
            conf_a(i, 3) = sp.lengthMin;
        end
        cost = cost + heuristic(conf_a, conf_b, sp.home_base);
    end
end

function [cost] = heuristic(conf_a, conf_b, home_base)
    j = size(conf_a,1);
    [conf_a_cc, ~] = solveForwardKinematics_3D(conf_a, home_base, false);
    [conf_b_cc, ~] = solveForwardKinematics_3D(conf_b, home_base, false);

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
    ee_max = max(ee_a,ee_b); % this is to repeat the coordinate of the end effector over the last joints that are still rolled inside the robot if one configuration is longer than the other

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
