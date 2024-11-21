
% This simple class represents path of configuration.
% Path of configuration is paired with cost to go from first element to
% last, and is updated each time new configuration is added.
classdef Path < matlab.mixin.Copyable
    properties
        p
        cost
    end

    methods
        function obj = Path(path, cost)
            global sp;

            switch nargin
                case 0
                    obj.p = {};
                    obj.cost = 0;
                case 1
                    obj.p = path;
                    obj.cost = 0;
                    for i = 1:size(path, 2) - 1
                        obj.cost = obj.cost + calculateCost(sp, path{i}, path{i + 1});
                    end
                case 2
                    obj.p = path;
                    obj.cost = cost;
                otherwise
                    error("Not enough input arguments to construct: Path");
            end
        end

        function add(obj, config)
            global sp;

            obj.p{end + 1} = config;
            if size(obj.p, 2) == 1
                obj.cost = 0;
            else
                obj.cost = obj.cost + calculateCost(obj.p{end - 1}, obj.p{end});
            end
        end
        
        function addition = plus(obj, config)
            arguments
                obj
                config Configuration
            end
            addition = copy(obj);
            addition.add(config);
        end
    end

    methods (Static)
        function [cost] = calculateCost(conf_a, conf_b)
            global sp;

            switch sp.metric
                case "space"
                    cost = calculateCostSpace(sp, conf_a, conf_b);
                case "time"
                    cost = calculateCostTime(sp, conf_a, conf_b);
                otherwise
                    error("'" + sp.metric + "' is not a valid metric.");
            end
        end

        function [cost] = calculateCostSpace(conf_a, conf_b)
            home_base = sp.home_base;

            j = size(conf_a,1);
            conf_a_cc = conf_a.solveFK();
            conf_b_cc = conf_b.solveFK();

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

        function timeCost = calculateCostTime(conf_a, conf_b)
            confA = conf_a.j_space;
            conB = conf_a.j_space;

            times = sp.times; % Steering: rad/time, Retraction: m/time, Growth: m/time

            angles1 = confA(:, 1:2);
            angles2 = conB(:, 1:2);

            diffAngles = abs(angles1 - angles2);

            timeCost = 0;
            for i = 1:size(diffAngles, 1)
                timeCost = timeCost + (max(diffAngles(i, 1), diffAngles(i, 2))) / times(1);
            end

            % totalAngleDiff = sum(sum(diffAngles));
            % timeCost = totalAngleDiff / times(1);

            length1 = sum(confA(:, 3));
            length2 = sum(conB(:, 3));

            diffLength = length1 - length2;
            if diffLength > 0
                timeCost = timeCost + diffLength / times(3);
            else
                timeCost = timeCost + (-diffLength / times(2));
            end
        end
    end

end

