%{
Generates a sub-sequent configuration to go from 'config'
to 'goal'.
Prefered order of action is:
 1- Retraction,
 2- Steer,
 3- Grow,
In this preference list, if robot can do the prior action, then it will
generate the result configuration of the action.
It should be noted that due to physical and other constraints of the robot,
even though a specific action is chosen, the resulting configuration may 
be not from that specific action. For example, even if 'config's length is smaller
than 'goal's, and top priority is retraction, it may not be able to
retract because it first need to steer to some position, then it will
perform steering action in the name of 'Retraction'.

Inputs:
- sp, the problem specification
- config, the configuration to expand from, it is a sp.j x 3 matrix.
- goal, the goal configuration to expand to, its structure is same as
config.

Outputs:
- Resulting configuration from expansion from config to goal.
%}
function nextConfig = greedyExpand(sp, config, goal, actionOrder)
    arguments
        sp
        config
        goal
        actionOrder = ['R', 'S', 'G'];
    end

    
    nextConfig = [];
    for action = actionOrder
        nextConfig = doAction(sp, config, goal, action);
        if  ~isempty(nextConfig)
             return;
        end
    end
end

function config = doAction(sp, config, goal, action)
    switch action
        case 'S'
            config = steering(sp, config, goal);
        case 'R'
            config = retraction(sp, config, goal);
        case 'G'
            config = growing(sp, config, goal);
        otherwise
            error("Action: '" + action + "' is not defined.");
    end
end

function config = steering(sp, config, goal)
    lastExpanded = -1;
    for i = size(config, 1):-1:1
        if config(i, 3) > 0.0001
            lastExpanded = i;
            break,
        end
    end
    diffAngles = goal(1:lastExpanded, 1:2) - config(1:lastExpanded, 1:2);

    if isequal(diffAngles, zeros(lastExpanded, 2))
        config = [];
        return;
    end 

    if config(lastExpanded, 3) < sp.lengthMin && ~isequal(config(lastExpanded, 1:2) - goal(lastExpanded, 1:2), [0, 0])
        growAmount = sp.lengthMin - config(lastExpanded, 3);
        growAmount = min(sp.stepSize(2), growAmount);

        config = grow(sp, config, growAmount);
    else
        % jointIndex = 1;
        % angleIndex = 1;
        % maxAngle = abs(diffAngles(1, 1));
        % for i = 1:size(diffAngles, 1)
        %     for j = 1:size(diffAngles, 2)
        %         if abs(diffAngles(i, j)) > maxAngle
        %             maxAngle = abs(diffAngles(i, j));
        %             jointIndex = i;
        %             angleIndex = j;
        %         end
        %     end
        % end
        % 
        % if diffAngles(jointIndex, angleIndex) > 0
        %     config(jointIndex, angleIndex) = config(jointIndex, angleIndex) + min(diffAngles(jointIndex, angleIndex), sp.stepSize(1));
        % else
        %     config(jointIndex, angleIndex) = config(jointIndex, angleIndex) + max(diffAngles(jointIndex, angleIndex), -sp.stepSize(1));
        % end

        angleChanged = false;
        for i = 1:size(diffAngles, 1)
            if angleChanged
                break;
            end

            if diffAngles(i, 1) ~= 0
                angleChanged = true;

                if diffAngles(i, 1) > 0
                    config(i, 1) = config(i, 1) + min(diffAngles(i, 1), sp.stepSize(1));
                else
                    config(i, 1) = config(i, 1) + max(diffAngles(i, 1), -sp.stepSize(1));
                end
            end
            if diffAngles(i, 2) ~= 0
                angleChanged = true;

                if diffAngles(i, 2) > 0
                    config(i, 2) = config(i, 2) + min(diffAngles(i, 2), sp.stepSize(1));
                else
                    config(i, 2) = config(i, 2) + max(diffAngles(i, 2), -sp.stepSize(1));
                end
            end
        end

        % for i = 1:size(diffAngles, 1) * size(diffAngles, 2)
        %     if diffAngles(i) > 0
        %         diffAngles(i) = min(diffAngles(i), sp.stepSize(1));
        %     else
        %         diffAngles(i) = max(diffAngles(i), -sp.stepSize(1));
        %     end
        % end
        % config(1:lastExpanded, 1:2) = config(1:lastExpanded, 1:2) + diffAngles;
    end
end

function config = retraction(sp, config, goal)
    lastExpanded = -1;
    for i = size(config, 1):-1:1
        if config(i, 3) > 0.0001
            lastExpanded = i;
            break,
        end
    end
    diffLength = sum(goal(:, 3)) - sum(config(:, 3));

    if abs(diffLength) < 0.0001 || diffLength >= 0
        config = [];
        return;
    end

    retAmount = max(diffLength, -sp.stepSize(2));
    if config(lastExpanded, 3) + retAmount < sp.lengthMin && ~isequal(config(lastExpanded, 1:2), [0, 0])
        angleAmounts = -config(lastExpanded, 1:2);
        for i = 1:size(angleAmounts, 2)
            angleAmount = angleAmounts(i);
            if angleAmount > 0
                angleAmount = min(angleAmount, sp.stepSize(1));
            else
                angleAmount = max(angleAmount, -sp.stepSize(1));
            end
            angleAmounts(i) = angleAmount;
        end
        config(lastExpanded, 1:2) = config(lastExpanded, 1:2) + angleAmounts;
    else
        config = retract(config, -retAmount);
    end
end

function config = growing(sp, config, goal)
    diffLength = sum(goal(:, 3)) - sum(config(:, 3));

    if abs(diffLength) < 0.0001 || diffLength < 0
        config = [];
        return;
    end
    
    growAmount = min(diffLength, sp.stepSize(2));
    config = grow(sp, config, growAmount);
end
