function [child] = RrtExpand(sp)

    %Generate a random number between 0 and 1, if it is less than sp.weight, return the goal configuration 
    if rand() < sp.weight
        child = sp.goal_conf;
        disp("yes");
        return;
    end
    
    numJoints = sp.j;
    
    randomConf = zeros(numJoints, 3); % Each row: rotation on x, rotation on y, length of the link

    maxLength = 0;
    %add up the design maximum length of each of the links
    for i = 1:numJoints
        maxLength = maxLength + sp.design(i);
    end
    
    %generate a random length between 0 and the maximum length
    randomLength = rand() * maxLength;
    

    %while randomlength != 0, keep adding from it to the robot until it is 0
    addedSoFar = 0;
    for i = 1:numJoints
        if randomLength - addedSoFar > sp.design(i)
            randomConf(i, 3) = sp.design(i);
            addedSoFar = addedSoFar + sp.design(i);
        else
            randomConf(i, 3) = randomLength - addedSoFar;
            addedSoFar = randomLength;
        end

        %generate random angles for the x and y rotations between the sp.SteerBounds, first index is the lower bound, second index is the upper bound
        randomConf(i, 1) = sp.steerBounds(1) + (sp.steerBounds(2) - sp.steerBounds(1)) * rand();
        randomConf(i, 2) = sp.steerBounds(1) + (sp.steerBounds(2) - sp.steerBounds(1)) * rand();

    end

    child = randomConf;
end