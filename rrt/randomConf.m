%{
Generates a random configuration according to robot's design parameters.

Generated random configuration is valid in a sense that a complete
path can be constructed from any other valid configuration to this
one and visa-versa.

@omerk
%}
function conf = randomConf(sp)
    lengthSum = 0;
    for i = 2:size(sp.design)
        length = sp.design(i);
        lengthSum = lengthSum + length;
    end
    length = rand(1) * lengthSum + sp.design(1);

    steerRange = sp.steerBounds(2) - sp.steerBounds(1);

    conf = zeros(size(sp.design, 1), 3);
    conf(1, :) = [0, 0, sp.design(1)];
    remindingLength = length - sp.design(1);
    for i = 2:size(sp.design, 1)
        if remindingLength < sp.design(i)
            conf(i, :) = [rand(1) * steerRange + sp.steerBounds(1), rand(1) * steerRange + sp.steerBounds(1), remindingLength];
            for j = (i + 1):size(sp.design, 1)
                conf(j, :) = [0, 0, 0];
            end
            break;
        else
            conf(i, :) = [rand(1) * steerRange + sp.steerBounds(1), rand(1) * steerRange + sp.steerBounds(1), sp.design(i)];
            remindingLength = remindingLength - sp.design(i);
        end
    end

    % Accomodate for gripper.
    lastExpanded = -1;
    for i = size(conf, 1):-1:1
        if conf(i, 3) > 0.0001
            lastExpanded = i;
            break;
        end
    end

    if conf(lastExpanded, 3) < sp.lengthMin
        conf(lastExpanded, 1:2) = 0;
    end
end