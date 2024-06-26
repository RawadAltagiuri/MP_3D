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
end

