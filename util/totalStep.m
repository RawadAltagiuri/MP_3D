function totalStep = totalStep(conf1, conf2, sp)
    totalStep = 0;
    for i = 1:size(conf1, 1)
        if conf1(i, 3) == conf2(i, 3)
            dTheta = floor((abs(conf1(i, 1) - conf2(i, 1)) + abs(conf1(i, 2) - conf2(i, 2))) / sp.stepSize(1, 1));
            if totalStep < dTheta
                totalStep = dTheta;
            end
        else
            totalStep = totalStep + floor((abs(conf1(i, 3) - conf2(i, 3))) / sp.stepSize(2));
            if conf1(i, 3) > conf2(i, 3)
                maxConf = conf1;
            else
                maxConf = conf2;
            end

            for j = i + 1:size(conf1, 1)
                totalStep = totalStep + floor(maxConf(i, 3) / sp.stepSize(2));
                if maxConf(i, 3) == 0 || j == size(conf1, 1)
                    i = size(conf1, 1) + 1;
                    break;
                end
            end
        end
    end
end

