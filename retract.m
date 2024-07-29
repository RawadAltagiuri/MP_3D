function conf = retract(conf, amountOfRet)
    for lastExpanded = size(conf, 1):-1:1
        if conf(lastExpanded, 3) ~= 0
            break;
        end
    end

    while amountOfRet > 0 && lastExpanded > 0 
        if conf(lastExpanded, 3) > amountOfRet
            conf(lastExpanded, 3) = conf(lastExpanded, 3) - amountOfRet;
            break;
        else
            amountOfRet = amountOfRet - conf(lastExpanded, 3);
            conf(lastExpanded, :) = 0;
            lastExpanded = lastExpanded - 1;
        end
    end
end