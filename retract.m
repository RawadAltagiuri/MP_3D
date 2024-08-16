%{
Takes sp, configuration and amount of retraction and returns the resulting
configuration from retraction config that amount.
Important thing to notice here is that this function does NOT take any
robot constraints into account. That is, retraction is simply the collapse
of the last link.

Inputs:
- conf, config to retract from
- amountOfRet, amount to retraction

Outputs:
- config, the resulting config from retraction of the previous one
%}

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