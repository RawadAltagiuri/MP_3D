function sp = preMotion(sp)
    config = sp.start_conf;
    for lastJoint = size(config, 1):-1:1
        if ~isequal(config(lastJoint, 1:2), [0, 0])
            sp.start_conf(lastJoint:end, :) = 0;
            sp.goal_conf(lastJoint:end, :) = 0;
            return;
        end
    end
end

