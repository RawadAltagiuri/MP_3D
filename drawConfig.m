function drawConfig(config, sp, color)
    n_joints = size(config, 1);
    robot_CC = solveForwardKinematics_3D(config, sp.home_base, false); %solve the forward kinematics for a given robot configuration

    % draws the soft robot
    for i=2:1:n_joints+1
        plot3([robot_CC(i-1,1),robot_CC(i,1)],[robot_CC(i-1,2),robot_CC(i,2)],[robot_CC(i-1,3),robot_CC(i,3)],'-o','Color',color, 'LineWidth', 1.5);
    end

    % f.CurrentAxes.ZDir = 'Reverse';
    % cameratoolbar('SetCoordSys','x');
    % view(60, 30)
end