% This function creates an animation for the soft robot motion planner, given the motion commands to be executed (which are retreived with A*)
%
% INPUT:
% 'commands' nx3xs contains an array of configurations of a soft robot in polar coordinates, for each of the 'n' joint: rotation on x, rotation on y, length of the link, for a total of 's' configurations
% 'home_base' 1x3 contains the coordinates x,y,z of the container. robot will grow in z direction
%
% DISCLAIMER:
% It must me updated to draw obstacles when we will add any in the future
function [] = softRobot_animation(commands, home_base, drawPath, sp)

 
    
    steps = size(commands,3);       % number of steps of motion
    n_joints = size(commands,1);    % number of joints of the robot
    
    end_effectors = zeros(steps,3); % end effector array that will contain the coordinates of the end effector for each step of motion
    
    f = figure;
    n_obstacles = size(sp.obstacles,1);


    for k=1:1:steps
        clf;
        hold on;
        axis equal;
        grid on;
        xlabel('x');
        ylabel('y');
        zlabel('z');
        xlim([-400 400]);
        ylim([-400 400]);
        zlim([-100 1000]);
        plot3(home_base(1),home_base(2),home_base(3),'--gs','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','b'); %draw home
        [startConf,~] = solveForwardKinematics_3D(commands(:,:,1),home_base,false);
        [robot_CC,~] = solveForwardKinematics_3D(commands(:,:,k),home_base,false); %solve the forward kinematics for a given robot configuration
        
        % collect the end effector coordinates for each step of motion to draw the path of the robot 
        end_effectors(k,:) = robot_CC(n_joints+1,:); 
        
        %draw the start configuration
        grayRobotColor = '#569c69';
        for i=2:1:n_joints+1
            plot3([startConf(i-1,1),startConf(i,1)],[startConf(i-1,2),startConf(i,2)],[startConf(i-1,3),startConf(i,3)],'-o','Color',grayRobotColor, 'LineWidth', 1.5);
        end
        % draws the soft robot
        for i=2:1:n_joints+1
            plot3([robot_CC(i-1,1),robot_CC(i,1)],[robot_CC(i-1,2),robot_CC(i,2)],[robot_CC(i-1,3),robot_CC(i,3)],'-o','Color','r', 'LineWidth', 1.5);
        end
        
        % draws the path from the end effector array
        if drawPath == true
            hexCode = '#FFA500';
            for j=1:3:k
                plot3(end_effectors(j,1),end_effectors(j,2),end_effectors(j,3),'.','Color','b');
            end
        end

        % drawing the obstacles for the animation visualizing 
        for i = 1:1:n_obstacles
            [X, Y, Z] = cylinder(sp.obstacles(i, 4), 10);
            X = X + sp.obstacles(i, 1);
            Y = Y + sp.obstacles(i, 2);
            Z = Z * -sp.obstacles(i, 5) + sp.obstacles(i, 3);

            surf(X, Y, Z, 'FaceColor', 'w', 'EdgeColor', 'none');
            grayColor = '#778079';
            plot3(X,Y,Z,'Color',grayColor);
            th = 0:pi / 50:2 * pi;
            xunit = sp.obstacles(i, 4) * cos(th) + sp.obstacles(i, 1);
            yunit = sp.obstacles(i, 4) * sin(th) + sp.obstacles(i, 2);
            zunit = 0 * th + sp.obstacles(i, 3);

            plot3(xunit, yunit, zunit,'Color',grayColor);
            plot3(xunit, yunit, (zunit-sp.obstacles(i,5)),'Color',grayColor);
        end


        f.CurrentAxes.ZDir = 'Reverse';
        cameratoolbar('SetCoordSys','x');
        view(60, 30)

        pause(0); %change this to make the animation faster/slower
    end
    
end

