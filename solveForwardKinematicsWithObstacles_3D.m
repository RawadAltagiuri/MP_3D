% This function solves the forward kinematics of a 3d soft growing robot given angles and link lengths
%
% INPUT:
% 'robot_PC' nx3 contains the configuration of a soft robot in polar coordinates, for each joint: rotation on x, rotation on y, length of the link
% 'home_base' 1x3 contains the coordinates x,y,z of the container. robot will grow in z direction
% 'draw_plot' is a boolean flag, true if you want to plot the segments from targets
% 'osbtacles' nx5 contains the coordinates x,y,z , the radius and the height of the obstacles
% OUTPUT:
% 'robot_CC' (n+1)x3 contains the configuration of a soft robot in cartesian coordinates, for each joint: x, y, z (starting from home base, last one is the end effector)
% 'u' 1x3 is the unit vector of the end effector's orientation
function [robot_CC, u] = solveForwardKinematicsWithObstacles_3D(robot_PC, home_base, draw_plot, osbtacles)
    
    n_joints = size(robot_PC,1); % number of joints of the robot
    robot_CC = zeros(n_joints+1,3);   % initializing the 
    n_obstacles = size(osbtacles,1); % number of the obstacles in the area
    
    if draw_plot==true
        f = figure;
        hold on;
        axis equal;
        grid on;
        xlabel('x');
        ylabel('y');
        zlabel('z');
        xlim([-400 400]);
        ylim([-400 400]);
        zlim([-100 1000]);
        %draw home
        plot3(0,0,0,'--gs','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','b');
    end
    
    p = [home_base(1:3) 1]; % starting point
    A = eye(4); % Transformation Matrix Initialization as an identity matrix
    
    robot_CC(1,:) = home_base(1:3);
    
    for i=1:1:n_obstacles
        %drawing the obstacles over the area according to the x y z
        %coordinates, the radius and the height of the obstacle 
        [X,Y,Z] = cylinder(osbtacles(i,4)); % setting a cylinder at the x y and z coordinates with the specified radius 
        X = X + osbtacles(i,1); % setting the x coordinate
        Y = Y + osbtacles(i,2); % setting the x coordinate
        Z = Z*-osbtacles(i,5) + osbtacles(i,3); % setting the z coordinate so that it' starts from the z plane value up to z + height 
        surf(X, Y, Z, 'FaceColor', 'k', 'EdgeColor', 'none');
        plot3(X,Y,Z,'Color','w');
        th = 0:pi/50:2*pi;
        xunit = osbtacles(i,4) * cos(th) + osbtacles(i,1);
        yunit = osbtacles(i,4) * sin(th) + osbtacles(i,2);
        zunit = 0*th + osbtacles(i,3);
        plot3(xunit, yunit, zunit,'Color','k');
        plot3(xunit, yunit, (zunit-osbtacles(i,5)),'Color','k');
    end
    for i=1:1:n_joints
        % Reference Frame change: rotate on x of -90°
        theta = 0;              % rotate on z                 
        d = 0;                  % translate on z
        a = 0;                  % translate on x
        alpha = deg2rad(-90);	% rotate on x

        A = A * getA(theta, d, a, alpha);
        ee = (A*p')';

        % joint actuation
        theta = deg2rad(robot_PC(i,2));	% rotate on z (beta)              
        d = 0;                          % translate on z
        a = 0;                          % translate on x
        alpha = deg2rad(robot_PC(i,1)) + deg2rad(90);	% rotate on x (alpha)

        A = A * getA(theta, d, a, alpha);
        ee = (A*p')';

        % robot growth
        theta = 0;              % rotate on z                 
        d = robot_PC(i,3);      % translate on z (link length)
        a = 0;                  % translate on x
        alpha = 0;	% rotate on x

        A = A * getA(theta, d, a, alpha);
        ee = (A*p')';

        robot_CC(i+1,:) = ee(1:3);  % add point to final configuration
    end
    
    
    u = (robot_CC(n_joints+1,:) - robot_CC(n_joints,:))/norm(robot_CC(n_joints+1,:) - robot_CC(n_joints,:)); % orientation of the end effector
    
    if draw_plot==true
        for i=2:1:n_joints+1
            plot3([robot_CC(i-1,1),robot_CC(i,1)],[robot_CC(i-1,2),robot_CC(i,2)],[robot_CC(i-1,3),robot_CC(i,3)],'-o','Color','r', 'LineWidth', 1.5);
        end
        f.CurrentAxes.ZDir = 'Reverse';
        cameratoolbar('SetCoordSys','x');
        view(48.5877551020408,31.2);

    end
    
end

% Get the transformation matrix
function [A] = getA(theta, d, a, alpha)
    
    A = [
            cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0, sin(alpha), cos(alpha), d;
            0, 0, 0, 1;
    ];
end