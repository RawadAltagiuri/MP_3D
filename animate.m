function animate(sp, path)
    if iscell(path)
        path = pathConversionC_M(path);
    end

    % Calculate the number of submatrices you will create
    numSubMatrices = size(path, 2) / 3;

    % Preallocate the 3D array to store the submatrices
    formattedPath = zeros(sp.j, 3, numSubMatrices);

    % Extract the submatrices and store them in the 3D array
    for i = 1:numSubMatrices
        formattedPath(:, :, i) = path(:, (i-1)*3 + 1 : i*3);
    end

    growthCount = 0;
    retractCount = 0;
    steerCount = 0;
    for i=2:size(formattedPath,3)
        [growthCount, retractCount, steerCount] = actionCounter(formattedPath(:, :, i), formattedPath(:, :, i-1), growthCount, retractCount, steerCount);
    end

    softRobot_animation(sp, formattedPath);
end

function animateFast(sp, formattedPath)
    home_base = sp.home_base;
    steps = size(formattedPath, 3);       % number of steps of motion
    n_joints = size(formattedPath, 1);    % number of joints of the robot
    
    end_effectors = zeros(steps,3); % end effector array that will contain the coordinates of the end effector for each step of motion
    
    f = figure;
    n_obstacles = size(sp.obstacles,1);

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

    f.CurrentAxes.ZDir = 'Reverse';
    cameratoolbar('SetCoordSys','x');
    view(60, 30)

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

    prevRobot_CC = [];
    for k=1:1:steps
        startConf = solveForwardKinematics_3D(formattedPath(:,:,1),home_base,false);
        robot_CC = solveForwardKinematics_3D(formattedPath(:,:,k),home_base,false); %solve the forward kinematics for a given robot configuration
        
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

        % erases (fades to the background) the previous soft robot state
        if ~isempty(prevRobot_CC)
            for i=2:1:n_joints+1
                plot3([prevRobot_CC(i-1,1),prevRobot_CC(i,1)],[prevRobot_CC(i-1,2),prevRobot_CC(i,2)],[prevRobot_CC(i-1,3),prevRobot_CC(i,3)],'-o','Color','w', 'LineWidth', 1.5);
            end
        end
        
        % draws the path from the end effector array
        hexCode = '#FFA500';
        for j=1:3:k
            plot3(end_effectors(j,1),end_effectors(j,2),end_effectors(j,3),'.','Color','b');
        end

        prevRobot_CC = robot_CC;
        pause(0.1); %change this to make the animation faster/slower
    end
end

