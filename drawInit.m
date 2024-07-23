function drawInit(startConfig, goalConfig, sp)
    f = figure;

    grayRobotColor = '#569c69';

    home_base = sp.home_base;
    n_obstacles = size(sp.obstacles, 1);

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
    [startConf,~] = solveForwardKinematics_3D(startConfig,home_base,false);
    [goalConf,~] = solveForwardKinematics_3D(goalConfig,home_base,false);

    n_joints = size(startConfig,1);

    for i=2:1:n_joints+1
        plot3([startConf(i-1,1),startConf(i,1)],[startConf(i-1,2),startConf(i,2)],[startConf(i-1,3),startConf(i,3)],'-o','Color','r', 'LineWidth', 1.5);
    end
    for i=2:1:n_joints+1
        plot3([goalConf(i-1,1),goalConf(i,1)],[goalConf(i-1,2),goalConf(i,2)],[goalConf(i-1,3),goalConf(i,3)],'-o','Color','g', 'LineWidth', 1.5);
    end    

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
end
