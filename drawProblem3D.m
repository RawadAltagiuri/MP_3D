           function [] = drawProblem3D(op, robot_configurations,home_base)
    % global op;

    f = figure;
    hold on;
    axis equal;
    xlabel('x');
    ylabel('y'); 
    zlabel('z');
    f.CurrentAxes.ZDir = 'Reverse';
    f.CurrentAxes.XDir = 'Reverse';
    view(-159.1605,31.6712);

    % n_targets = size(op.targets,1);
    
    %draw op.obstacles
    % n_obstacles = size(op.obstacles,1);
    % for i=1:1:n_obstacles
    %     [X,Y,Z] = cylinder(op.obstacles(i,4));
    %     X = X + op.obstacles(i,1);
    %     Y = Y + op.obstacles(i,2);
    %     Z = Z*-op.obstacles(i,5) + op.obstacles(i,3);
    %     plot3(X,Y,Z,'Color','k');
    %     th = 0:pi/50:2*pi;
    %     xunit = op.obstacles(i,4) * cos(th) + op.obstacles(i,1);
    %     yunit = op.obstacles(i,4) * sin(th) + op.obstacles(i,2);
    %     zunit = 0*th + op.obstacles(i,3);
    %     plot3(xunit, yunit, zunit,'Color','k');
    %     plot3(xunit, yunit, (zunit-op.obstacles(i,5)),'Color','k');
    % end
    
    %draw home
    plot3(home_base(1),home_base(2),home_base(3),'--gs','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','b');
    
    %draw robot configurations
    if isempty(robot_configurations) == false
        for i=1:1:size(robot_configurations,3)
            conf = robot_configurations(:,:,i);
            % xyz = solveForwardKinematics_3D_emre(conf,op.home_base,false);
            xyz = solveForwardKinematics(op, conf,op.home_base,false);
            
            for j = 1 : 1 : size(conf,1)
                plot3([xyz(j,1),xyz(j+1,1)],[xyz(j,2),xyz(j+1,2)],[xyz(j,3),xyz(j+1,3)],'-o','Color','r');
            end  
        end
    end
    
    %draw op.targets
    % for i=1:1:n_targets
    %     plot3(op.targets(i,1), op.targets(i,2), op.targets(i,3),'-x','Color','b', 'LineWidth', 8);  
    % 
    %     %draw endpoints
    %     plot3([op.end_points(i,1),op.targets(i,1)],[op.end_points(i,2),op.targets(i,2)],[op.end_points(i,3),op.targets(i,3)],'--o','Color','b');
    % 
    % end
    arrowSegmentAmount = 10;
    % for j = 1:size(op.targets(), 1)
    %     drawSegmentationArrow(op.end_points,op.targets, arrowSegmentAmount, j)
    % end
    
%     set(h1, 'Zdir', 'reverse');
    
end