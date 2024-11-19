function [ans] = intersectObstacle_2D(s,o, draw_plot)
    
    % s(1,:) is segment end point 1 xy
    % s(2,:) is segment end point 2 xy
    % o(1:2) is obstacle xy
    % o(4) is obstacle radius
    
    if draw_plot==true
        figure;
        hold on;
        axis equal;
        xlabel('x');
        ylabel('y') ;       
        plot([s(1,1),s(2,1)],[s(1,2),s(2,2)],'-o','Color','r');
        
        th = 0:pi/50:2*pi;
        xunit = o(4) * cos(th) + o(1);
        yunit = o(4) * sin(th) + o(2);
        plot(xunit, yunit,'Color','r');
        
    end
    
    dist_segment = point2segment_2D(o(1:2), s(1,:), s(2,:), o(4)+1);
    
    dist_e1 = norm(s(1,:)-o(1:2));
    dist_e2 = norm(s(2,:)-o(1:2));
    
    if dist_segment <= o(4) || dist_e1 <= o(4) || dist_e2 <= o(4)
        ans = true;
    else
        ans = false;
    end    
end

function d = point2segment_2D(pt, v1, v2, max)
    
    pt = pt-v1;
    v2 = v2-v1;
    v1 = v1-v1;    
    angle = -atan2(v2(2)-v1(2),v2(1)-v1(1));
    r = [cos(angle) -sin(angle);sin(angle) cos(angle)];
    
    pt = (r*pt')';
    v2 = (r*v2')';
    
    if pt(1)>=v1(1) && pt(1)<=abs(v2(1))
        a = v1 - v2;
        b = pt - v2;
        a = [a,0];
        b = [b,0];

        d = norm(cross(a,b)) / norm(a);
    else
        d = max;
    end
end