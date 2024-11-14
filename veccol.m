%startPoint is a point in 3d space with 3 indices, being X Y Z, it is
%the start point of the vector that ends with endPoint.
%endPoint is a point in 3d space with 3 indices, being X Y Z, it is the
%end point of the vector that starts with startPoint.
%obstacle is a 5 index array representing a cylinder, first being X position, second being Y
%position and the third being Z position. 4th index is the radius of the
%cylinder and 5th is the leng
%this function finds out if the vector collides with the cylinder obstacle or not.

function isColliding = veccol(startPoint, endPoint, obstacle)

    obstaclePos = obstacle(1:3);  %cylinder's position (center of base)
    radius = obstacle(4);         %cylinder's radius
    length = obstacle(5);         %cylinder's height

    cylinderTop = obstaclePos(3);             %top Z-coordinate of the cylinder
    cylinderBottom = cylinderTop - length;    %bottom Z-coordinate of the cylinder

    %plotter()

    if (startPoint(3) < cylinderBottom && endPoint(3) < cylinderBottom) || ...
       (startPoint(3) > cylinderTop && endPoint(3) > cylinderTop)
        isColliding = false;
        return;
    end

    lineVector = endPoint - startPoint;
    lineLength = norm(lineVector);

    if lineLength == 0
        isColliding = false;
        return;
    end

    direction = lineVector / lineLength;

    if startPoint(3) < cylinderBottom
        tStart = (cylinderBottom - startPoint(3)) / lineVector(3);
        trimmedStart = startPoint + tStart * lineVector;
    elseif startPoint(3) > cylinderTop
        tEnd = (cylinderTop - startPoint(3)) / lineVector(3);
        trimmedStart = startPoint + tEnd * lineVector;
    else
        trimmedStart = startPoint;
    end
    

    if endPoint(3) > cylinderTop
        tEnd = (cylinderTop - startPoint(3)) / lineVector(3);
        trimmedEnd = startPoint + tEnd * lineVector;
    elseif endPoint(3) < cylinderBottom
        tEnd = (cylinderBottom - startPoint(3)) / lineVector(3);
        trimmedEnd = startPoint + tEnd * lineVector;
    else
        trimmedEnd = endPoint;
    end

    d = obstaclePos - trimmedStart;

    projLength = dot(d, direction);

    trimmedLineVector = trimmedEnd - trimmedStart;
    trimmedLineLength = norm(trimmedLineVector);
    
    %if projection is lower than the limit, startPoint is the closest
    %if projection is bigger than the limit, the endPoint is the closest
    if projLength < 0
        closestPointOnLine = trimmedStart;
    elseif projLength > trimmedLineLength
        closestPointOnLine = trimmedEnd;
    else
        closestPointOnLine = trimmedStart + projLength * direction;
    end

    distanceToCylinderBase = norm(closestPointOnLine(1:2) - obstaclePos(1:2));

    isWithinHeight = closestPointOnLine(3) >= cylinderBottom && closestPointOnLine(3) <= cylinderTop;

    isColliding = distanceToCylinderBase <= radius && isWithinHeight;

end

function plotter()

    % startPoint = [71.0412, -6.09121, 100.871];
    % endPoint = [94.3988, -13.6421, 105.333];
    % obstacle = [85 0 200 12.5 300];

    %startPoint = [35.4352, -33.9399, 113.338];
    %endPoint = [47.9788, -33.4889, 113.528];
    %obstacle = [45 -45 200 12.5 300];

    startPoint = [26 0 14];
    endPoint = [14 0 9];
    obstacle = [20 0 10 5 5];
    
    obstaclePos = obstacle(1:3);
    radius = obstacle(4);
    height = obstacle(5);
    
    cylinderTop = obstaclePos(3);
    cylinderBottom = cylinderTop - height;

    figure;
    hold on;
    grid on;

    scatter3(startPoint(1), startPoint(2), startPoint(3), 100, 'green', 'filled');
    scatter3(endPoint(1), endPoint(2), endPoint(3), 100, 'red', 'filled');

    plot3([startPoint(1), endPoint(1)], [startPoint(2), endPoint(2)], [startPoint(3), endPoint(3)], 'b', 'LineWidth', 2);

    [X, Y, Z] = cylinder(radius);
    Z = Z * height + cylinderBottom;

    X = X + obstaclePos(1);
    Y = Y + obstaclePos(2);
    
    surf(X, Y, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Plot the cylinder with transparency

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Collision Check Visualization');
    axis equal;
    
    hold off;

end