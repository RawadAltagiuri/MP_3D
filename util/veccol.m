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

    if (startPoint(3) < cylinderBottom && endPoint(3) < cylinderBottom) || ...
       (startPoint(3) > cylinderTop && endPoint(3) > cylinderTop)
        isColliding = false;
        return;
    end

    lineVector = endPoint - startPoint;
    trimmedStart = startPoint;
    trimmedEnd = endPoint;

    lineLength = norm(lineVector);

    if startPoint(3) < cylinderBottom
        t = (cylinderBottom - startPoint(3)) / lineVector(3);
        trimmedStart = startPoint + t * lineVector;
    elseif startPoint(3) > cylinderTop
        t = (cylinderTop - startPoint(3)) / lineVector(3);
        trimmedStart = startPoint + t * lineVector;
    end

    if endPoint(3) < cylinderBottom
        t = (cylinderBottom - startPoint(3)) / lineVector(3);
        trimmedEnd = startPoint + t * lineVector;
    elseif endPoint(3) > cylinderTop
        t = (cylinderTop - startPoint(3)) / lineVector(3);
        trimmedEnd = startPoint + t * lineVector;
    end

    direction = trimmedEnd - trimmedStart;
    % segmentLength = norm(direction);
    % if segmentLength == 0
    %     closestPoint = trimmedStart;
    % else
    %     direction = direction / segmentLength;
    % 
    %     d = obstaclePos - trimmedStart;
    %     projLength = dot(d, direction);
    % 
    %     if projLength < 0
    %         closestPoint = trimmedStart;
    %     elseif projLength > segmentLength
    %         closestPoint = trimmedEnd;
    %     else
    %         closestPoint = trimmedStart + projLength * direction;
    %     end
    % end
    %if projection is lower than the limit, startPoint is the closest
    %if projection is bigger than the limit, the endPoint is the closest

    %distanceToCylinderBase = distanceLinePoint(startPoint(1:2), endPoint(1:2), obstacle(1:2));

    distanceToCylinderBase = distanceLinePoint(trimmedStart(1:2), trimmedEnd(1:2), obstaclePos(1:2));

    isColliding = distanceToCylinderBase <= radius;
    
    %plotter(startPoint, endPoint, obstacle);
end

function distance = distanceLinePoint(startPoint, endPoint, comparisonPoint)
    numerator = abs((endPoint(1) - startPoint(1)) * (startPoint(2) - comparisonPoint(2)) - (startPoint(1) - comparisonPoint(1)) * (endPoint(2) - startPoint(2)));
	denominator = sqrt((endPoint(1) - startPoint(1)) ^ 2 + (endPoint(2) - startPoint(2)) ^ 2);
	distance = numerator ./ denominator;
end

function plotter(startPoint, endPoint, obstacle)

    % startPoint = [71.0412, -6.09121, 100.871];
    % endPoint = [94.3988, -13.6421, 105.333];
    % obstacle = [85 0 200 12.5 300];

    %startPoint = [35.4352, -33.9399, 113.338];
    %endPoint = [47.9788, -33.4889, 113.528];
    %obstacle = [45 -45 200 12.5 300];
    
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

    %scatter3(closestPoint(1), closestPoint(2), closestPoint(3), 100, 'blue');

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