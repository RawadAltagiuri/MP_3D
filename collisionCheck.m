function intersections = collisionCheck(conf)
    global op;
    drawProblem3D(conf);
    obstacles = op.obstacles;
    nObstacles = size(obstacles,1);

    nodes = solveForwardKinematics3D(conf,op.home_base,0);
    nUsedLinks = 0;
    for i = 1:size(conf,1)
        if conf(i,3)==0
            nUsedLinks = i-1;
            break;
        end
    end
    
    nUsedNodes = nUsedLinks + 1;

    intersections = 0;
    for i = 1 : nUsedNodes - 1 
        for j = 1 : nObstacles
            intersections = intersections + doTheyIntersect(nodes(i,:),nodes(i+1,:),obstacles(j,:));
        end
    end
    disp(intersections)
end

function intersect = doTheyIntersect(startPt,endPt,obstacle)
    intersectXY = planarIntersection(startPt,endPt,obstacle,"XY");
    if ~intersectXY
        intersect = false;
        return;
    end
    intersectYZ = planarIntersection(startPt,endPt,obstacle,"YZ");
    intersectXZ = planarIntersection(startPt,endPt,obstacle,"XZ");
    intersect = intersectYZ && intersectXZ && intersectXY;
end

function intersect  = planarIntersection(startPt,endPt,obstacle,plane)
    intersect = false;
    
    if plane=="YZ"
        obstacleX = obstacle(2);
        obstacleY = obstacle(3);
        startPoint = [startPt(2) startPt(3)];
        endPoint = [endPt(2) endPt(3)];
    elseif plane=="XZ"
        obstacleX = obstacle(1);
        obstacleY = obstacle(3);
        startPoint = [startPt(1) startPt(3)];
        endPoint = [endPt(1) endPt(3)];
    elseif plane=="XY"
        obstacleX = obstacle(1);
        obstacleY = obstacle(2);
        startPoint = [startPt(1) startPt(2)];
        endPoint = [endPt(1) endPt(2)];
        radius = obstacle(4);
        segment = [startPoint; endPoint];
        circle = [obstacleX, obstacleY, radius];
        intersect = segmentIntersectsCircle(segment,circle);
        return;
    end
    radius = obstacle(4);
    height= obstacle(5);
    lowerVertex1 = [obstacleX-radius, obstacleY];
    lowerVertex2 = [obstacleX+radius, obstacleY]; 
    upperVertex1 = [obstacleX-radius, obstacleY-height];
    upperVertex2 = [obstacleX+radius, obstacleY-height];

    inside = lineInsideRectangle(startPoint,endPoint,[upperVertex1; upperVertex2; lowerVertex2; lowerVertex1]);

    if inside
        intersect = true;
        return;
    end
    
    [x1out, ~] = polyxpoly([startPoint(1), endPoint(1)],[startPoint(2),endPoint(2)],...
        [lowerVertex1(1), upperVertex1(1)],[lowerVertex1(2), upperVertex1(2)]);
    [x2out, ~] = polyxpoly([startPoint(1), endPoint(1)],[startPoint(2),endPoint(2)],...
        [lowerVertex2(1), upperVertex2(1)],[lowerVertex2(2), upperVertex2(2)]);
    [x3out, ~] = polyxpoly([startPoint(1), endPoint(1)],[startPoint(2),endPoint(2)],...
        [upperVertex1(1), upperVertex2(1)],[upperVertex1(2), upperVertex2(2)]);
    [x4out, ~] = polyxpoly([startPoint(1), endPoint(1)],[startPoint(2),endPoint(2)],...
        [lowerVertex1(1), lowerVertex2(1)],[lowerVertex1(2), lowerVertex2(2)]);

    if ~isempty(x1out) || ~isempty(x2out) || ~isempty(x3out) || ~isempty(x4out)
        intersect = true;
    end
    
end

function intersects = segmentIntersectsCircle(segment,circle)
    point1 = segment(1,:);
    point2 = segment(2,:);
    center = circle(1:2);
    centerToSegmentDistance = pointToSegment3D([center 0],[point1 0],[point2 0]);

    radius = circle(3);
    
    if centerToSegmentDistance > radius
        intersects = false;
    else
        intersects = true;
    end
end

function inside = lineInsideRectangle(startPoint,endPoint,rectangle)
    inside = false;
    upperVertex1 = rectangle(1,:);
    lowerVertex2 = rectangle(3,:);


    distanceStartToUpper1X = abs(startPoint(1)-upperVertex1(1));
    distanceStartToUpper2X = abs(startPoint(1)-lowerVertex2(1));
    distanceUpper1Upper2X = abs(upperVertex1(1)-lowerVertex2(1));
    
    distanceStartToUpper1Y = abs(startPoint(2)-upperVertex1(2));
    distanceStartToLower2Y = abs(startPoint(2)-lowerVertex2(2));
    distanceUpper1Lower2Y = abs(upperVertex1(2)-lowerVertex2(2));

    if (distanceStartToUpper1X+distanceStartToUpper2X == distanceUpper1Upper2X) && ...
            (distanceStartToUpper1Y+distanceStartToLower2Y == distanceUpper1Lower2Y)
        inside = true;
        return;
    end
    
    distanceStartToUpper1X = abs(endPoint(1)-upperVertex1(1));
    distanceStartToUpper2X = abs(endPoint(1)-lowerVertex2(1));
    distanceUpper1Upper2X = abs(upperVertex1(1)-lowerVertex2(1));
    
    distanceStartToUpper1Y = abs(endPoint(2)-upperVertex1(2));
    distanceStartToLower2Y = abs(endPoint(2)-lowerVertex2(2));
    distanceUpper1Lower2Y = abs(upperVertex1(2)-lowerVertex2(2));

    if (distanceStartToUpper1X+distanceStartToUpper2X == distanceUpper1Upper2X) && ...
            (distanceStartToUpper1Y+distanceStartToLower2Y == distanceUpper1Lower2Y)
        inside = true;
        return;
    end
end