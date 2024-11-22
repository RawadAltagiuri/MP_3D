function intersect = segmentxcylinder(startPt,endPt,obstacle)

    [inside, insideXZ,insideYZ] = segmentInsideCylinder(startPt,endPt,obstacle);
    if inside
        intersect = true;
        return;
    end
    intersectXY = planarIntersection(startPt,endPt,obstacle,"XY");
    
    if ~intersectXY
        intersect = false;
        return;
    end
    intersectYZ = planarIntersection(startPt,endPt,obstacle,"YZ");

    if insideYZ 
    elseif ~intersectYZ
        intersect = false;
        return;
    end
    intersectXZ = planarIntersection(startPt,endPt,obstacle,"XZ");

    if insideXZ 
    elseif ~intersectXZ
        intersect = false;
        return;
    end
    intersect = true;
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

