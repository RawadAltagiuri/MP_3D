function [inside,insideXZ,insideYZ] = segmentInsideCylinder(startPt,endPt,obstacle)

radius = obstacle(4);
height= obstacle(5);
obstacleX = obstacle(2);
obstacleY = obstacle(3);
startPoint = [startPt(2) startPt(3)];
endPoint = [endPt(2) endPt(3)];

lowerVertex1 = [obstacleX-radius, obstacleY];
lowerVertex2 = [obstacleX+radius, obstacleY];
upperVertex1 = [obstacleX-radius, obstacleY-height];
upperVertex2 = [obstacleX+radius, obstacleY-height];

insideYZ = lineInsideRectangle(startPoint,endPoint,[upperVertex1; upperVertex2; lowerVertex2; lowerVertex1]);

obstacleX = obstacle(1);
obstacleY = obstacle(3);
startPoint = [startPt(1) startPt(3)];
endPoint = [endPt(1) endPt(3)];
lowerVertex1 = [obstacleX-radius, obstacleY];
lowerVertex2 = [obstacleX+radius, obstacleY];
upperVertex1 = [obstacleX-radius, obstacleY-height];
upperVertex2 = [obstacleX+radius, obstacleY-height];

insideXZ = lineInsideRectangle(startPoint,endPoint,[upperVertex1; upperVertex2; lowerVertex2; lowerVertex1]);

distance = pointToSegment3D([obstacle(1:2) 0],[startPt(1:2) 0],[endPt(1:2) 0]);
if distance > obstacle(4)
    insideXY = false;
else
    insideXY = true;
end
inside = insideXY && insideXZ && insideYZ;

end

function inside = lineInsideRectangle(startPoint,endPoint,rectangle)
    insideStart = false;
    insideEnd = false;
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
        insideStart = true;
    end
    
    distanceStartToUpper1X = abs(endPoint(1)-upperVertex1(1));
    distanceStartToUpper2X = abs(endPoint(1)-lowerVertex2(1));
    distanceUpper1Upper2X = abs(upperVertex1(1)-lowerVertex2(1));
    
    distanceStartToUpper1Y = abs(endPoint(2)-upperVertex1(2));
    distanceStartToLower2Y = abs(endPoint(2)-lowerVertex2(2));
    distanceUpper1Lower2Y = abs(upperVertex1(2)-lowerVertex2(2));

    if (distanceStartToUpper1X+distanceStartToUpper2X == distanceUpper1Upper2X) && ...
            (distanceStartToUpper1Y+distanceStartToLower2Y == distanceUpper1Lower2Y)
        insideEnd = true;
    end
    inside = insideStart && insideEnd;
end