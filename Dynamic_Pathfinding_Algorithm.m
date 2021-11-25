function [drone, poses] = Dynamic_Pathfinding_Algorithm(drone, maxMoves)
    drone.MaxMoves = maxMoves;
    map = drone.TrueMap;
    speedCoeff = 0.6;
    poses = [drone.X, size(map,1)- drone.Y];
    startX = drone.X;
    startY = drone.Y;
    while drone.canMove()
        boundary = drone.getBoundary();
        target = closestUnexploredWithVelocity(boundary, drone.X, drone.Y, drone.DeltaX, drone.DeltaY, startX, startY);
    %     target = closestUnexplored(boundary, drone.X, drone.Y);
        targetX = target(1);
        targetY = target(2);
        if size(poses,1) > 20
           speedCoeff = 0.7; 
        end
%         count = 0;
        while ~drone.isNear(targetX,targetY)
            drone = drone.navigateWithSpeed(targetX,targetY, speedCoeff);
            drone.plot();
            pause(0.02);
            poses(end+1,:) = [drone.X, size(map,1)- drone.Y];
        end
    end
    plot(poses(:,1),poses(:,2),"blue")
end

function boundary = getBoundary(robotMap, robotX, robotY)
    pixel = robotY;
    visible = robotMap(robotY, robotX);
    for i = robotY:robotY+100
        if robotMap(i, robotX) ~= visible
            pixel = i;
            break;
        end
    end
    boundary = bwtraceboundary(robotMap, [pixel, robotX], 'N');
end

function closest = closestUnexplored(boundary, robotX, robotY)
    closest = [0, 0];
    closestDistance = 100000;
    for i = 1:size(boundary, 1)
        newDist = distance([robotY, robotX], boundary(i,:));
        if newDist < closestDistance
            closestDistance = newDist;
            closest = [boundary(i,2), boundary(i,1)];
        end
    end
end

function boundaryDist = boundaryWithDistance(boundary, robotX, robotY)
    boundaryDist = boundary;
    size(boundary, 1)
    for i = 1:size(boundary, 1)
        boundaryDist(i,3) = distance([robotY, robotX], boundary(i,:));
    end
end

function closest = closestUnexploredWithVelocity(boundary, robotX, robotY, deltaX, deltaY, startX, startY)
%     disp("find closest with vel")
    closest = [0, 0];
    closestDistance = 1000000;
    for i = 1:size(boundary, 1)
        deltaRobot = [deltaX, deltaY];
        if norm(deltaRobot) ~=0
            normRobot = deltaRobot / norm(deltaRobot);
        else
            normRobot = [0, 0];
        end
        
        pointDelta = [boundary(i,2)-robotX, boundary(i,1)-robotY];
        normPoint = pointDelta / norm(pointDelta);
        
        dist = distance([robotY, robotX], boundary(i,:));

        startDist = distance(boundary(i,:), [startY startX]);
        
        newDist = distanceVelocityHeuristic(dist, normRobot, normPoint, startDist);
        
        if newDist < closestDistance
            closestDistance = newDist;
            closest = [boundary(i,2), boundary(i,1)];
        end
    end
end

function heuristic = distanceVelocityHeuristic(dist, normRobot, normPoint, startDist)
    velocityCoeff = 2;
%     normRobot = normRobot;
%     normPoint = normPoint;
    diffX = abs(normRobot(1) - normPoint(1));
    diffY = abs(normRobot(2) - normPoint(2));
    heuristic = dist + 0.75 * startDist + diffX * velocityCoeff + diffY * velocityCoeff;
end

function dist = distance(a, b)
    dist = sqrt((a(1) - b(1)) ^ 2 + (a(2) - b(2)) ^ 2);
end