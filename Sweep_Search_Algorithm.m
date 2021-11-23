
function [drone, poses] = Sweep_Search_Algorithm(drone, maxMoves)
drone.MaxMoves = maxMoves;
map = drone.TrueMap;
sensingDistance = 3;
mapWidth = size(drone.TrueMap, 2);
mapHeight = size(drone.TrueMap, 1); 
navigating = true;

targetX = [15];
targetY = [5];

xMove = mapWidth - 30;
yMove = sensingDistance*2;

direction = 'r';

poses = [15,5];
hold on

while navigating == true
    
    if direction == 'r'
        targetX = targetX + xMove;
        direction = 'l';
    else
        targetX = targetX - xMove;
        direction = 'r';
    end     
    
    while ~drone.isNear(targetX,targetY)
        drone = drone.navigateToPoint(targetX,targetY);
        drone.plot();
        pause(0.02);
        poses(end+1,:) = [drone.X, size(map,1) - drone.Y];
    end
    
    targetY = targetY + yMove; 
    while ~drone.isNear(targetX,targetY)
        drone = drone.navigateToPoint(targetX,targetY);
        drone.plot();
        pause(0.02);
        poses(end+1,:) = [drone.X, size(map,1) - drone.Y];
    end
    
    if targetY >= (mapHeight - yMove)
        navigating = false;
       
    end
    
end
plot(poses(:,1),poses(:,2),"blue")
end