%%%% Using RRT for planning the optimize path. %%%%%%%%
clear
clc

%loading the seleted map in png format
image = imread('MapTest2.PNG');
[rows, columns, numberOfColorChannels] = size(image);
if numberOfColorChannels > 1
    % It's a true color RGB image.  We need to convert to gray scale.
    grayimage = rgb2gray(image);
else
    % It's already gray scale.  No need to convert.
    grayimage = image;
end

grayimage = grayimage < 0.5;
grid = binaryOccupancyMap(grayimage);

show(grid)

% Set the start and goal poses
start = [39.5, 237.5, 0];
goal = [437.5, 56.5, 0];

%define state space
bounds = [grid.XWorldLimits; grid.YWorldLimits; [-pi pi]];

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

%%%%%%%%%%%%%Path Planner%%%%%%%%%%%%%%%%

%validation to avoid collision with map obstacle
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = grid;
stateValidator.ValidationDistance = 0.05;

%create the pathplanner & connect more states,set the #ofiterations for
planner = plannerRRT(ss, stateValidator);
planner.MaxConnectionDistance = 150000000;
planner.MaxIterations = 150000000;

planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

% plan the path between qstart and qfinal
rng(100,'twister') %for repeatability (control random # generator)
[pthObj, solnInfo] = plan(planner, start, goal);



%%%%%%%%%% plotting of path%%%%%%%%%%
show(grid)
hold on

% Search tree
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');
plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)



function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.1;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end

