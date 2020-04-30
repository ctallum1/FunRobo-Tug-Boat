% Final Tug Boat Simulator

% Initialization
clc;  
clear; 
disp('RoboTug Simulation Running');

% Load up map of lake

map = loadMap('lake1.png');
lakeFigure = figure('Name', 'lakeMap');
show(map); hold on

% Create Robot
createRobot()

% Set waypoints to drive to
waypoints = [-10 -10; 10,-10; 30 20];
path = createPath(map,waypoints);
plot(path(:,1),path(:,2), 'o-')

function lakeMap = loadMap(image)
% Creates binary ocupancy grid of map
img =  imread(image);
map = robotics.BinaryOccupancyGrid(~img,20);

% adjust the map size
map.GridLocationInWorld = [-30 -30];

% Inflate the map with the Robot Tug boat size, so robot can be treated as
% a point. Assume robot tug is 0.25m bow to stern
lakeMap = copy(map);
inflate(lakeMap, 0.25);
end

function createRobot()
% Create differential drive robot
diffDriveTug = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");

% Create pure persuit controller
controller = controllerPurePursuit('DesiredLinearVelocity',2,'MaxAngularVelocity',3);


end

function path = createPath(map,waypoints)
path = [];
for waypoint = 1:length(waypoints)-1
   start = waypoints(waypoint,:);
   goal = waypoints(waypoint+1,:);
   
   prm.NumNodes = 100;
   prm = robotics.PRM(map);
   pathSection = findpath(prm, start, goal);
   path(end+1:end+length(pathSection),:) = pathSection;
end
end