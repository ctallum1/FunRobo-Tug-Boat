% Final Tug Boat Simulator

% Initialization
clc;  
clear; 
disp('RoboTug Simulation Running');

% Load up map of lake
map = loadMap('lake1.png');

% Set waypoints to drive to
waypoints = [-25 -25; 
    -15 -12; -25 0; -20 15; 0 18; 25 21; 40 16; 45 25; 46 6; 42 15;
    40 0; 37 -14; 45 -16; 45 -25; 7 -25; 7 -16; 0 -4;
    -10 2; 0 8; 23 5; 30 9; 23 17.5; 18 12; -2 -4;
    -25 -25]; % Grand Finale path
pointArray = createPath(map,waypoints);

% Plot planned path
figure('Name','Planned Path')
show(map), hold on
plot(pointArray(:,1),pointArray(:,2), 'o-')
hold off

% Create animation of robot driving through pointArray
animation = driveThru(pointArray, map);

% Functions 
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

function pointArray = createPath(map,waypoints)
pointArray = [];

% Generates a probabilistic roadmap with 1000 nodes
prm = robotics.PRM(map,1000);

% generates path section for every sequential pair of waypoints
for waypoint = 1:length(waypoints)-1
   start = waypoints(waypoint,:);
   goal = waypoints(waypoint+1,:);
   
   pathSection = findpath(prm, start, goal);
   pointArray(end+1:end+length(pathSection),:) = pathSection;
end

% removes waypoints to clean up path 
repeatIndex = [1];
while ~isempty(repeatIndex)
    [repeatIndex, pointArrayPosition] = intersect(pointArray,waypoints(2:end-1,:),'row');
    pointArray(pointArrayPosition,:)= [];
end

end

function myVideo = driveThru(pointArray, lakeMap)
% Set up video
myVideo = VideoWriter('myVideoFile');
myVideo.FrameRate = 20;
open(myVideo)

% create figure
lakeFigure = figure('Name','lakeMap');
show(lakeMap)
title('Grand Finale')

% Create differential drive robot
diffDriveTug = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");

% Create pure pursuit controller
controller = controllerPurePursuit('DesiredLinearVelocity',2,'MaxAngularVelocity',3);
controller.Waypoints = pointArray;

% Set up robot tug simulation

% Set the initial pose and final goal location based on the path. 
% Create global variables for storing the current pose and an 
initPose = [pointArray(1,1) pointArray(1,2), pi/2];
goal = [pointArray(end,1) pointArray(end,2)]';

% Set up simulation variables
sampleTime = 0.05;            % Sample time [s]
t = 0:sampleTime:1000;         % Time array
poses = zeros(3,numel(t));    % Pose matrix
poses(:,1) = initPose';

% Set iteration rate
r = rateControl(1/sampleTime);

% Get the axes from the figures
ax1 = lakeFigure.CurrentAxes;
disp('RoboTug Simulation set up');

for idx = 1:numel(t)
    position = poses(:,idx)';
    currPose = position(1:2);
    
    % End if pathfollowing is vehicle has reached goal waypoint 
    % position within tolerance of 0.2m
    distToGoal = norm(goal'-currPose);
    if (distToGoal < 1) && (idx > 100)
        disp("Goal position reached")
        break;       % end simulation at or near goal waypoint
    end
    
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(poses(:,idx));

    % Perform forward discrete integration step
    vel = derivative(diffDriveTug, poses(:,idx), [vRef wRef]);
    poses(:,idx+1) = poses(:,idx) + vel*sampleTime; 


    % Update visualization
    plotTrvec = [poses(1:2, idx+1); 0];
    plotRot = axang2quat([0 0 1 poses(3, idx+1)]);
    
    % Delete image of the last robot to prevent displaying multiple robots
    if idx > 1
       items = get(ax1, 'Children');
       delete(items(1)); 
    end

    %plot robot onto known map
    plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 2, 'Parent', ax1);
    
    % grab video frame
    frame = getframe(gcf);
    writeVideo(myVideo, frame);
    % waiting to iterate at the proper rate
    waitfor(r);
end
close(myVideo)
end
