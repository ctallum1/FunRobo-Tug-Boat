% finalProject2020_RevA.m
% This script contains the bare bones simulation set-up you can start with
% to help build your final robot boat project
% D. Barrett 4/12/20

clc;   % clear command window
clear; % clear Workspace
disp('RoboTug Simulation Running');

%% Load final project test lake

% Read a map of Robo Tug test lake (Binary Ocupancy Grid)
img =  imread('lake1.png');
mapTestLake = robotics.BinaryOccupancyGrid(~img,20);

% adjust the map size
map.GridLocationInWorld = [-30 -30];

% Inflate the map with the Robot Tug boat size, so robot can be treated as
% a point. Assume robot tug is 0.25m bow to stern
lakeMap = copy(mapTestLake);
inflate(lakeMap, 0.25);
lakeFigure = figure('Name', 'lakeMap'); % can comment out after running a few times
show(lakeMap);
disp('lakeMap built');

%% Create a robot-view map

% Create an empty map of the same dimensions as the selected map 
% with a resolution of 10. Show the figure and save the handle of the figure. 
% Lock the axes at the size of the map
robotMap = binaryOccupancyMap(82,60,5);
robotMapFigure = figure('Name','robotMap');
show(robotMap);
disp('robotMap built (yes it should start out blank)');

%% Create a differential drive (2 propellor) robot tug

% Create a differential-drive kinematic motion model. 
% The motion model represents the motion of the simulated 
% differential-drive robot. This model takes left and right wheels speeds
% or linear and angular velocities for the robot heading. 
% For this example, use the vehicle speed and heading rate for the VehicleInputs.
diffDriveTug = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");
disp('RoboTug built');

%% Create a pure pursuit robot tug controller

% Create a pure pursuit controller. This controller generates the velocity
% inputs for the simulated robot to follow a desired path. Set your 
% desired linear velocity and maximum angular velocity, specified in 
% meters per second and radians per second respectively.
controller = controllerPurePursuit('DesiredLinearVelocity',2,'MaxAngularVelocity',3);
disp('RoboTug controller built');

%% Create range LIDAR

% Create a sensor with a max range of 10 meters. This sensor simulates 
% range readings based on a given pose and map. The reference map is used
% with this range sensor to simulate collecting sensor readings in 
% an unknown environment.
% Its a little hard to fine documentation on rangeSensor fuction, so right
% click on it and choose "help" or f1
lidar = rangeSensor;    % creates a rangeSensor system object see help for info
lidar.Range = [0.5,40];   % sets minimum and maximum range of sensor
testPose = [40 40 pi/2];     % set an inital test pose for lidar

% Plot the test spot for lidar scan on the reference lakeMap figure.
figure(lakeFigure);
hold on
plot(40,40, 'r*');
hold off

% Generate a test scan.
    [ranges, angles] = lidar(testPose, lakeMap);
    scan = lidarScan(ranges, angles);
    
% Visualize the test lidar scan.
    figure('Name','lidarTestMap');
    plot(scan)
disp('RoboTug lidar built (should see live lidarTestMap)');

%% Create the Planned Path

% Create a path to drive through the map for gathering range sensor readings.
path = [10 10; 20 20; 30 20; 40 20; 40 30; 40 40; 30 50]; 

% Plot the path on the reference map figure.
figure(lakeFigure);
hold on
plot(path(:,1),path(:,2), 'o-');
hold off

% Set the path as the waypoints of the pure pursuit controller.
controller.Waypoints = path;
disp('RoboTug path planned');

%% Set up robot tug simulation

% Set the initial pose and final goal location based on the path. 
% Create global variables for storing the current pose and an 
% index for tracking the iterations.
initPose = [path(1,1) path(1,2), pi/2];
goal = [path(end,1) path(end,2)]';
poses(:,1) = initPose';

% Set up simulation variables
sampleTime = 0.05;            % Sample time [s]
t = 0:sampleTime:100;         % Time array
poses = zeros(3,numel(t));    % Pose matrix
poses(:,1) = initPose';

% Set iteration rate
r = rateControl(1/sampleTime);

% Get the axes from the figures
ax1 = lakeFigure.CurrentAxes;
ax2 = robotMapFigure.CurrentAxes;
disp('RoboTug Simulation set up');

%% Follow Path and Map Environment 
% Run robot on path created in last section
% 1.Scan the reference map using the range sensor and the current pose. 
% This simulates normal range readings for driving in an unknown environment.
% 2.Update the map with the range readings.
% 3.Get control commands from pure pursuit controller to drive to next waypoint.
% 4.Calculate derivative of robot motion based on control commands.
% 5.Increment the robot pose based on the derivative.

% Main simulation loop ***************************************************
    for idx = 1:numel(t)
        position = poses(:,idx)';
        currPose = position(1:2);
        
        % End if pathfollowing is vehicle has reached goal waypoint 
        % position within tolerance of 0.2m
        dist = norm(goal'-currPose);
        if (dist < .2)
            disp("Goal position reached")
            break;       % end simulation at or near goal waypoint
        end
        
        % Update map by taking sensor measurements
        figure(robotMapFigure)         
        [ranges, angles] = lidar(position, lakeMap);
        scan = lidarScan(ranges,angles);
        validScan = removeInvalidData(scan,'RangeLimits',[0,lidar.Range(2)]);
        insertRay(robotMap,position,validScan,lidar.Range(2));
        show(robotMap);
        
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
        %plot robot on new map
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 2, 'Parent', ax2);
    
        % waiting to iterate at the proper rate
        waitfor(r);
    end
    % Simulation loop end ************************************************
    beep
    pause(1);
    beep
    pause(1);
    beep
disp('RoboTug at end waypoint goal, Simulation 1 complete');
    
%% Ok lets get fancy and do some probalistic path planning
    
% Create a probabilistic roadmap of lake test enviroment
% Less Num.Nodes is faster, but less potential paths
% More Num.Nodes  is slower, but more paths
prm = robotics.PRM(lakeMap);
prm.NumNodes = 50;
figure(lakeFigure);
axlf = axes;
show(prm, 'Parent', axlf);

% Find a path around lake
start = [10 10];
goal = [70 10];
newPath = findpath(prm, start, goal);
hold('on');
show(prm, 'Map', 'off', 'Roadmap', 'off');
hold(axlf, 'off');
disp('RoboTug probablistic plan complete');

% Set the path as the waypoints of the pure pursuit controller.
controller.Waypoints = newPath;

%% Set up robot tug simulation (again)

% Set the initial pose and final goal location based on the path. 
% Create global variables for storing the current pose and an 
% index for tracking the iterations.
initPose = [newPath(1,1) newPath(1,2), pi/2];
goal = [newPath(end,1) newPath(end,2)]';
newPoses(:,1) = initPose';

% Set up simulation variables
sampleTime = 0.05;            % Sample time [s]
t = 0:sampleTime:100;         % Time array
newPoses = zeros(3,numel(t));    % Pose matrix
newPoses(:,1) = initPose';

% Set iteration rate
r = rateControl(1/sampleTime);

% Get the axes from the figures
ax1 = lakeFigure.CurrentAxes;
ax2 = robotMapFigure.CurrentAxes;
disp('RoboTug Simulation setup (again)');

%% Follow newPath and Map Environment  (again)
% Run robot on path created in last section
% 1.Scan the reference map using the range sensor and the current pose. 
% This simulates normal range readings for driving in an unknown environment.
% 2.Update the map with the range readings.
% 3.Get control commands from pure pursuit controller to drive to next waypoint.
% 4.Calculate derivative of robot motion based on control commands.
% 5.Increment the robot pose based on the derivative.

% Main simulation loop ***************************************************
    for idx = 1:numel(t)
        position = newPoses(:,idx)';
        currPose = position(1:2);
        
        % End if pathfollowing is vehicle has reached goal waypoint 
        % position within tolerance of 0.2m
        dist = norm(goal'-currPose);
        if (dist < .2)
            disp("Goal position reached")
            break;       % end simulation at or near goal waypoint
        end
        
        % Update map by taking sensor measurements
        figure(robotMapFigure)         
        [ranges, angles] = lidar(position, lakeMap);
        scan = lidarScan(ranges,angles);
        validScan = removeInvalidData(scan,'RangeLimits',[0,lidar.Range(2)]);
        insertRay(robotMap,position,validScan,lidar.Range(2));
        show(robotMap);
        
        % Run the Pure Pursuit controller and convert output to wheel speeds
        [vRef,wRef] = controller(newPoses(:,idx));
    
        % Perform forward discrete integration step
        vel = derivative(diffDriveTug, newPoses(:,idx), [vRef wRef]);
        newPoses(:,idx+1) = newPoses(:,idx) + vel*sampleTime; 
    
    
        % Update visualization
        plotTrvec = [newPoses(1:2, idx+1); 0];
        plotRot = axang2quat([0 0 1 newPoses(3, idx+1)]);
        
        % Delete image of the last robot to prevent displaying multiple robots
        if idx > 1
           items = get(ax1, 'Children');
           delete(items(1)); 
        end
    
        %plot robot onto known map
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 2, 'Parent', ax1);
        %plot robot on new map
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 2, 'Parent', ax2);
    
        % waiting to iterate at the proper rate
        waitfor(r);
    end
    % Simulation loop end ************************************************
    beep
    pause(1);
    beep
    pause(1);
    beep
disp('RoboTug at endwaypoint goal, Simulation 2 complete');
    
 %% Complete simulation and exit script
    
    beep
    pause(1);
    beep
    pause(1);
    beep
    disp('Final project script complete');
    






