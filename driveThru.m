function driveThru(pointArray, lakeMap, lakeFigure, robotMapFigure)

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

for idx = 1:numel(t)
    position = poses(:,idx)';
    currPose = position(1:2);
    
    % End if pathfollowing is vehicle has reached goal waypoint 
    % position within tolerance of 0.2m
    distToGoal = norm(goal'-currPose);
    if (distToGoal < .2)
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




end