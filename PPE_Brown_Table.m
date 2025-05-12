%% Initialize CoppeliaSim Remote API
sim = remApi('remoteApi');
sim.simxFinish(-1); % Close previous connections
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5); % Start connection

if clientID > -1
    disp('Connected to CoppeliaSim for path execution.');
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    
    % Get motor handles
    [resL, leftMotorHandle] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
    [resR, rightMotorHandle] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
    [resP, pioneerHandle] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx',sim.simx_opmode_blocking);
    
    if resL ~= sim.simx_return_ok || resR ~= sim.simx_return_ok || resP ~= sim.simx_return_ok
        disp('Error: Unable to get robot or motor handles. Exiting.');
        sim.simxFinish(clientID);
        return;
    end
    
    %% Get current pose of the robot
    [~, robotPosition] = sim.simxGetObjectPosition(clientID, pioneerHandle, -1, sim.simx_opmode_streaming);
    [~, robotOrientation] = sim.simxGetObjectOrientation(clientID, pioneerHandle, -1, sim.simx_opmode_streaming);
    pause(0.5); % Allow streaming to fetch the latest values
    
    [~, robotPosition] = sim.simxGetObjectPosition(clientID, pioneerHandle, -1, sim.simx_opmode_buffer);
    [~, robotOrientation] = sim.simxGetObjectOrientation(clientID, pioneerHandle, -1, sim.simx_opmode_buffer);
    
    % Convert to world coordinates
    robotCurrentPose = double([robotPosition(1), robotPosition(2), robotOrientation(3)]); % [x, y, theta]
    
    % Load SLAM map
    load('slamMap.mat', 'map');
    
    % Get the object name from the global variable
    global objectNameForBrownTable objectToPick;
    
    % Define goal locations for each object
    goalLocations = containers.Map;
    goalLocations('Orange') = [0, -1.5, pi/2];    % Center
    goalLocations('Bottle') = [0.15, -1.5, pi/2];  % Right
    goalLocations('Cup') = [-0.15, -1.5, pi/2];    % Left
    
    % Get the appropriate goal location
    if isKey(goalLocations, objectNameForBrownTable)
        goalLocation = goalLocations(objectNameForBrownTable);
        fprintf('\nMoving to brown table location for %s\n', objectNameForBrownTable);
        
        % Set objectToPick based on name (1=orange, 2=bottle, 3=cup)
        switch objectNameForBrownTable
            case 'Orange'
                objectToPick = 1;
            case 'Bottle'
                objectToPick = 2;
            case 'Cup'
                objectToPick = 3;
        end
    else
        goalLocation = [0, -1.5, pi/2]; % Default location
        disp('Unknown object, using default brown table location');
    end
    
    % Validate goal location
    goalLocationGrid = world2grid(map, goalLocation(1:2));
    if checkOccupancy(map, goalLocationGrid)
        disp('Goal location is invalid. Adjusting...');
        goalLocation(1:2) = findFreeLocation(map, goalLocation(1:2), 0.5);
        goalLocationGrid = world2grid(map, goalLocation(1:2));
        disp(['New Goal Location: ', mat2str(goalLocation(1:2))]);
    end
    
    %% Path Planning
    path = pathPlanner(robotCurrentPose, goalLocation, map);
    
    if isempty(path)
        disp('No valid path found. The robot will not move.');
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
        sim.simxFinish(clientID);
        return;
    end
    
    %% Path Execution
    executePathPID_BROWN(clientID, sim, path, pioneerHandle, leftMotorHandle, rightMotorHandle);
    disp('Path execution completed.');
    
    % Ensure the robot is fully stopped before swiveling
    sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_oneshot);
    pause(0.5); % Brief pause to let the robot settle
    
    %% Swivel robot to -90 degrees for brown table
    swivelRobot(clientID, sim, pioneerHandle, leftMotorHandle, rightMotorHandle, 'brown');
    pause(1); % Brief pause to observe the swivel
    
    %% Drop Off Procedure
    disp('Preparing to drop off object...');
    pause(2); % Wait 2 seconds before dropping
    
    % Set global variables needed by dropOffAtBrownTable
    global objectName;
    objectName = objectNameForBrownTable;
    
    % Call the drop off function (modified to use existing connection)
    dropOffAtBrownTable(clientID, sim); % Pass existing connection
    
    % Keep simulation running
    disp('Disconnected from CoppeliaSim.');
else
    disp('Failed to connect to CoppeliaSim.');
end

%% Helper Function: Find Free Location
function newLocation = findFreeLocation(map, location, stepSize)
theta = 0:pi/8:2*pi;
for r = stepSize:stepSize:2
    x = location(1) + r * cos(theta);
    y = location(2) + r * sin(theta);
    candidates = [x', y'];
    for i = 1:size(candidates, 1)
        if ~checkOccupancy(map, world2grid(map, candidates(i, :)))
            newLocation = candidates(i, :);
            return;
        end
    end
end
disp('Warning: No free location found. Using the original location.');
newLocation = location;
end