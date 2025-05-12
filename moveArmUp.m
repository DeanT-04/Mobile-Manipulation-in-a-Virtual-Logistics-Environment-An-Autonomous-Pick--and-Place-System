% moveArmUp.m
% Script to move the UR3 arm upward, pause for 2 seconds, reverse the Pioneer robot for 2 seconds, and stop

clc;
disp('Move Arm Up started');

%% Access global variables
global clientIDGlobal simGlobal dhParamsGlobal jointHandlesGlobal
global orangeAvgX orangeAvgY bottleAvgX bottleAvgY cupAvgX cupAvgY objectToPick

% Check if global variables are properly set
if isempty(clientIDGlobal) || isempty(simGlobal) || isempty(dhParamsGlobal) || isempty(jointHandlesGlobal)
    disp('Error: Global variables not set. Ensure main script ran correctly.');
    return;
end

% Assign variables from globals
clientID = clientIDGlobal;
sim = simGlobal;
d = dhParamsGlobal.d;
a = dhParamsGlobal.a;
jh = jointHandlesGlobal;

% Check if objectToPick is defined
if isempty(objectToPick)
    disp('Error: objectToPick not defined. Defaulting to bottle.');
    objectToPick = 2; % Default to bottle if undefined
end

%% Define the "Up" Waypoint
% Move to z = 0.5m, keeping x, y from the grasped object's waypoint, same orientation
upWaypoint = [0, 0, 0.7, 180, 0, 0]; % [x, y, z, roll, pitch, yaw] in meters and degrees

% Set x, y coordinates based on the object picked
switch objectToPick
    case 1 % Orange
        upWaypoint(1) = orangeAvgX;
        upWaypoint(2) = -orangeAvgY; % Match main script's coordinate convention
        objectName = 'Orange';
    case 2 % Bottle
        upWaypoint(1) = -bottleAvgX; % Match main script's coordinate convention
        upWaypoint(2) = -bottleAvgY;
        objectName = 'Bottle';
    case 3 % Cup
        upWaypoint(1) = cupAvgX;
        upWaypoint(2) = -cupAvgY; % Match main script's coordinate convention
        objectName = 'Cup';
    otherwise
        disp('Error: Invalid objectToPick. Using default x, y = 0.');
        upWaypoint(1) = 0;
        upWaypoint(2) = 0;
        objectName = 'Unknown';
end

fprintf('Moving %s up, keeping x=%.3f, y=%.3f\n', objectName, upWaypoint(1), upWaypoint(2));

%% Retrieve Current Joint Positions
dof = 6;
currentJoints = zeros(1, dof);
for i = 1:dof
    [~, currentJoints(i)] = sim.simxGetJointPosition(clientID, jh(i), sim.simx_opmode_blocking);
end
disp('Current Joint Positions (radians):');
disp(currentJoints);

%% Compute IK for Up Waypoint
% Convert orientation to rotation matrix
R_target = rpy2rotm(deg2rad(upWaypoint(4:6)));
% Create 4x4 homogeneous transformation matrix
T_target = [R_target, upWaypoint(1:3)'; 0 0 0 1];

% Compute inverse kinematics solution
joints = invKin8sol(d, a, T_target);
targetJoints = joints(4, :); % Select the 4th solution (consistent with main script)

fprintf('Up Waypoint target joints (radians):\n');
disp(targetJoints);

%% Plan and Execute Trajectory to Up Waypoint
% Define trajectory parameters
T_seg = 5;                  % Duration in seconds
timeStep = 0.05;            % Time step
timeVec = 0:timeStep:T_seg; % Time vector

% Preallocate trajectory matrix
trajSegment = zeros(length(timeVec), dof);

% Generate quintic trajectory for each joint
for j = 1:dof
    trajSegment(:, j) = quinticTrajectory(timeVec, T_seg, ...
        currentJoints(j), targetJoints(j), 0, 0, 0, 0);
end

% Execute the trajectory
disp('Executing move up trajectory');
for idx = 1:length(timeVec)
    for j = 1:dof
        sim.simxSetJointTargetPosition(clientID, jh(j), trajSegment(idx, j), sim.simx_opmode_oneshot);
    end
    pause(timeStep);
end

%% Pause for 2 Seconds
disp('Pausing for 2 seconds');
pause(2);

%% Reverse Pioneer Robot and Stop
disp('Reversing Pioneer robot for 2 seconds');
% Get motor handles
[resL, leftMotorHandle] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
[resR, rightMotorHandle] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
if resL ~= sim.simx_return_ok || resR ~= sim.simx_return_ok
    disp('Error: Unable to get motor handles. Skipping reverse motion.');
else
    % Set negative velocities to move backward
    reverseVelocity = -1; % rad/s
    reverseDuration = 6; % seconds
    sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, reverseVelocity, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, reverseVelocity, sim.simx_opmode_oneshot);
    pause(reverseDuration);
    % Explicitly stop the motors
    sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_blocking);
    disp('Pioneer robot stopped');
end

%% Disconnect from CoppeliaSim
disp('Move Arm Up and Reverse completed');
sim.simxFinish(clientID);
sim.delete();
disp('Disconnected from CoppeliaSim');

%% Helper Functions
function theta = quinticTrajectory(t, T, theta0, thetaf, vel0, velf, acc0, accf)
    A = [1, 0, 0, 0, 0, 0;
         1, T, T^2, T^3, T^4, T^5;
         0, 1, 0, 0, 0, 0;
         0, 1, 2*T, 3*T^2, 4*T^3, 5*T^4;
         0, 0, 2, 0, 0, 0;
         0, 0, 2, 6*T, 12*T^2, 20*T^3];
    b = [theta0; thetaf; vel0; velf; acc0; accf];
    coeffs = A\b;
    theta = coeffs(1) + coeffs(2)*t + coeffs(3)*t.^2 + coeffs(4)*t.^3 + coeffs(5)*t.^4 + coeffs(6)*t.^5;
end

function R = rpy2rotm(rpy)
    roll = rpy(1); pitch = rpy(2); yaw = rpy(3);
    R_x = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
    R_y = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
    R_z = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    R = R_z * R_y * R_x;
end