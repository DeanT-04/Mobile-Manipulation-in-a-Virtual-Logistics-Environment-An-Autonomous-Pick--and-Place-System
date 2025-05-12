% Copy_of_Cartesian_grasping_script.m
% This script performs Cartesian grasping of a selected object using a UR3 robot
% in CoppeliaSim. It computes inverse kinematics for waypoints and executes
% the trajectory.

clc;
fprintf('[INFO] Cartesian Grasping Program started.\n');

%% ======================================================================= %%
%  GLOBAL VARIABLE DECLARATIONS                                             %
%% ======================================================================= %%
global orangeAvgX orangeAvgY; % Average X, Y coordinates for the orange
global bottleAvgX bottleAvgY; % Average X, Y coordinates for the bottle
global cupAvgX cupAvgY;       % Average X, Y coordinates for the cup
global objectToPick;          % Index of the object to pick (1: orange, 2: bottle, 3: cup)

%% ======================================================================= %%
%  INITIALIZATION                                                           %
%% ======================================================================= %%
fprintf('[INFO] Initializing parameters and connecting to CoppeliaSim...\n');

% Robot parameters
degreesOfFreedom = 6;                 % Degrees of freedom for UR3
linkOffsets_d = zeros(1, degreesOfFreedom + 1); % Link offsets (DH parameter d_i)
linkLengths_a = zeros(1, degreesOfFreedom);   % Link lengths (DH parameter a_i)

% --- Connect to CoppeliaSim ---
sim = remApi('remoteApi');      % Load the remote API
sim.simxFinish(-1);             % Just in case, close all LUA open connections
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if clientID <= -1
    fprintf('[ERROR] Failed to connect to CoppeliaSim. Please ensure CoppeliaSim is running.\n');
    return; % Exit script if connection failed
end
fprintf('[INFO] Successfully connected to CoppeliaSim (ClientID: %d).\n', clientID);

% --- Initialize Gripper ---
graspRG2(clientID, sim, 'open'); % Open the gripper initially

% --- Define DH Parameters for UR3 ---
% These values should be specific to your UR3 robot configuration.
% d_i: offset along previous z to the common normal
% a_i: length of the common normal (distance between previous z and current z along current x)
linkOffsets_d(1) = 0.10887;     % d1: Base to Joint 1
linkOffsets_d(2) = 0.11154;     % d2: Joint 1 to Joint 2 (along z0, but effectively d2 for std DH)
linkOffsets_d(3) = 0;           % d3
linkOffsets_d(4) = 0;           % d4
linkOffsets_d(5) = 0.11223 - 0.11154; % d5: (UR robots often have d values here)
linkOffsets_d(6) = 0.194 - 0.11223;   % d6
linkOffsets_d(7) = 0;           % d_EE: End-effector offset (if any, from tool flange)

linkLengths_a(1) = 0;           % a1 (Typically 0 for first joint if z0 aligns with z1 rotation axis)
linkLengths_a(2) = 0.35252 - 0.10887; % a2: Joint 2 to Joint 3
linkLengths_a(3) = 0.56577 - 0.35252; % a3: Joint 3 to Joint 4
linkLengths_a(4) = 0.64999 - 0.56577; % a4
linkLengths_a(5) = 0.65111 - 0.64999; % a5
linkLengths_a(6) = 0;           % a6 (Typically 0 for wrist joints)

jointHandleNamePrefix = 'UR3_joint';

%% ======================================================================= %%
%  OBJECT SELECTION AND WAYPOINT DEFINITION                                 %
%% ======================================================================= %%
fprintf('[INFO] Selecting object and defining waypoints...\n');

% --- Define which object to pick ---
% 1 = orange, 2 = bottle, 3 = cup
if ~exist('objectToPick', 'var') || isempty(objectToPick)
    fprintf('[WARN] objectToPick not set. Defaulting to bottle (2).\n');
    objectToPick = 1; % Default to bottle if not specified
end

% --- Define waypoints for each object ---
% Format: [x, y, z, roll, pitch, yaw] (position in meters, angles in degrees)
% Note: X and Y are dynamic and will be updated by global average positions.
% The Z, roll, pitch, yaw are pre-grasp approach poses.
orangeWaypoint = [NaN, NaN, 0.34, 180, 0, 90];
bottleWaypoint = [NaN, NaN, 0.38, 0, 180, 0];
cupWaypoint    = [NaN, NaN, 0.345, 180, -10, 0];

% --- Update waypoints with global position variables from object detection ---
% Ensure global variables are populated (e.g., by an object detection script)
% The negative signs for AvgX and AvgY might be due to coordinate system differences
% between the detection frame and the robot's base frame.
orangeWaypoint(1) = -orangeAvgX;
orangeWaypoint(2) = -orangeAvgY;

bottleWaypoint(1) = -bottleAvgX;
bottleWaypoint(2) = -bottleAvgY;

cupWaypoint(1) = -cupAvgX;
cupWaypoint(2) = -cupAvgY;

% --- Select active waypoint based on objectToPick ---
activeWaypoint = [];
objectName = 'Unknown';

switch objectToPick
    case 1 % Orange
        activeWaypoint = orangeWaypoint;
        objectName = 'Orange';
    case 2 % Bottle
        activeWaypoint = bottleWaypoint;
        objectName = 'Bottle';
    case 3 % Cup
        activeWaypoint = cupWaypoint;
        objectName = 'Cup';
    otherwise
        fprintf('[WARN] Invalid object selection. Using default placeholder waypoint.\n');
        activeWaypoint = [0.013, -0.237, 0.34, 180, 0, 0]; % Default fallback
        objectName = 'Unknown (Default)';
end

% Check for NaN values in active waypoint (if AvgX/AvgY were NaN)
if any(isnan(activeWaypoint(1:2)))
    fprintf('[ERROR] Target object "%s" has NaN coordinates. Grasping cannot proceed with this waypoint.\n', objectName);
    fprintf('[ERROR] Please ensure object detection provides valid coordinates.\n');
    % Clean up CoppeliaSim connection
    sim.simxFinish(clientID);
    sim.delete();
    fprintf('[INFO] Disconnected from CoppeliaSim due to waypoint error.\n');
    return;
end

%% ======================================================================= %%
%  CARTESIAN WAYPOINTS FOR GRASPING TASK                                    %
%% ======================================================================= %%
% For this script, I'm using a single active waypoint for the grasp.
% More complex tasks might involve multiple waypoints (e.g., pre-grasp, grasp, lift).
targetWaypoints = [activeWaypoint]; % Using the selected waypoint
numWaypoints = size(targetWaypoints, 1);

fprintf('[INFO] Target Object: %s\n', objectName);
fprintf('[INFO] Target Waypoint (Pose): [X:%.3fm, Y:%.3fm, Z:%.3fm, R:%.1fdeg, P:%.1fdeg, Y:%.1fdeg]\n', ...
        activeWaypoint(1), activeWaypoint(2), activeWaypoint(3), ...
        activeWaypoint(4), activeWaypoint(5), activeWaypoint(6));

%% ======================================================================= %%
%  ROBOT ARM CONTROL                                                        %
%% ======================================================================= %%

% --- Retrieve Joint Handles ---
fprintf('[INFO] Retrieving joint handles...\n');
jointHandles = zeros(1, degreesOfFreedom);
for jointIdx = 1:degreesOfFreedom
    jointName = strcat(jointHandleNamePrefix, int2str(jointIdx));
    [returnCode, jointHandles(jointIdx)] = sim.simxGetObjectHandle(clientID, jointName, sim.simx_opmode_blocking);
    if returnCode ~= sim.simx_return_ok
        fprintf('[ERROR] Failed to get handle for joint %s. Return code: %d\n', jointName, returnCode);
        sim.simxFinish(clientID);
        sim.delete();
        fprintf('[INFO] Disconnected from CoppeliaSim due to handle error.\n');
        return;
    end
end
fprintf('[INFO] All joint handles retrieved successfully.\n');

% --- Retrieve Current Joint Positions ---
fprintf('[INFO] Retrieving current joint positions...\n');
currentJointAngles = zeros(1, degreesOfFreedom);
for jointIdx = 1:degreesOfFreedom
    [returnCode, currentJointAngles(jointIdx)] = sim.simxGetJointPosition(clientID, jointHandles(jointIdx), sim.simx_opmode_blocking);
    if returnCode ~= sim.simx_return_ok
        fprintf('[ERROR] Failed to get position for joint %d. Return code: %d\n', jointIdx, returnCode);
        % Continue, but trajectory might be inaccurate. Or choose to stop:
        % sim.simxFinish(clientID); sim.delete(); return;
    end
end
fprintf('[INFO] Current Joint Positions (radians):\n');
disp(currentJointAngles);

% --- Compute Inverse Kinematics (IK) Solutions for Each Waypoint ---
fprintf('[INFO] Computing Inverse Kinematics for target waypoint(s)...\n');
targetJointConfigurations = zeros(numWaypoints, degreesOfFreedom);

for waypointIdx = 1:numWaypoints
    targetPosition_xyz = targetWaypoints(waypointIdx, 1:3)';  % Target position [x; y; z]
    targetOrientation_rpy = targetWaypoints(waypointIdx, 4:6); % Target orientation [roll, pitch, yaw] in degrees

    % Convert RPY orientation to a rotation matrix
    rotationMatrixTarget = rpy2rotmLocal(deg2rad(targetOrientation_rpy)); % Use local version
    
    % Create the 4x4 homogeneous transformation matrix for the target pose
    homogeneousTransformTarget = [rotationMatrixTarget, targetPosition_xyz; 0 0 0 1];
    
    % Compute the inverse kinematics solution(s)
    % invKin8sol should return multiple solutions if available.
    ikSolutions = invKin8sol(linkOffsets_d, linkLengths_a, homogeneousTransformTarget);
    
    % Select one solution (e.g., the 4th solution, as in original script)
    % It's crucial to choose a consistent and reachable solution.
    if size(ikSolutions, 1) >= 4 % Check if 4th solution exists
        targetJointConfigurations(waypointIdx, :) = ikSolutions(4, :);
    elseif ~isempty(ikSolutions) % Fallback to the first solution if 4th doesn't exist but others do
        fprintf('[WARN] IK solution 4 not available. Using solution 1 for waypoint %d.\n', waypointIdx);
        targetJointConfigurations(waypointIdx, :) = ikSolutions(1, :);
    else
        fprintf('[ERROR] No IK solution found for waypoint %d. Cannot proceed.\n', waypointIdx);
        sim.simxFinish(clientID);
        sim.delete();
        fprintf('[INFO] Disconnected from CoppeliaSim due to IK failure.\n');
        return;
    end
    
    fprintf('[INFO] Waypoint %d - Target Joint Angles (radians):\n', waypointIdx);
    disp(targetJointConfigurations(waypointIdx, :));
end

% --- Create Full Joint Configuration Sequence for Trajectory ---
% Prepend the current joint configuration to move from current to first waypoint.
jointConfigurationSequence = [currentJointAngles; targetJointConfigurations];
numSegments = size(jointConfigurationSequence, 1) - 1;

% --- Plan and Execute Trajectory Segments ---
fprintf('[INFO] Planning and executing trajectory...\n');
for segmentIdx = 1:numSegments
    fprintf('[INFO] Executing trajectory segment %d of %d (Waypoint %d to %d).\n', ...
            segmentIdx, numSegments, segmentIdx-1, segmentIdx);
            
    % Define trajectory parameters for this segment
    segmentDuration_T = 5.0;     % Duration of the segment in seconds
    timeStep = 0.05;           % Time step for trajectory execution
    timeVector = 0:timeStep:segmentDuration_T; % Time vector for this segment
    
    % Preallocate trajectory matrix for the current segment
    trajectorySegment = zeros(length(timeVector), degreesOfFreedom);
    
    % Generate a quintic trajectory for each joint for the current segment
    startJointAngles = jointConfigurationSequence(segmentIdx, :);
    endJointAngles   = jointConfigurationSequence(segmentIdx+1, :);
    
    for jointIdx = 1:degreesOfFreedom
        trajectorySegment(:, jointIdx) = quinticTrajectoryLocal(timeVector, segmentDuration_T, ...
            startJointAngles(jointIdx), endJointAngles(jointIdx), ...
            0, 0, 0, 0); % Zero initial/final velocity and acceleration
    end
    
    % (Optional) Plot the joint trajectories for the current segment
    % figure(3);
    % for jointIdxPlot = 1:degreesOfFreedom
    %     subplot(3, 2, jointIdxPlot);
    %     plot(timeVector, trajectorySegment(:, jointIdxPlot), 'LineWidth', 1.5);
    %     xlabel('Time (s)');
    %     ylabel(sprintf('Joint %d (rad)', jointIdxPlot));
    %     title(sprintf('Segment %d - Joint %d Trajectory', segmentIdx, jointIdxPlot));
    %     grid on;
    % end
    % sgtitle(sprintf('Joint Trajectories for %s - Segment %d', objectName, segmentIdx));

    % Execute the trajectory segment on the UR3 in simulation
    for timeStepIndex = 1:length(timeVector)
        for jointIdx = 1:degreesOfFreedom
            sim.simxSetJointTargetPosition(clientID, jointHandles(jointIdx), ...
                trajectorySegment(timeStepIndex, jointIdx), sim.simx_opmode_oneshot);
        end
        pause(timeStep); % Pause to allow simulation to keep up
    end
    fprintf('[INFO] Segment %d execution complete.\n', segmentIdx);
end
fprintf('[INFO] All trajectory segments executed.\n');

% --- Grasp the object ---
fprintf('[INFO] Grasping %s...\n', objectName);
graspRG2(clientID, sim, 'close'); % Close the gripper
pause(1); % Wait for gripper to close

%% ======================================================================= %%
%  POST-GRASP OPERATIONS (PREPARE FOR moveArmUp)                            %
%% ======================================================================= %%
fprintf('[INFO] Storing globals for use in subsequent scripts (e.g., moveArmUp.m).\n');

% Store the clientID, sim object, DH parameters, and joint handles in global variables
% so that 'moveArmUp.m' (or other scripts) can use the same connection and robot info.
global clientIDGlobal simGlobal dhParamsGlobal jointHandlesGlobal;
clientIDGlobal = clientID;
simGlobal = sim;
dhParamsGlobal.d = linkOffsets_d;
dhParamsGlobal.a = linkLengths_a;
jointHandlesGlobal = jointHandles;

% --- Run the moveArmUp script ---
% This script is expected to handle further arm movements and potentially disconnection.
fprintf('[INFO] Calling moveArmUp.m script...\n');
try
    run('moveArmUp.m');
catch moveArmUpException
    fprintf('[ERROR] Error occurred while running moveArmUp.m: %s\n', moveArmUpException.message);
    % Decide on cleanup if moveArmUp fails
    % sim.simxFinish(clientIDGlobal); % clientIDGlobal might be the one moveArmUp uses
    % sim.delete();
    % fprintf('[INFO] Disconnected from CoppeliaSim due to error in moveArmUp.m.\n');
end

%% ======================================================================= %%
%  CLEANUP (IF NOT HANDLED BY moveArmUp.m)                                  %
%% ======================================================================= %%
% The original script notes: "Don't disconnect as moveArmUp will handle that"
% If moveArmUp.m is guaranteed to disconnect, these lines are not needed.
% If there's a chance moveArmUp.m doesn't run or fails before disconnecting,
% uncommenting a cleanup here might be a fallback.
% For now, respecting the original comment.

% fprintf('[INFO] Cartesian Grasping Program finished.\n');
% if clientID > -1 % Check if still connected (e.g. if moveArmUp didn't run)
%     % sim.simxFinish(clientID);
%     % sim.delete();
%     % fprintf('[INFO] Disconnected from CoppeliaSim by main script (fallback).\n');
% end


%% ======================================================================= %%
%  LOCAL HELPER FUNCTIONS                                                   %
%% ======================================================================= %%

function theta_t = quinticTrajectoryLocal(t, T_total, theta_start, theta_final, vel_start, vel_final, acc_start, acc_final)
    % quinticTrajectoryLocal Generates a quintic polynomial trajectory.
    %
    % Args:
    %   t (vector): Time vector for which to calculate trajectory points.
    %   T_total (double): Total duration of the trajectory.
    %   theta_start (double): Initial joint angle.
    %   theta_final (double): Final joint angle.
    %   vel_start (double): Initial joint velocity.
    %   vel_final (double): Final joint velocity.
    %   acc_start (double): Initial joint acceleration.
    %   acc_final (double): Final joint acceleration.
    %
    % Returns:
    %   theta_t (vector): Joint angles at each time point in t.

    % System of equations: M * coeffs = b
    M = [1,    0,      0,        0,          0,           0;
         1,    T_total, T_total^2, T_total^3, T_total^4,   T_total^5;
         0,    1,      0,        0,          0,           0;
         0,    1,      2*T_total, 3*T_total^2, 4*T_total^3, 5*T_total^4;
         0,    0,      2,        0,          0,           0;
         0,    0,      2,        6*T_total, 12*T_total^2, 20*T_total^3];
    
    b_vector = [theta_start; theta_final; vel_start; vel_final; acc_start; acc_final];
    
    % Solve for coefficients
    if rcond(M) < eps % Check if matrix is singular or badly conditioned
        fprintf('[ERROR] Quintic trajectory matrix M is singular or badly conditioned. Cannot compute coefficients.\n');
        theta_t = NaN(size(t)); % Return NaN or handle error appropriately
        % A simple linear interpolation might be a fallback, but changes functionality.
        % For now, returning NaN and erroring out.
        % Example: theta_t = linspace(theta_start, theta_final, length(t));
        return;
    end
    coeffs = M \ b_vector;
    
    % Calculate trajectory positions
    theta_t = coeffs(1) + ...
              coeffs(2) * t + ...
              coeffs(3) * t.^2 + ...
              coeffs(4) * t.^3 + ...
              coeffs(5) * t.^4 + ...
              coeffs(6) * t.^5;
end

% --- rpy2rotmLocal Function ---
function R = rpy2rotmLocal(rpy_radians)
    % rpy2rotmLocal Converts Roll, Pitch, Yaw angles (in radians) to a 3x3 Rotation Matrix.
    % Uses ZYX convention for Euler angles.
    %
    % Args:
    %   rpy_radians (vector): 1x3 vector [roll, pitch, yaw] in radians.
    %
    % Returns:
    %   R (matrix): 3x3 Rotation Matrix.

    roll  = rpy_radians(1);
    pitch = rpy_radians(2);
    yaw   = rpy_radians(3);

    % Rotation matrix around X-axis
    R_x = [1,  0,           0;
           0,  cos(roll),  -sin(roll);
           0,  sin(roll),   cos(roll)];

    % Rotation matrix around Y-axis
    R_y = [cos(pitch),  0,  sin(pitch);
           0,           1,  0;
          -sin(pitch),  0,  cos(pitch)];

    % Rotation matrix around Z-axis
    R_z = [cos(yaw),  -sin(yaw),  0;
           sin(yaw),   cos(yaw),  0;
           0,          0,         1];

    % Combined rotation matrix (ZYX convention)
    R = R_z * R_y * R_x;
end