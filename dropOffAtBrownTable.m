function dropOffAtBrownTable(clientID, sim)  
    % === Load stored object positions ===
    fprintf('[LOADING] Loading stored object positions...\n');
    run('frameDataStorage.m');
    global orangeAvgX orangeAvgY bottleAvgX bottleAvgY cupAvgX cupAvgY
    
    % === Load picked object info ===
    global objectToPick objectName
    if ~exist('objectToPick', 'var')
        objectToPick = 2; % Default to bottle
    end
    
    % === Object parameters (in meters) ===
    objectHeights     = [0.08, 0.18, 0.10];
    objectClearances  = [3, 3, 3]; % Reduced from 3m to 3cm
    approachHeights   = [0.1, 0.18, 0.95];
    
    % === Get robot & gripper handles ===
    fprintf('[INIT] Getting robot handles...\n');
    [~, ur3Handle] = sim.simxGetObjectHandle(clientID, 'UR3', sim.simx_opmode_blocking);
    
    % === Locate brown table ===
    fprintf('[TABLE] Locating brown table...\n');
    [~, brownTableHandle] = sim.simxGetObjectHandle(clientID, 'customizableTable', sim.simx_opmode_blocking);
    [~, tablePos] = sim.simxGetObjectPosition(clientID, brownTableHandle, -1, sim.simx_opmode_blocking);
    [~, tableOri] = sim.simxGetObjectOrientation(clientID, brownTableHandle, -1, sim.simx_opmode_blocking);
    
    fprintf('[TABLE] Table position: X=%.3f, Y=%.3f, Z=%.3f\n', tablePos(1), tablePos(2), tablePos(3));
    
    % === Drop parameters ===
    objHeight = objectHeights(objectToPick);
    clearance = objectClearances(objectToPick);
    approachHeight = approachHeights(objectToPick);
    dropHeight = tablePos(3) + objHeight/2 + clearance;
    
    fprintf('[SAFETY] Object "%s" dimensions:\n', objectName);
    fprintf('  Height: %.3fm (%.1fcm)\n', objHeight, objHeight * 100);
    fprintf('  Clearance: %.3fm (%.1fcm)\n', clearance, clearance * 100);
    fprintf('  Approach height: %.3fm (%.1fcm)\n', approachHeight, approachHeight * 100);
    fprintf('  Calculated drop height: %.3fm (%.1fcm)\n', dropHeight, dropHeight * 100);
    
    % === DH Parameters ===
    dof = 6;
    d = [0.10887, 0.11154, 0, 0, 0.11223 - 0.11154, 0.194 - 0.11223, 0];
    a = [0, 0.35252 - 0.10887, 0.56577 - 0.35252, 0.64999 - 0.56577, 0.65111 - 0.64999, 0];
    
    % === Define waypoints for drop ===
    dropOrientation = [0, 180, 0]; % [roll, pitch, yaw]
    waypoints = [
        tablePos(1), tablePos(2), dropHeight + approachHeight, dropOrientation;
        tablePos(1), tablePos(2), dropHeight, dropOrientation;
        tablePos(1), tablePos(2), dropHeight + approachHeight, dropOrientation
    ];
    
    numWaypoints = size(waypoints, 1);
    fprintf('[PLAN] Planned trajectory with %d waypoints:\n', numWaypoints);
    disp(waypoints);
    
    % === Get joint handles ===
    fprintf('[JOINTS] Getting joint handles...\n');
    jh = zeros(1, dof);
    for i = 1:dof
        [~, jh(i)] = sim.simxGetObjectHandle(clientID, ['UR3_joint', int2str(i)], sim.simx_opmode_blocking);
    end
    
    % === Get current joint positions ===
    currentJoints = zeros(1, dof);
    for i = 1:dof
        [~, currentJoints(i)] = sim.simxGetJointPosition(clientID, jh(i), sim.simx_opmode_blocking);
    end
    fprintf('[JOINTS] Current joint positions (rad):\n');
    disp(currentJoints);
    
    % === Inverse kinematics for each waypoint ===
    fprintf('[IK] Calculating inverse kinematics (solution 4)...\n');
    targetJointsAll = zeros(numWaypoints, dof);
    
    for wp = 1:numWaypoints
        pos = waypoints(wp, 1:3)';
        ori = deg2rad(waypoints(wp, 4:6));
        
        R_target = rpy2rotm(ori);
        T_target = [R_target, pos; 0 0 0 1];
        
        joints = invKin8sol(d, a, T_target);
        solIdx = min(4, size(joints, 1));
        targetJointsAll(wp, :) = joints(solIdx, :);
        
        fprintf('[IK] Waypoint %d (Solution %d):\n', wp, solIdx);
        fprintf('  Position: [%.3f, %.3f, %.3f] Orientation: [%.1f°, %.1f°, %.1f°]\n', ...
                pos(1), pos(2), pos(3), waypoints(wp, 4), waypoints(wp, 5), waypoints(wp, 6));
        fprintf('  Joints: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f rad\n', joints(solIdx, :));
    end
    
    % === Create full joint sequence ===
    jointConfigSeq = [currentJoints; targetJointsAll];
    numSegments = size(jointConfigSeq, 1) - 1;
    
    % === Execute trajectory ===
    fprintf('[EXEC] Starting trajectory execution...\n');
    for seg = 1:numSegments
        fprintf('[EXEC] Segment %d/%d: From waypoint %d to %d\n', seg, numSegments, seg - 1, seg);
        
        T_seg = 3;
        timeStep = 0.05;
        timeVec = 0:timeStep:T_seg;
        trajSegment = zeros(length(timeVec), dof);
        
        for j = 1:dof
            trajSegment(:, j) = quinticTrajectory(timeVec, T_seg, ...
                jointConfigSeq(seg, j), jointConfigSeq(seg + 1, j), 0, 0, 0, 0);
        end
        
        for idx = 1:length(timeVec)
            for j = 1:dof
                sim.simxSetJointTargetPosition(clientID, jh(j), trajSegment(idx, j), sim.simx_opmode_oneshot);
            end
            pause(timeStep);
        end
        
        % === Open gripper after drop ===
        if seg == 2
            pause(2);  % Ensure robot stops moving
            fprintf('[GRIPPER] Opening gripper at drop position\n');
            
            % Get the correct object name based on objectToPick
            objectNames = {'Orange', 'bottle_1_convexDecomposed', 'koffie'};
            currentObjectName = objectNames{objectToPick};
            
            graspRG2(clientID, sim, 'open');
            pause(1);  % Short pause to allow gripper to begin opening
            
            % Call the stickObject function from the imported script
            stickObject(clientID, sim, currentObjectName);
            
            pause(2);  % Additional time to ensure gripper completes opening
        end
    end

    % === Reverse Pioneer Robot ===
    fprintf('[PIONEER] Reversing Pioneer robot for 2 seconds\n');
    [resL, leftMotorHandle] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
    [resR, rightMotorHandle] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
    
    if resL ~= sim.simx_return_ok || resR ~= sim.simx_return_ok
        fprintf('[PIONEER ERROR] Unable to get motor handles\n');
    else
        reverseVelocity = -1; % rad/s
        reverseDuration = 6; % seconds
        sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, reverseVelocity, sim.simx_opmode_oneshot);
        sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, reverseVelocity, sim.simx_opmode_oneshot);
        pause(reverseDuration);
        % Stop the motors
        sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_blocking);
        fprintf('[PIONEER] Stopped reversing\n');
    end

    % After reversing stops:
    fprintf('[PIONEER] Stopped reversing\n');
    
    % Call the returnToHome function
    returnToHome(clientID, sim);  % Call the function with existing clientID and sim
    
    fprintf('[STATUS] Successfully placed %s on the table\n', objectName);
    sim.simxFinish(clientID);
    
    sim.delete();
end

%% === Helper Functions ===

function theta = quinticTrajectory(t, T, theta0, thetaf, vel0, velf, acc0, accf)
    A = [1,0,0,0,0,0; 1,T,T^2,T^3,T^4,T^5; 0,1,0,0,0,0;
         0,1,2*T,3*T^2,4*T^3,5*T^4; 0,0,2,0,0,0; 0,0,2,6*T,12*T^2,20*T^3];
    b = [theta0; thetaf; vel0; velf; acc0; accf];
    coeffs = A\b;
    theta = coeffs(1) + coeffs(2)*t + coeffs(3)*t.^2 + coeffs(4)*t.^3 + coeffs(5)*t.^4 + coeffs(6)*t.^5;
end

function R = rpy2rotm(rpy)
    roll = rpy(1); pitch = rpy(2); yaw = rpy(3);
    R_x = [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];
    R_y = [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)];
    R_z = [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];
    R = R_z * R_y * R_x;
end