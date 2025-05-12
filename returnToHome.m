function returnToHome(clientID, sim)
    % RETURNTOHOME Function to smoothly reset UR3 robot orientation to default
    % Inputs: clientID - CoppeliaSim connection ID
    %         sim - RemoteAPI object
    
    fprintf('\n=== INITIATING SMOOTH RETURN TO HOME FOR UR3 ===\n');
    
    % Parameters for smooth movement
    duration = 5.0; % Total duration for the movement in seconds
    steps = 100;    % Number of interpolation steps
    pauseDuration = duration/steps; % Time between each step
    
    % Get UR3 robot handle and joint handles
    [~, ur3Handle] = sim.simxGetObjectHandle(clientID, 'UR3', sim.simx_opmode_blocking);
    
    % Define UR3 joint names (adjust based on your CoppeliaSim scene)
    jointNames = {'UR3_joint1', 'UR3_joint2', 'UR3_joint3', ...
                  'UR3_joint4', 'UR3_joint5', 'UR3_joint6'};
    jointHandles = zeros(1, 6);
    
    % Get current joint positions and target positions
    currentPositions = zeros(1, 6);
    targetPositions = zeros(1, 6); % Home position (all zeros)
    
    for i = 1:6
        [~, jointHandles(i)] = sim.simxGetObjectHandle(clientID, jointNames{i}, sim.simx_opmode_blocking);
        [~, currentPositions(i)] = sim.simxGetJointPosition(clientID, jointHandles(i), sim.simx_opmode_blocking);
    end
    
    fprintf('Moving smoothly to home position over %.1f seconds...\n', duration);
    
    % Perform smooth interpolation
    for step = 0:steps
        % Calculate interpolation factor (0 to 1)
        t = step/steps;
        
        % Cubic easing function for smoother movement (ease in-out)
        if t < 0.5
            easedT = 4 * t^3;
        else
            easedT = 1 - ((-2 * t + 2)^3)/2;
        end
        
        % Interpolate between current and target positions
        interpolatedPositions = currentPositions * (1 - easedT) + targetPositions * easedT;
        
        % Set new joint positions
        for i = 1:6
            sim.simxSetJointTargetPosition(clientID, jointHandles(i), interpolatedPositions(i), sim.simx_opmode_oneshot);
        end
        
        % Small pause to allow movement
        pause(pauseDuration);
    end
    
    % Final check to ensure we reached the target
    for i = 1:6
        sim.simxSetJointTargetPosition(clientID, jointHandles(i), targetPositions(i), sim.simx_opmode_blocking);
    end
    
    fprintf('UR3 has smoothly returned to default orientation.\n');
    fprintf('=== UR3 RETURN TO HOME COMPLETED ===\n');
end