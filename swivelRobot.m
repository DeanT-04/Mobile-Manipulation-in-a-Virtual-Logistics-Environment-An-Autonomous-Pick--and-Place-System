function swivelRobot(clientID, sim, pioneerHandle, leftMotorHandle, rightMotorHandle, tableType)
    % Determine target orientation in radians (global frame)
    if strcmpi(tableType, 'white')
        targetAngle = pi/2;  % +90 degrees
    elseif strcmpi(tableType, 'brown')
        targetAngle = -pi/2; % -90 degrees
    else
        return; % Exit if table type is invalid
    end

    % Loop until the robot reaches the target orientation
    while true
        % Get current orientation
        [~, robotOrientation] = sim.simxGetObjectOrientation(clientID, pioneerHandle, -1, sim.simx_opmode_buffer);
        currentAngle = robotOrientation(3); % Gamma (yaw) in radians

        % Calculate angle difference to target
        angleDiff = targetAngle - currentAngle;
        angleDiff = atan2(sin(angleDiff), cos(angleDiff)); % Normalize to [-pi, pi]

        % Check if within a tight threshold (0.0017 radians ≈ 0.1 degrees)
        if abs(angleDiff) < 0.0017
            % Stop motors when aligned
            sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_oneshot);
            sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_oneshot);
            pause(0.2); % Small delay to ensure the robot settles
            break; % Exit loop
        end

        % Adjust speed based on angle difference for finer control
        speed = min(0.2, abs(angleDiff) * 0.5); % Scale speed: max 0.2, slows down as it gets closer
        speed = max(speed, 0.05); % Minimum speed to ensure movement

        % Swivel direction based on whether current angle is above or below target
        if currentAngle < targetAngle
            % Current angle is below target: rotate counterclockwise
            % Right motor forward, left motor reverse
            sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, -speed, sim.simx_opmode_oneshot);
            sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, speed, sim.simx_opmode_oneshot);
        else
            % Current angle is above target: rotate clockwise
            % Left motor forward, right motor reverse
            sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, speed, sim.simx_opmode_oneshot);
            sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, -speed, sim.simx_opmode_oneshot);
        end
        pause(0.05); % Shorter pause for more frequent updates
    end
    disp('Swivel completed with precision.');
end