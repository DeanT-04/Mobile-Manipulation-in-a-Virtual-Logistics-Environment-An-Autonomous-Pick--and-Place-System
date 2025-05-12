function stickObject(clientID, sim, objectName)
    % This function "sticks" an object in place after being dropped
    % by temporarily disabling dynamics and then re-enabling with zero velocity
    %
    % Parameters:
    %   clientID - The CoppeliaSim client ID
    %   sim - The CoppeliaSim remote API object
    %   objectName - Name of the object to stick in place (string)
    
    fprintf('[OBJECT] Getting handle for "%s"...\n', objectName);
    [res, objectHandle] = sim.simxGetObjectHandle(clientID, objectName, sim.simx_opmode_blocking);
    
    if res ~= sim.simx_return_ok
        fprintf('[ERROR] Could not get handle for object "%s"\n', objectName);
        return;
    end
    
    fprintf('[OBJECT] Freezing "%s" in place to prevent rolling...\n', objectName);
    
    % Disable dynamics temporarily
    sim.simxSetObjectIntParameter(clientID, objectHandle, 3003, 0, sim.simx_opmode_oneshot);
    
    % Get current position and orientation
    [~, position] = sim.simxGetObjectPosition(clientID, objectHandle, -1, sim.simx_opmode_blocking);
    [~, orientation] = sim.simxGetObjectOrientation(clientID, objectHandle, -1, sim.simx_opmode_blocking);
    
    % Slight pause to ensure command is processed
    pause(0.2);
    
    % Set linear and angular velocities to zero
    sim.simxSetObjectFloatParameter(clientID, objectHandle, 3001, 0, sim.simx_opmode_oneshot);  % Linear velocity
    sim.simxSetObjectFloatParameter(clientID, objectHandle, 3002, 0, sim.simx_opmode_oneshot);  % Angular velocity
    
    % Re-enable dynamics
    sim.simxSetObjectIntParameter(clientID, objectHandle, 3003, 1, sim.simx_opmode_oneshot);
    
    fprintf('[OBJECT] Object "%s" stabilized at position [%.3f, %.3f, %.3f]\n', objectName, position(1), position(2), position(3));
end