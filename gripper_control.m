% Initialize Remote API connection
sim = remApi('remoteApi');
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
if clientID > -1
disp('Connected to CoppeliaSim');
% Define the signal name used in the Lua script
gripperSignal = 'RG2_open';
% Open the gripper by sending the signal
openSignalValue = 1; % Send '1' to open the gripper
sim.simxSetIntegerSignal(clientID, gripperSignal, openSignalValue,sim.simx_opmode_blocking);
disp('Signal sent to open the gripper');
pause(2); % Wait for the gripper to fully open
% Close the gripper by sending the signal
closeSignalValue = 0; % Send '0' to close the gripper
sim.simxSetIntegerSignal(clientID, gripperSignal, closeSignalValue,sim.simx_opmode_blocking);
disp('Signal sent to close the gripper');
pause(2); % Wait for the gripper to fully close
% Stop the simulation
sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
sim.simxFinish(clientID);
else
disp('Failed to connect to CoppeliaSim');
end
% Cleanup
sim.delete();
