sim = remApi('remoteApi'); % Load CoppeliaSim Remote API 
sim.simxFinish(-1); % Close any previous connections
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
if clientID > -1
disp('Connected to CoppeliaSim'); 
sim.simxFinish(clientID); % Close connection
else
disp('Failed to connect to CoppeliaSim'); 
end
sim.delete(); % Release Remote API resources