% MainScriptRunningAllProcesses.m - Minimal Version
clc; clear;
disp('Starting Pick-Place Operation');

% Declare global variables for clientID and sim
global clientID sim;

% Configuration
objectsToPick = [2, 1, 3];          % 1=orange, 2=bottle, 3=cup
objectNames = {'Orange', 'Bottle', 'Cup'};
maxRetries = 2;                     % Max retry attempts
retryDelays = [5, 10];              % Retry delays (seconds)

% Main loop
for objIdx = 1:length(objectsToPick)
    currentObject = objectsToPick(objIdx);
    currentName = objectNames{currentObject};
    fprintf('\nProcessing %s (%d/3)\n', currentName, objIdx);
    
    retryCount = 0;
    while retryCount <= maxRetries
        try
            % Set global object ID
            global objectToPick objectNameForBrownTable;
            objectToPick = currentObject;
            objectNameForBrownTable = currentName;
            
            % Execute sequence
            run('PPE_White_Table.m');            % Step 1: Go to white table
            pause(2)
            run('object_detectV3.m');            % Step 2: Detect object
            pause(2)
            run('Cartesian_grasping_script.m'); % Step 3: Grasp
            pause(2)
            run('PPE_Brown_Table.m');            % Step 4: Move to brown table (includes drop-off)
            pause(2)
            
            fprintf('%s: Success!\n', currentName);
            break; % Exit retry loop on success
            
        catch e
            retryCount = retryCount + 1;
            fprintf('Attempt %d failed: %s\n', retryCount, e.message);
            if retryCount <= maxRetries
                pause(retryDelays(retryCount));
            end
        end
    end
end
