% Initialize the CoppeliaSim remote API
sim = remApi('remoteApi');
sim.simxFinish(-1); % Close any existing connections
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

% === Load global storage and variables ===
run('frameDataStorage.m');
global orangeX orangeY
global bottleX bottleY
global cupX cupY

if clientID > -1
    disp('Connected to CoppeliaSim');

    % Get handles for the vision sensors
    [~, perspHandle] = sim.simxGetObjectHandle(clientID, 'Vision_sensor_persp', sim.simx_opmode_blocking);
    disp('Vision sensor handles retrieved successfully.');

    % Get handle for the UR3 robot
    [~, ur3Handle] = sim.simxGetObjectHandle(clientID, 'UR3', sim.simx_opmode_blocking);

    disp('Initializing YOLO object detector...');
    model = yolov4ObjectDetector('csp-darknet53-coco'); % Pre-trained YOLOv4 model

    % Main loop: Process 6 frames
    for frame = 1:5
        [res1, resolution1, image1] = sim.simxGetVisionSensorImage2(clientID, perspHandle, 0, sim.simx_opmode_blocking);

        if res1 == sim.simx_return_ok
            disp(['Processing frame ', num2str(frame), '...']);
            [bboxes1, scores1, labels1] = detect(model, image1);
            detectedImage1 = insertObjectAnnotation(image1, 'rectangle', bboxes1, labels1, 'Color', 'yellow', 'TextBoxOpacity', 0.7, 'FontSize', 12);

            objectPoses1 = estimateObjectPose(sim, clientID, perspHandle, model);
            [~, UR3Position] = sim.simxGetObjectPosition(clientID, ur3Handle, -1, sim.simx_opmode_blocking);
            [~, UR3Orientation] = sim.simxGetObjectOrientation(clientID, ur3Handle, -1, sim.simx_opmode_blocking);

            % Print header for current frame
            fprintf('Frame %d:\n', frame);

            for i = 1:length(objectPoses1)
                baseFramePos = WorldFrametoUr3(objectPoses1(i).Position, UR3Position, UR3Orientation);
                rframex = baseFramePos(1);
                rframey = baseFramePos(2);
                rframez = baseFramePos(3);

                % Print only RFrame values in clean format
                fprintf('  %6s: X = %6.3f  Y = %6.3f  Z = %6.3f\n', ...
                         objectPoses1(i).Label, rframex, rframey, rframez);

                label = objectPoses1(i).Label;

                switch label
                    case 'orange'
                        if length(orangeX) < frame
                            orangeX{frame} = [];
                            orangeY{frame} = [];
                        end
                        orangeX{frame}(end+1) = rframex;
                        orangeY{frame}(end+1) = rframey;

                    case 'bottle'
                        if length(bottleX) < frame
                            bottleX{frame} = [];
                            bottleY{frame} = [];
                        end
                        bottleX{frame}(end+1) = rframex;
                        bottleY{frame}(end+1) = rframey;

                    case 'cup'
                        if length(cupX) < frame
                            cupX{frame} = [];
                            cupY{frame} = [];
                        end
                        cupX{frame}(end+1) = rframex;
                        cupY{frame}(end+1) = rframey;
                end
            end

            fprintf('\n'); % Line break between frames

            figure(2);
            imshow(detectedImage1);
            title(['Perspective Sensor Detection - Frame ', num2str(frame)]);
            pause(0.5);
        else
            disp('Failed to capture image from sensor.');
        end
    end

    % === Compute final average positions after loop ===
    computeObjectAverages();

    sim.simxFinish(clientID);
    disp('Disconnected from CoppeliaSim');
else
    disp('Failed to connect to CoppeliaSim');
end

sim.delete(); % Clear API instance


%% Helper Functions
function objectPoses = estimateObjectPose(sim, clientID, visionSensorHandle, model)
objectPoses = [];
[res, ~, capturedImage] = sim.simxGetVisionSensorImage2(clientID, visionSensorHandle, 0, sim.simx_opmode_blocking);
if res == sim.simx_return_ok
    [bboxes, scores, labels] = detect(model, capturedImage);
    if ~isempty(bboxes)
        [~, visionSensorPose] = sim.simxGetObjectPosition(clientID, visionSensorHandle, -1, sim.simx_opmode_blocking);
        [~, visionSensorOrientation] = sim.simxGetObjectOrientation(clientID, visionSensorHandle, -1, sim.simx_opmode_blocking);
        for i = 1:size(bboxes, 1)
            bbox = bboxes(i, :);
            bboxCenterX = bbox(1) + bbox(3)/2;
            bboxCenterY = bbox(2) + bbox(4)/2;
            normX = -(bboxCenterX - size(capturedImage, 2)/2) / (size(capturedImage, 2)/2);
            normY = (size(capturedImage, 1)/2 - bboxCenterY) / (size(capturedImage, 1)/2);
            approxDepth = 0.53;
            x = normX * approxDepth * 0.51;
            y = normY * approxDepth;
            z = approxDepth;
            objectWorldPosition = transformToWorldFrame([x, y, z], visionSensorPose, visionSensorOrientation);
            objectPoses(end+1).Label = labels(i);
            objectPoses(end).Position = objectWorldPosition;
            objectPoses(end).Orientation = visionSensorOrientation;
        end
    end
end
end

function worldPosition = transformToWorldFrame(localPosition, sensorPosition, sensorOrientation)
rotationMatrix = eul2rotm(sensorOrientation, 'XYZ');
worldPosition = (rotationMatrix * localPosition')' + sensorPosition;
end

function baseFramePos = WorldFrametoUr3(globalPosition, UR3Position, UR3Orientation)
R = eul2rotm(UR3Orientation, 'XYZ');
T = [R, UR3Position(:); 0 0 0 1];
T_inv = inv(T);
worldHom = [globalPosition(:); 1];
baseHom = T_inv * worldHom;
baseFramePos = baseHom(1:3);
end