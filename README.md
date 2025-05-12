# Autonomous Mobile Manipulator: Pick-and-Place in CoppeliaSim & MATLAB

Welcome to the repository for my ENGD3030 Robotics & AI coursework project! This project showcases an autonomous mobile manipulator system designed to perform a complete pick-and-place task in a simulated industrial logistics environment. The entire system was developed using MATLAB for control logic and algorithm implementation, interfaced with CoppeliaSim (formerly V-REP) for realistic 3D simulation.

## 🚀 Project Overview

The core challenge was to integrate various robotics and AI concepts to enable a Pioneer P3-DX mobile robot, equipped with a UR3 robotic arm and an RG2 gripper, to:
1.  **Map its environment** using SLAM (Simultaneous Localization and Mapping).
2.  **Navigate autonomously** to different locations (a white pick-up table and a brown drop-off table) using A* path planning.
3.  **Detect and identify objects** (a bottle, an orange, and a cup) on the white table using a vision sensor and the YOLOv4 object detection model.
4.  **Estimate the 3D poses** of these objects.
5.  **Plan and execute precise arm trajectories** using inverse kinematics and quintic polynomials to grasp the objects.
6.  **Transport and accurately place** the objects at designated spots on the brown table.

It was a fascinating journey bringing all these components together to create a cohesive and functional system!

## 🎬 Demonstration Video

Seeing is believing! Check out the full operation of the robot in action:

[![Untitled design (2)](https://github.com/user-attachments/assets/7c5b8e36-cced-4371-9ee3-e553b7a123b4)](https://youtu.be/Nke4raYkS_U)


## ✨ Key Features & Technologies

*   **Simulation Environment:** CoppeliaSim 4.x EDU
*   **Control & Algorithm Development:** MATLAB R2023a (or similar)
*   **Mobile Robot:** Pioneer P3-DX
*   **Robotic Arm:** Universal Robots UR3 (6-DOF)
*   **Gripper:** RG2 Parallel Gripper
*   **Sensors:**
    *   Simulated 2D LiDAR (for SLAM and navigation)
    *   Simulated Perspective Vision Sensor (for object detection)
*   **Core Algorithms & Techniques:**
    *   **SLAM:** `lidarSLAM` object (MATLAB Navigation Toolbox) for 2D occupancy grid mapping.
    *   **Path Planning:** A* algorithm (`plannerAStarGrid` in MATLAB) with map inflation for collision-free mobile robot navigation.
    *   **Navigation Control:** PID controllers for smooth path following.
    *   **Object Detection:** YOLOv4 (`yolov4ObjectDetector` in MATLAB Deep Learning Toolbox) for identifying and classifying objects.
    *   **Pose Estimation:** Custom scripts to transform 2D detection coordinates to 3D poses in the robot's base frame.
    *   **Arm Kinematics:**
        *   Denavit-Hartenberg (DH) parameterization for the UR3.
        *   Analytical Inverse Kinematics (`invKin8sol.m`) to calculate joint angles from Cartesian poses.
        *   Forward Kinematics (implicitly used for workspace understanding and waypoint validation).
    *   **Trajectory Planning:** Quintic polynomial trajectories for smooth joint-space motion of the UR3 arm.
    *   **Gripper Control:** Signal-based control via CoppeliaSim's remote API.

## 📂 Repository Structure

The project is organized into several MATLAB scripts, each responsible for a specific part of the task:

*   `MainScriptRunningAllProcesses.m`: The main script that orchestrates the entire pick-and-place sequence for all objects.
*   `map_creation.m` & `slamFunc.m`: Scripts for generating the environment map using SLAM and teleoperation.
*   `PPE_White_Table.m` & `PPE_Brown_Table.m`: Handle the navigation of the mobile robot to the respective tables.
*   `pathPlanner.m`: Implements the A* path planning algorithm.
*   `executePathPID_WHITE.m` & `executePathPID_BROWN.m`: Control the robot's movement along the planned paths.
*   `swivelRobot.m`: For precise orientation of the robot at the tables.
*   `object_detectV3.m`: Performs object detection and pose estimation using YOLOv4.
*   `Cartesian_grasping_script.m`: Manages the UR3 arm for grasping objects.
*   `moveArmUp.m`: Lifts the arm after grasping.
*   `dropOffAtBrownTable.m`: Controls arm movement for placing objects and subsequent actions.
*   `graspRG2.m`: Handles gripper open/close commands.
*   `stickObject.m`: "Sticks" objects in place after they are dropped to prevent rolling.
*   `returnToHome.m`: Returns the UR3 arm to its home position.
*   **Kinematics & API:**
    *   `invKin8sol.m`: Inverse kinematics solver for the UR3.
    *   `MDHMatrix.m`: Computes transformation matrices from DH parameters.
    *   `remApi.m`, `remoteApiProto.m`: CoppeliaSim remote API files for MATLAB.
*   **Helper Scripts:**
    *   `computeObjectAverages.m`, `computeAverage.m`, `frameDataStorage.m`: Utilities for processing and storing object detection data.
*   **CoppeliaSim Scene File:**
    *   `robotics_logistics_environment_bottle_change.ttt` (or similar name): The simulation scene file.
*   **Saved Map:**
    *   `slamMap.mat`: The generated 2D occupancy map.

## 🛠️ How to Run

1.  **Prerequisites:**
    *   MATLAB (R2023a or a compatible version) with the following toolboxes:
        *   Navigation Toolbox (for SLAM and path planning)
        *   Robotics System Toolbox (for kinematics, transformations)
        *   Deep Learning Toolbox (for YOLOv4 object detection)
        *   Image Processing Toolbox
    *   CoppeliaSim EDU (Version 4.x recommended). Ensure the remote API server is enabled (typically on port 19997).
2.  **Setup:**
    *   Clone this repository.
    *   Ensure the `remApi.m`, `remoteApiProto.m`, and the appropriate remote API library file (e.g., `remoteApi.dll` for Windows) are in your MATLAB path or the project's root directory.
    *   Open the `robotics_logistics_environment_bottle_change.ttt` (or your scene file name) in CoppeliaSim.
3.  **Execution Steps:**
    *   **Mapping (Optional if `slamMap.mat` is already provided and satisfactory):**
        1.  In CoppeliaSim, start the simulation.
        2.  Run `map_creation.m` from MATLAB. A figure window will appear for teleoperation.
        3.  Use W (forward), A (left turn), S (backward), D (right turn) keys to drive the robot around the environment and build the map.
        4.  Close the teleoperation figure window to stop mapping and save the map (`slamMap.mat`, `SLAM_OccupancyMap.fig`, `SLAMPoseGraph.fig`).
        5.  Stop the simulation in CoppeliaSim.
    *   **Main Pick-and-Place Operation:**
        1.  In CoppeliaSim, **ensure the simulation is running.**
        2.  Open and run `MainScriptRunningAllProcesses.m` in MATLAB.
        3.  Watch the robot autonomously perform the pick-and-place tasks for the bottle, orange, and cup! MATLAB's command window will display progress and logs.

## 💡 Challenges & Learnings

This project was a fantastic learning experience, but not without its challenges!
*   **PID Tuning:** Getting the mobile robot to navigate smoothly and accurately required quite a bit of iteration on the PID controller gains.
*   **IK Solutions:** The UR3 arm has multiple solutions for its inverse kinematics. Choosing a consistent and collision-free solution was key.
*   **Object Pose Accuracy:** Translating 2D detections from YOLOv4 into precise 3D world coordinates for grasping needed careful calibration and transformation logic. Averaging over multiple frames helped a lot here.
*   **System Integration:** Making all the individual modules (SLAM, navigation, vision, arm control) work together seamlessly was the biggest, but most rewarding, part of the project. Debugging often involved tracing data flow across multiple scripts and the CoppeliaSim interface.
*   **Object Naming Consistency:** A small hiccup occurred with the "cup" object due to a naming mismatch between the detection phase and the stabilization phase. A good reminder about the importance of consistent data handling in complex systems!

## 🔮 Future Improvements

While the system performs the task successfully, there's always room for improvement:
*   **Enhanced Error Handling:** More robust error detection (e.g., failed grasp detection) and recovery strategies.
*   **Dynamic Obstacle Avoidance:** Implementing real-time replanning for the mobile base if unexpected obstacles appear.
*   **Improved Pose Estimation:** Incorporating depth data or more advanced 6D pose estimation techniques for even greater accuracy.
*   **Reinforcement Learning:** Exploring RL for optimizing navigation paths or even learning grasping strategies.

## 🙏 Acknowledgements

*   This project was completed as part of the ENGD3030 Robotics & AI module.
*   Thanks to the developers of CoppeliaSim and MATLAB for providing such powerful tools for robotics research and education.

---

Feel free to explore the code and reach out if you have any questions!
