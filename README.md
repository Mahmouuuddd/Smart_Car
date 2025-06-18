# Smart_Car
This code is for the implementation of ROS code to contact all nodes (SW) with the HW

There are three nodes:
 - main_node.py: Primary Purpose: Basic GPS navigation with serial communication
 - main_node2.py: Primary Purpose: Enhanced GPS navigation with IMU and obstacle avoidance
 - last.py: Primary Purpose: GPS-based navigation with obstacle avoidance

Then comes the brain_node.py

Key Features and Explanations
1. Initialization and Configuration
 - Sets up ROS node with configurable parameters (port, baudrate, target coordinates, etc.)

 - Establishes serial connection to Arduino for motor control

 - Initializes subscribers for all sensor data (GPS, IMU, compass, ultrasonic)

2. Sensor Callbacks
 - gps_callback: Updates current position from GPS data

 - imu_callback: Gets orientation from IMU (more accurate than compass)

 - compass_callback: Backup orientation source if IMU isn't available

 - ultrasonic_callback: Processes obstacle detection data

3. Core Navigation Functions
 - haversine_distance: Accurate GPS distance calculation using Earth's curvature

 - calculate_target_angle: Determines required heading to reach target

 - obstacle_detected: Checks ultrasonic sensors for nearby obstacles

 - perform_obstacle_avoidance: Executes standardized avoidance maneuver

4. Motor Control
 - set_motor_speeds: Converts speeds to appropriate serial commands

 - Implements differential drive calculations for turning

 - Handles speed normalization to stay within limits

5. Main Navigation Loop
 - Checks for obstacles first (highest priority)

 - Verifies sensor data is available

 - Calculates distance and angle to target

 - Determines if target is reached

 - Computes motor speeds using both linear and angular velocity

 - Normalizes speeds and sends commands

 - Optionally publishes cmd_vel for other nodes

6. Safety Features
 - Automatic motor stop on shutdown

 - Parameterized thresholds for easy adjustment

 - Comprehensive error checking

 - Graceful handling of missing sensor data

