#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import serial

class UnifiedRobotController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('unified_robot_controller')
        
        # Configuration parameters (adjust these for your hardware)
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.target_latitude = rospy.get_param('~target_lat', 30.944114)
        self.target_longitude = rospy.get_param('~target_lon', 30.103464)
        self.max_linear_velocity = rospy.get_param('~max_linear', 1.0)
        self.max_angular_velocity = rospy.get_param('~max_angular', 1.0)
        self.obstacle_threshold = rospy.get_param('~obstacle_thresh', 0.5)  # meters
        
        # Motor command mappings (adjust for your Arduino code)
        self.motor_commands = {
            'left_forward': b'4',
            'left_backward': b'5',
            'right_forward': b'6',
            'right_backward': b'7'
        }
        
        # Initialize serial connection for motor control
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo(f"Connected to serial port {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to serial port: {e}")
            rospy.signal_shutdown("Serial connection failed")
            return
        
        # Robot state variables
        self.current_latitude = None
        self.current_longitude = None
        self.orientation = None  # in radians
        self.ultrasonic_data = None
        self.imu_data = None
        
        # Setup ROS subscribers
        rospy.Subscriber('/gps_data', NavSatFix, self.gps_callback)
        rospy.Subscriber('/imu_data', Imu, self.imu_callback)
        rospy.Subscriber('/compass_data', MagneticField, self.compass_callback)
        rospy.Subscriber('/ultrasonic_data', String, self.ultrasonic_callback)
        
        # Setup ROS publisher (for potential higher-level commands)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Control loop rate
        self.rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo("Unified Robot Controller initialized")

    def gps_callback(self, data):
        """Update current GPS coordinates"""
        self.current_latitude = data.latitude
        self.current_longitude = data.longitude

    def imu_callback(self, data):
        """Update orientation from IMU data"""
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.orientation = yaw

    def compass_callback(self, data):
        """Update orientation from compass (backup if no IMU)"""
        if self.orientation is None:  # Only use compass if we don't have IMU data
            self.orientation = data.magnetic_field.z

    def ultrasonic_callback(self, data):
        """Process ultrasonic sensor data for obstacle detection"""
        try:
            self.ultrasonic_data = [float(d) for d in data.data.split(',')]
        except ValueError:
            rospy.logwarn("Invalid ultrasonic data format")

    def set_motor_speeds(self, left_speed, right_speed):
        """Send motor commands via serial connection"""
        try:
            # Left motor
            if left_speed > 0:
                self.ser.write(self.motor_commands['left_forward'])
            else:
                self.ser.write(self.motor_commands['left_backward'])
            
            # Right motor
            if right_speed > 0:
                self.ser.write(self.motor_commands['right_forward'])
            else:
                self.ser.write(self.motor_commands['right_backward'])
            
            # Send speed values (convert to bytes)
            self.ser.write(bytes([int(abs(left_speed)*255]))
            self.ser.write(bytes([int(abs(right_speed)*255]))
            
        except serial.SerialException as e:
            rospy.logerr(f"Motor command failed: {e}")

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates using Haversine formula"""
        # Convert degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371 * c * 1000  # Earth radius in km -> convert to meters
        
        return distance

    def calculate_target_angle(self, target_lat, target_lon):
        """Calculate required heading to reach target coordinates"""
        if None in [self.current_latitude, self.current_longitude, self.orientation]:
            return 0.0
            
        # Calculate bearing to target
        dlon = math.radians(target_lon - self.current_longitude)
        lat1 = math.radians(self.current_latitude)
        lat2 = math.radians(target_lat)
        
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        
        # Calculate angle difference between current orientation and target bearing
        angle = bearing - self.orientation
        
        # Normalize angle to [-π, π]
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
            
        return angle

    def obstacle_detected(self):
        """Check ultrasonic sensors for obstacles"""
        if self.ultrasonic_data is None:
            return False
        return any(d < self.obstacle_threshold for d in self.ultrasonic_data)

    def perform_obstacle_avoidance(self):
        """Execute obstacle avoidance maneuver"""
        rospy.loginfo("Obstacle detected! Performing avoidance maneuver")
        
        # Stop the robot
        self.set_motor_speeds(0, 0)
        rospy.sleep(1)
        
        # Rotate 90 degrees to the right
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.0:  # Adjust time as needed
            self.set_motor_speeds(0.5, -0.5)
            rospy.sleep(0.1)
        
        # Move forward for 2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.0:
            self.set_motor_speeds(0.5, 0.5)
            rospy.sleep(0.1)
        
        # Stop after maneuver
        self.set_motor_speeds(0, 0)

    def navigate_to_target(self):
        """Main navigation logic to reach target coordinates"""
        while not rospy.is_shutdown():
            # Check for obstacles first
            if self.obstacle_detected():
                self.perform_obstacle_avoidance()
                continue
                
            # Check if we have all required data
            if None in [self.current_latitude, self.current_longitude, self.orientation]:
                rospy.logwarn("Waiting for sensor data...")
                self.rate.sleep()
                continue
                
            # Calculate distance and angle to target
            distance = self.haversine_distance(
                self.current_latitude, self.current_longitude,
                self.target_latitude, self.target_longitude
            )
            angle = self.calculate_target_angle(self.target_latitude, self.target_longitude)
            
            # Check if target reached
            if distance < 1.0:  # 1 meter threshold
                rospy.loginfo("Target reached!")
                self.set_motor_speeds(0, 0)
                break
                
            # Calculate motor speeds (differential drive)
            linear_vel = min(self.max_linear_velocity, distance/5)  # Scale down as we approach
            angular_vel = angle * self.max_angular_velocity
            
            # Differential drive calculations
            left_speed = linear_vel - angular_vel
            right_speed = linear_vel + angular_vel
            
            # Normalize speeds to stay within limits
            max_speed = max(abs(left_speed), abs(right_speed))
            if max_speed > self.max_linear_velocity:
                left_speed = left_speed * self.max_linear_velocity / max_speed
                right_speed = right_speed * self.max_linear_velocity / max_speed
                
            # Send motor commands
            self.set_motor_speeds(left_speed, right_speed)
            
            # Publish cmd_vel for other nodes (optional)
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist)
            
            self.rate.sleep()

    def run(self):
        """Main execution method"""
        try:
            self.navigate_to_target()
        except rospy.ROSInterruptException:
            self.set_motor_speeds(0, 0)  # Stop motors on shutdown
            rospy.loginfo("Controller shutdown requested")

if __name__ == '__main__':
    controller = UnifiedRobotController()
    controller.run()