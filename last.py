#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, MagneticField
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import serial

# Adjust these values as necessary to match your Arduino setup
PORT = '/dev/ttyACM0'
BAUDRATE = 115200

# Adjust these values to match the motor controller commands used by your Arduino code
LEFT_MOTOR_FORWARD = b'4'
LEFT_MOTOR_BACKWARD = b'5'
RIGHT_MOTOR_FORWARD = b'6'
RIGHT_MOTOR_BACKWARD = b'7'

# Adjust these values based on your hardware and desired behavior
MAX_LINEAR_VELOCITY = 1.0  # Maximum linear velocity (adjust as needed)
MAX_ANGULAR_VELOCITY = 1.0  # Maximum angular velocity (adjust as needed)

class MotorController:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate)

    def set_left_motor_speed(self, speed):
        if speed > 0:
            self.ser.write(LEFT_MOTOR_FORWARD.encode())
        else:
            self.ser.write(LEFT_MOTOR_BACKWARD.encode())

    def set_right_motor_speed(self, speed):
        if speed > 0:
            self.ser.write(RIGHT_MOTOR_FORWARD.encode())
        else:
            self.ser.write(RIGHT_MOTOR_BACKWARD.encode())

class MoveRobot:
    def __init__(self):
        self.motor_controller = MotorController(PORT, BAUDRATE)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/gps_data', NavSatFix, self.gps_callback)
        rospy.Subscriber('/compass_data', MagneticField, self.compass_callback)
        rospy.Subscriber('/ultrasonic_data', String, self.ultrasonic_callback)
        self.latitude = None
        self.longitude = None
        self.orientation = None
        self.ultrasonic_distance = None
        self.target_latitude = 30.944114  # Replace with your target latitude
        self.target_longitude = 30.103464  # Replace with your target longitude

    def gps_callback(self, data):
        # Update current GPS coordinates
        self.latitude = data.latitude
        self.longitude = data.longitude

    def compass_callback(self, data):
        # Update current magnetic field data
        self.orientation = data.magnetic_field.z

    def ultrasonic_callback(self, data):
        # Parse and process the ultrasonic sensor data
        ultrasonic_data = data.data.split(',')
        if len(ultrasonic_data) == 3:
            self.ultrasonic_distance = [float(d) for d in ultrasonic_data]

    def move_to_coordinate(self):
        # Move robot to a specified GPS coordinate
        while not rospy.is_shutdown():
            # Calculate distance and angle to the target coordinate
            distance = self.calculate_distance(self.target_latitude, self.target_longitude, self.latitude, self.longitude)
            angle = self.calculate_angle(self.target_latitude, self.target_longitude, self.orientation)

            # Stop moving if the target has been reached
            if distance < 1:
                break

            # Create a Twist message to send movement commands
            twist = Twist()
            twist.linear.x = min(MAX_LINEAR_VELOCITY, distance)
            twist.angular.z = angle * MAX_ANGULAR_VELOCITY

            # Publish movement commands
            self.cmd_vel_pub.publish(twist)

            # Sleep for a short time
            rospy.sleep(0.1)

    def calculate_distance(self, x1, y1, x2, y2):
        # Calculate distance between two GPS coordinates
        dlat = math.radians(x2 - x1)
        dlon = math.radians(y2 - y1)
        a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(math.radians(x1)) * \
            math.cos(math.radians(x2)) * math.sin(dlon / 2) * math.sin(dlon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = 6371 * c  # Earth radius in kilometers
        return distance

    def calculate_angle(self, x, y, orientation):
        # Calculate angle between current orientation and target GPS coordinates
        if x is None or y is None or orientation is None:
            return 0.0
        target_angle = math.atan2(y - self.latitude, x - self.longitude)
        current_angle = math.atan2(math.sin(orientation), math.cos(orientation))
        angle = target_angle - current_angle
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def obstacle_avoidance(self):
        # Perform obstacle avoidance based on ultrasonic sensor data
        while not rospy.is_shutdown():
            # Check if an obstacle is detected
            if self.ultrasonic_distance is not None and \
                    all(d < 0.5 for d in self.ultrasonic_distance):
                # Stop the robot
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(1)

                # Rotate the robot to avoid the obstacle
                twist.angular.z = math.pi / 2
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(2)

                # Move forward to clear the obstacle
                twist.angular.z = 0.0
                twist.linear.x = 0.5
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(2)

                # Stop the robot after clearing the obstacle
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(1)

            rospy.sleep(0.1)

def main():
    rospy.init_node('motor_controller', anonymous=True)
    move_robot = MoveRobot()
    move_robot.move_to_coordinate()
    rospy.spin()

if __name__ == '__main__':
    main()
