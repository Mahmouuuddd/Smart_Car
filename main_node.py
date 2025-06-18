#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import serial
class MoveRobot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('move_robot_node')

        # Create subscribers to get GPS, IMU, and compass data
        rospy.Subscriber('/gps_data', NavSatFix, self.gps_callback)
        rospy.Subscriber('/compass_data', MagneticField, self.compass_callback)

        # Create a publisher to send movement commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize variables to store robot's current pose
        self.latitude = None
        self.longitude = None
        self.orientation = None

        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)  # Update with your serial port and baud rate

    def gps_callback(self, data):
        # Update current GPS coordinates
        self.latitude = data.latitude
        self.longitude = data.longitude

    

    def compass_callback(self, data):
        # Update current magnetic field data
        self.magnetic_field = data.magnetic_field

    def read_serial_data(self):
        # Read data from the serial port
        while not rospy.is_shutdown():
            serial_data = self.serial_port.readline().decode().rstrip()
            self.process_serial_data(serial_data)

    
    def process_serial_data(self, serial_data):
        # Process the received serial data
        data_type, data_values = serial_data.split(':', 1)
        values = data_values.split(',')

        if data_type == 'GPS':
            self.latitude = float(values[0])
            self.longitude = float(values[1])
            self.altitude = float(values[2])
            # Call GPS processing logic or update the necessary variables

        elif data_type == 'Compass':
            self.orientation = float(values[0])
            # Call compass processing logic or update the necessary variables
    
    def move_to_coordinate(self, x, y):
        # Move robot to a specified GPS coordinate
        while not rospy.is_shutdown():
            # Calculate distance and angle to the target coordinate
            distance = self.calculate_distance(x, y, self.latitude, self.longitude)
            angle = self.calculate_angle(x, y)

            # Stop moving if the target has been reached
            if distance < 1:
                break

            # Create a Twist message to send movement commands
            twist = Twist()
            twist.linear.x = min(0.5, distance)
            twist.angular.z = angle

            # Publish movement commands
            self.cmd_vel_pub.publish(twist)

            # Sleep for a short time
            rospy.sleep(0.1)

    def calculate_distance(self, x1, y1, x2, y2):
        # Calculate distance between two GPS coordinates
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def calculate_angle(self, x, y, orientation):
        # Calculate angle between current orientation and target GPS coordinate
        heading = math.atan2(y - self.latitude, x - self.longitude)
        angle = math.degrees(heading - self.orientation)
        return angle

    def run(self):
        rate = rospy.Rate(10)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            #self.obstacle_avoidance()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MoveRobot()
        node.read_serial_data()  # Start reading and processing serial data
        node.move_to_coordinate(47.6062, -122.3321)
        node.run()  # Start obstacle avoidance and movement
    except rospy.ROSInterruptException:
        pass