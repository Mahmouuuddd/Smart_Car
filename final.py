#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import tensorflow as tf
import face_recognition
import speech_recognition as sr
from gtts import gTTS
import playsound
import os
import time
import serial
import colorsys
import random
import pyttsx3
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge

class UnifiedRobotAssistant:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('unified_robot_assistant')
        
        # Configuration parameters
        self.setup_configuration()
        
        # Initialize components
        self.setup_voice_engine()
        self.setup_serial_connection()
        self.setup_object_detection()
        self.setup_face_recognition()
        self.setup_ros_components()
        
        # Robot state
        self.current_latitude = None
        self.current_longitude = None
        self.orientation = None
        self.ultrasonic_data = None
        
        rospy.loginfo("Unified Robot Assistant initialized")

    def setup_configuration(self):
        """Load all configuration parameters"""
        # Navigation parameters
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.target_latitude = rospy.get_param('~target_lat', 30.944114)
        self.target_longitude = rospy.get_param('~target_lon', 30.103464)
        self.max_linear_velocity = rospy.get_param('~max_linear', 1.0)
        self.max_angular_velocity = rospy.get_param('~max_angular', 1.0)
        self.obstacle_threshold = rospy.get_param('~obstacle_thresh', 0.5)
        
        # Voice command parameters
        self.detection_commands = [
            "what do you see", "second service", "what is it", 
            "tell me what do you see", "detect it", "see it",
            "tell", "tell me", "detect", "what's in front of me",
            "front", "in front"
        ]
        self.help_commands = [
            "help", "help me", "hey siri", "hey assistant",
            "hello", "hello assistant"
        ]
        self.confirmation_commands = ["yes", "yeah", "yup"]
        self.exit_commands = ["no", "close", "terminate"]
        
        # Face recognition paths
        self.registered_faces_path = rospy.get_param('~faces_path', 'registered/')
        
        # Object detection model
        self.model_path = rospy.get_param('~model_path', 'ssdlite_mobilenet_v2.tflite')
        self.classes_path = rospy.get_param('~classes_path', 'coco_classes.txt')

    def setup_voice_engine(self):
        """Initialize text-to-speech engine"""
        self.engine = pyttsx3.init()
        voices = self.engine.getProperty("voices")
        self.engine.setProperty("voice", voices[1].id)
        self.engine.setProperty('rate', 150)
        self.engine.runAndWait()

    def setup_serial_connection(self):
        """Initialize serial connection for motor control"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo(f"Connected to serial port {self.port}")
        except serial.SerialException as e:
            rospy.logerr(f"Serial connection failed: {e}")
            rospy.signal_shutdown("Serial connection failed")

    def setup_object_detection(self):
        """Load object detection model and classes"""
        # Load TFLite model
        self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()
        
        # Get input/output details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        # Load class names and colors
        self.class_names = self.read_classes(self.classes_path)
        self.colors = self.generate_colors(self.class_names)
        
        # Initialize camera
        self.camera = cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def setup_face_recognition(self):
        """Initialize face recognition system"""
        # Load known faces
        self.known_faces = []
        self.known_names = []
        
        for name in os.listdir(self.registered_faces_path):
            images_mask = f'{self.registered_faces_path}{name}/*.jpg'
            images_paths = glob.glob(images_mask)
            for img_path in images_paths:
                encoding = self.get_face_encoding(img_path)
                if encoding is not None:
                    self.known_faces.append(encoding)
                    self.known_names.append(name)
        
        # Load face detection classifier
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

    def setup_ros_components(self):
        """Initialize ROS publishers and subscribers"""
        # Subscribers
        rospy.Subscriber('/gps_data', NavSatFix, self.gps_callback)
        rospy.Subscriber('/imu_data', Imu, self.imu_callback)
        rospy.Subscriber('/compass_data', MagneticField, self.compass_callback)
        rospy.Subscriber('/ultrasonic_data', String, self.ultrasonic_callback)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.object_detection_pub = rospy.Publisher('/detected_objects', String, queue_size=1)
        self.face_recognition_pub = rospy.Publisher('/recognized_faces', String, queue_size=1)

    # --------------------- Core Functionality ---------------------

    def run(self):
        """Main execution loop"""
        rospy.loginfo("Starting Unified Robot Assistant")
        self.speech("Hello, I am your robot assistant. How can I help you today?")
        
        try:
            while not rospy.is_shutdown():
                command = self.listen_for_command()
                self.process_command(command)
                
        except rospy.ROSInterruptException:
            self.cleanup()
    
    def process_command(self, command):
        """Process voice commands"""
        if not command:
            return
            
        # Object detection commands
        if any(cmd in command for cmd in self.detection_commands):
            self.speech("Okay sir, I will detect objects for you")
            self.detect_objects()
            
        # Help commands
        elif any(cmd in command for cmd in self.help_commands):
            self.speech("Hello sir, I can help you with object detection or navigation. What would you like?")
            response = self.listen_for_command()
            
            if any(cmd in response for cmd in self.confirmation_commands):
                self.speech("What would you like me to do? Detect objects or navigate somewhere?")
                
        # Navigation commands
        elif "navigate" in command or "go to" in command:
            self.speech("Please specify the target location")
            # Here you would implement location parsing from voice command
            # For now we'll use the default target
            self.navigate_to_target()
            
        # Face recognition
        elif "who is this" in command or "recognize" in command:
            self.recognize_faces()
            
        # Exit commands
        elif any(cmd in command for cmd in self.exit_commands):
            self.speech("Goodbye sir. Shutting down now.")
            rospy.signal_shutdown("User requested shutdown")

    # --------------------- Navigation Functions ---------------------

    def navigate_to_target(self):
        """Navigate to target GPS coordinates with obstacle avoidance"""
        while not rospy.is_shutdown():
            if self.obstacle_detected():
                self.perform_obstacle_avoidance()
                continue
                
            if None in [self.current_latitude, self.current_longitude, self.orientation]:
                rospy.logwarn("Waiting for sensor data...")
                rospy.sleep(1)
                continue
                
            distance = self.haversine_distance(
                self.current_latitude, self.current_longitude,
                self.target_latitude, self.target_longitude
            )
            
            if distance < 1.0:  # 1 meter threshold
                self.speech("Target reached!")
                break
                
            angle = self.calculate_target_angle()
            self.move_robot(distance, angle)
            
            rospy.sleep(0.1)

    def move_robot(self, distance, angle):
        """Calculate and execute movement commands"""
        linear_vel = min(self.max_linear_velocity, distance/5)
        angular_vel = angle * self.max_angular_velocity
        
        left_speed = linear_vel - angular_vel
        right_speed = linear_vel + angular_vel
        
        # Normalize speeds
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > self.max_linear_velocity:
            left_speed *= self.max_linear_velocity / max_speed
            right_speed *= self.max_linear_velocity / max_speed
            
        # Send motor commands
        self.set_motor_speeds(left_speed, right_speed)
        
        # Publish cmd_vel for other nodes
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

    # --------------------- Object Detection Functions ---------------------

    def detect_objects(self):
        """Perform object detection and announce results"""
        ret, frame = self.camera.read()
        if not ret:
            self.speech("Failed to capture image")
            return
            
        image_data = self.preprocess_image_for_tflite(frame)
        scores, boxes, classes = self.run_detection(image_data, range(1, 90))
        
        # Draw boxes and get detected objects
        result, objects = self.draw_boxes(frame, scores, boxes, classes)
        
        # Show detection result briefly
        cv2.imshow("Object Detection", frame)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
        
        # Announce results
        if not objects:
            self.speech("I don't see any objects")
        elif len(objects) == 1:
            self.speech(f"I see a {objects[0]}")
        else:
            objects_str = ", ".join(objects[:-1]) + " and " + objects[-1]
            self.speech(f"I see {objects_str}")
            
        # Publish detection results
        self.object_detection_pub.publish(String(", ".join(objects)))

    # --------------------- Face Recognition Functions ---------------------

    def recognize_faces(self):
        """Perform face recognition and announce results"""
        ret, frame = self.camera.read()
        if not ret:
            self.speech("Failed to capture image")
            return
            
        # Convert to RGB (face_recognition uses RGB)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Find all face locations
        face_locations = face_recognition.face_locations(rgb_frame)
        
        if not face_locations:
            self.speech("I don't see any faces")
            return
            
        # Recognize each face
        recognized_names = []
        for face_location in face_locations:
            encoding = face_recognition.face_encodings(rgb_frame, [face_location])[0]
            matches = face_recognition.compare_faces(self.known_faces, encoding, tolerance=0.6)
            
            name = "Unknown"
            if True in matches:
                name = self.known_names[matches.index(True)]
                
            recognized_names.append(name)
            
            # Draw rectangle around the face
            top, right, bottom, left = face_location
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(frame, name, (left, top-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Show result briefly
        cv2.imshow("Face Recognition", frame)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
        
        # Announce results
        if "Unknown" in recognized_names:
            self.register_new_face(frame)
        else:
            names_str = ", ".join(recognized_names)
            self.speech(f"I see {names_str}")
            
        # Publish recognition results
        self.face_recognition_pub.publish(String(", ".join(recognized_names)))

    def register_new_face(self, frame):
        """Register a new face that wasn't recognized"""
        self.speech("I don't recognize you. Let me register you as a new user.")
        
        # Get name from voice
        self.speech("Please tell me your name")
        name = self.listen_for_command()
        
        if not name:
            self.speech("Sorry, I didn't catch your name. Please try again later.")
            return
            
        # Create directory for new user
        user_dir = os.path.join(self.registered_faces_path, name)
        os.makedirs(user_dir, exist_ok=True)
        
        # Capture multiple images of the face
        self.speech("Please look at the camera while I take some photos")
        
        count = 0
        while count < 5:  # Capture 5 images
            ret, frame = self.camera.read()
            if not ret:
                continue
                
            face = self.face_extractor(frame)
            if face is not None:
                face = cv2.resize(face, (600, 600))
                img_path = os.path.join(user_dir, f"{name}_{count}.jpg")
                cv2.imwrite(img_path, face)
                count += 1
                time.sleep(0.5)
                
        # Add new face to known faces
        encoding = self.get_face_encoding(img_path)
        if encoding is not None:
            self.known_faces.append(encoding)
            self.known_names.append(name)
            self.speech(f"Thank you {name}. You are now registered in my system.")
        else:
            self.speech("Sorry, I couldn't register your face. Please try again.")

    # --------------------- Helper Functions ---------------------
    
    # (Include all the helper functions from your original code here:
    # read_classes, generate_colors, preprocess_image, preprocess_image_for_tflite,
    # non_max_suppression, draw_boxes, haversine_distance, calculate_target_angle,
    # obstacle_detected, perform_obstacle_avoidance, set_motor_speeds, etc.)
    
    # Also include the voice-related functions:
    # speech, listen_for_command, get_face_encoding, face_extractor, etc.

    def cleanup(self):
        """Clean up resources"""
        self.ser.close()
        self.camera.release()
        cv2.destroyAllWindows()
        self.speech("Shutting down all systems. Goodbye.")

if __name__ == '__main__':
    try:
        assistant = UnifiedRobotAssistant()
        assistant.run()
    except rospy.ROSInterruptException:
        pass