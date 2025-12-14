---
sidebar_position: 5
title: Capstone Project - The Autonomous Humanoid
---

# Capstone Project: The Autonomous Humanoid

The capstone project brings together all the concepts learned in previous modules to create an integrated autonomous humanoid robot system. This project involves implementing a simulated humanoid robot that can receive voice commands, plan paths, navigate obstacles, identify objects, and manipulate them based on user instructions.

## Project Overview

The Autonomous Humanoid capstone project demonstrates the integration of:
- **Module 1**: ROS 2 communication framework and controller integration
- **Module 2**: Simulation environment with Gazebo and perception
- **Module 3**: AI-based perception, planning, and navigation
- **Module 4**: Voice command processing and cognitive planning

### Learning Objectives

Upon completion of this project, you will be able to:
- Integrate multiple ROS 2 nodes into a cohesive autonomous system
- Implement voice control with natural language understanding
- Plan navigation paths in complex environments
- Execute manipulation tasks based on high-level commands
- Demonstrate the ability to interact with the environment and perform tasks based on user instructions

## System Architecture

The autonomous humanoid system architecture consists of these interconnected components:

```
[User Voice Command] 
        ↓
[Whisper ASR Module] → [NLP Command Interpreter] → [Cognitive Planner]
        ↓                                             ↓
[Mission Control Node] ←─────────────────────────────┘
        ↓
[Navigation System] → [Path Planner] → [Footstep Planner] → [Motion Controller]
        ↓                                                     ↓
[Perception System] ←────────────────────────────────────────┘
        ↓
[Object Detection] → [Manipulation Planner] → [Arm Controller]
        ↓
[Execution Monitor] → [Safety Checker] → [Feedback Generator]
```

## Implementation Details

### 1. Mission Control Node

The central hub that coordinates all system components:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
from builtin_interfaces.msg import Duration
import queue
import threading
import time

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control')
        
        # Publishers
        self.command_pub = self.create_publisher(String, 'missions/command_input', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.voice_response_pub = self.create_publisher(String, 'voice_response', 10)
        
        # Subscribers
        self.voice_cmd_sub = self.create_subscription(
            String, 'voice_command', self.voice_command_callback, 10)
        self.navigation_status_sub = self.create_subscription(
            String, 'navigation_status', self.nav_status_callback, 10)
        self.perception_status_sub = self.create_subscription(
            String, 'perception_status', self.perception_status_callback, 10)
        self.manipulation_status_sub = self.create_subscription(
            String, 'manipulation_status', self.manipulation_status_callback, 10)
        
        # Internal state
        self.current_task = None
        self.task_queue = queue.Queue()
        self.system_status = {
            'navigation': 'idle',
            'perception': 'idle',
            'manipulation': 'idle',
            'voice_interface': 'active'
        }
        
        # Timer for system monitoring
        self.monitor_timer = self.create_timer(1.0, self.system_monitor)
        
        # Initialize subsystem controllers
        self.nav_controller = NavigationController(self)
        self.perception_controller = PerceptionController(self)
        self.manipulation_controller = ManipulationController(self)
        
        self.get_logger().info("Mission Control Node initialized")
    
    def voice_command_callback(self, msg):
        """Process incoming voice commands"""
        self.get_logger().info(f"Received voice command: {msg.data}")
        
        # Parse and validate command
        parsed_command = self.parse_command(msg.data)
        if parsed_command:
            # Add to task queue based on parsed command
            self.task_queue.put(parsed_command)
            self.execute_next_task()
        else:
            self.respond_to_user("I didn't understand that command. Please try again.")
    
    def parse_command(self, command_text):
        """Parse natural language command into structured task"""
        command_text = command_text.lower().strip()
        
        # Simple command parsing - in a real system, this would be more sophisticated
        if "go to" in command_text and "kitchen" in command_text:
            return {
                'type': 'NAVIGATION',
                'destination': 'kitchen',
                'position': self.get_kitchen_location(),
                'action': 'navigate'
            }
        elif "pick up" in command_text or "grab" in command_text:
            object_name = self.extract_object_name(command_text)
            return {
                'type': 'MANIPULATION',
                'action': 'pick_up',
                'target_object': object_name,
                'destination': None
            }
        elif "bring" in command_text and "me" in command_text:
            object_name = self.extract_object_name(command_text)
            return {
                'type': 'COMPOSITE',
                'tasks': [
                    {'type': 'NAVIGATION', 'destination': 'object_location', 'target_object': object_name},
                    {'type': 'MANIPULATION', 'action': 'pick_up', 'target_object': object_name},
                    {'type': 'NAVIGATION', 'destination': 'user_location', 'action': 'return'}
                ]
            }
        elif "clean" in command_text or "tidy" in command_text:
            return {
                'type': 'COMPOSITE',
                'tasks': [
                    {'type': 'PERCEPTION', 'action': 'scan_area', 'scan_target': 'room'},
                    {'type': 'MANIPULATION', 'action': 'collect_items', 'item_types': self.get_cleanable_items()},
                    {'type': 'NAVIGATION', 'destination': 'disposal_area', 'action': 'deposit_items'}
                ]
            }
        
        # Default: unrecognized command
        self.respond_to_user(f"I don't know how to '{command_text}'. Can you rephrase?")
        return None
    
    def execute_next_task(self):
        """Execute the next task in the queue"""
        if not self.task_queue.empty():
            task = self.task_queue.get()
            
            self.get_logger().info(f"Executing task: {task['type']}")
            
            if task['type'] == 'NAVIGATION':
                self.nav_controller.execute_navigation_task(task)
            elif task['type'] == 'MANIPULATION':
                self.manipulation_controller.execute_manipulation_task(task)
            elif task['type'] == 'PERCEPTION':
                self.perception_controller.execute_perception_task(task)
            elif task['type'] == 'COMPOSITE':
                self.execute_composite_task(task)
    
    def execute_composite_task(self, composite_task):
        """Execute a sequence of related tasks"""
        for subtask in composite_task['tasks']:
            # Execute each subtask and wait for completion before proceeding
            self.wait_for_task_completion(subtask)
    
    def wait_for_task_completion(self, task):
        """Wait for a specific task to complete"""
        # This would involve checking the system status and possibly timeouts
        timeout = time.time() + 30  # 30 second timeout
        while self.system_status[self.get_component_for_task(task)] != 'completed':
            if time.time() > timeout:
                self.get_logger().error(f"Task timed out: {task}")
                return False
            time.sleep(0.1)
        return True
    
    def get_component_for_task(self, task):
        """Get the system component responsible for a task"""
        if task['type'] == 'NAVIGATION':
            return 'navigation'
        elif task['type'] == 'MANIPULATION':
            return 'manipulation'
        elif task['type'] == 'PERCEPTION':
            return 'perception'
        return 'voice_interface'
    
    def nav_status_callback(self, msg):
        """Update navigation status"""
        self.system_status['navigation'] = msg.data
        self.get_logger().info(f"Navigation status: {msg.data}")
    
    def perception_status_callback(self, msg):
        """Update perception status"""
        self.system_status['perception'] = msg.data
        self.get_logger().info(f"Perception status: {msg.data}")
    
    def manipulation_status_callback(self, msg):
        """Update manipulation status"""
        self.system_status['manipulation'] = msg.data
        self.get_logger().info(f"Manipulation status: {msg.data}")
    
    def system_monitor(self):
        """Monitor system health and status"""
        # Check if all subsystems are responsive
        for component, status in self.system_status.items():
            if status == 'error':
                self.get_logger().error(f"{component} system is in error state")
                # Handle error condition
        
        # Log system status periodically
        self.get_logger().debug(f"System status: {self.system_status}")
    
    def respond_to_user(self, response_text):
        """Provide vocal response to user"""
        response_msg = String()
        response_msg.data = response_text
        self.voice_response_pub.publish(response_msg)
        self.get_logger().info(f"Responding to user: {response_text}")
    
    def get_kitchen_location(self):
        """Get pre-defined kitchen location"""
        # In a real system, this would come from a map or semantic navigation system
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 3.0
        pose.pose.position.y = 2.0
        pose.pose.orientation.w = 1.0
        return pose
    
    def extract_object_name(self, command):
        """Extract object name from command (simple implementation)"""
        # This is a simplified extraction - a real implementation would use more sophisticated NLP
        objects = ['cup', 'bottle', 'book', 'toy', 'phone', 'keys', 'glasses', 'hat']
        for obj in objects:
            if obj in command:
                return obj
        return 'unknown_object'
    
    def get_cleanable_items(self):
        """Get list of items that can be cleaned/tidied"""
        return ['cup', 'book', 'toy', 'hat', 'glasses']
```

### 2. Voice Command Processing Component

Implementing the voice processing pipeline:

```python
import openai
import speech_recognition as sr
import pyttsx3
import threading

class VoiceInterface:
    def __init__(self, mission_control_node):
        self.mission_control = mission_control_node
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Initialize text-to-speech
        self.tts_engine = pyttsx3.init()
        self.setup_tts()
        
        # Setup Whisper API
        self.whisper_api_key = self.mission_control.get_parameter_or(
            'whisper_api_key', 'sk-xxxxx').value
        
        # Start listening thread
        self.listening_active = True
        self.listening_thread = threading.Thread(target=self.continuous_listening)
        self.listening_thread.daemon = True
        self.listening_thread.start()
    
    def setup_tts(self):
        """Configure text-to-speech parameters"""
        voices = self.tts_engine.getProperty('voices')
        self.tts_engine.setProperty('rate', 150)  # Speed of speech
        self.tts_engine.setProperty('volume', 0.9)  # Volume level (0.0 to 1.0)
        
        # Try to use female voice if available (often sounds more natural for responses)
        for voice in voices:
            if "female" in voice.name.lower() or "zira" in voice.name.lower():
                self.tts_engine.setProperty('voice', voice.id)
                break
    
    def continuous_listening(self):
        """Continuously listen for voice commands"""
        with self.microphone as source:
            # Adjust for ambient noise
            self.mission_control.get_logger().info("Adjusting for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source)
            self.mission_control.get_logger().info("Listening for commands...")
        
        while self.listening_active:
            try:
                # Listen for audio
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5)
                
                # Process audio with Whisper
                command_text = self.process_audio_with_whisper(audio)
                
                if command_text:
                    # Publish command to ROS system
                    self.publish_command(command_text)
                
            except sr.WaitTimeoutError:
                # Normal - no speech detected within timeout
                continue
            except sr.UnknownValueError:
                # Speech was detected but not understood
                self.mission_control.get_logger().info("Could not understand audio")
                self.speak_response("I didn't catch that. Could you repeat?")
            except Exception as e:
                self.mission_control.get_logger().error(f"Error in voice processing: {e}")
                self.speak_response("I'm having trouble listening right now.")
    
    def process_audio_with_whisper(self, audio_data):
        """Process audio using OpenAI Whisper"""
        try:
            # Save audio to temporary file
            with open("/tmp/temp_audio.wav", "wb") as f:
                f.write(audio_data.get_wav_data())
            
            # Use Whisper API to transcribe
            openai.api_key = self.whisper_api_key
            with open("/tmp/temp_audio.wav", "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
            
            command_text = transcript.text.strip()
            self.mission_control.get_logger().info(f"Transcribed: {command_text}")
            
            return command_text
        except Exception as e:
            self.mission_control.get_logger().error(f"Whisper processing error: {e}")
            return None
    
    def publish_command(self, command_text):
        """Publish recognized command to ROS system"""
        from std_msgs.msg import String
        cmd_msg = String()
        cmd_msg.data = command_text
        self.mission_control.voice_cmd_sub.publish(cmd_msg)
    
    def speak_response(self, text):
        """Speak response to user"""
        self.mission_control.get_logger().info(f"Speaking response: {text}")
        
        def speak_thread():
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        
        # Speak in a separate thread to avoid blocking
        speak_thread = threading.Thread(target=speak_thread)
        speak_thread.daemon = True
        speak_thread.start()
```

### 3. Navigation Controller

Implementing the path planning and navigation system:

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import math

class NavigationController:
    def __init__(self, mission_control_node):
        self.mission_control = mission_control_node
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # Publisher for navigation status
        self.nav_status_pub = self.mission_control.create_publisher(
            String, 'navigation_status', 10)
        
        self.current_destination = None
        self.path_following_active = False
    
    def execute_navigation_task(self, task):
        """Execute a navigation task"""
        self.mission_control.get_logger().info(f"Navigating to: {task['destination']}")
        
        # Update status
        self.update_navigation_status('navigating')
        
        # Create destination pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.mission_control.get_clock().now().to_msg()
        
        if 'position' in task:
            goal_pose.pose.position.x = task['position'].pose.position.x
            goal_pose.pose.position.y = task['position'].pose.position.y
            goal_pose.pose.orientation.w = 1.0
        else:
            # Use predefined locations
            loc = self.get_predefined_location(task['destination'])
            if loc:
                goal_pose.pose.position.x = loc[0]
                goal_pose.pose.position.y = loc[1]
                goal_pose.pose.orientation.w = loc[2]
            else:
                self.mission_control.get_logger().error(f"Unknown destination: {task['destination']}")
                self.update_navigation_status('error')
                return False
        
        # Navigate to goal
        self.navigator.goToPose(goal_pose)
        
        # Monitor navigation progress
        while not self.navigator.isTaskComplete():
            # Check for cancellation
            feedback = self.navigator.getFeedback()
            if feedback:
                # Calculate remaining distance
                current_pose = self.navigator.getRobotPose()
                remaining_distance = self.calculate_distance(
                    current_pose.position, 
                    goal_pose.pose.position
                )
                
                # Log progress
                if remaining_distance < 0.5:
                    self.mission_control.get_logger().info("Approaching destination")
        
        # Check completion result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.mission_control.get_logger().info(f"Successfully reached {task['destination']}")
            self.update_navigation_status('completed')
            return True
        else:
            self.mission_control.get_logger().error(f"Failed to reach {task['destination']}")
            self.update_navigation_status('failed')
            return False
    
    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two points"""
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        return math.sqrt(dx*dx + dy*dy)
    
    def get_predefined_location(self, location_name):
        """Get predefined location coordinates"""
        locations = {
            'kitchen': [3.0, 2.0, 0.0],
            'living_room': [0.0, 0.0, 0.0],
            'bedroom': [-2.0, 1.5, 0.0],
            'office': [1.5, -2.0, 0.0],
            'dining_room': [2.0, -1.0, 0.0]
        }
        return locations.get(location_name.lower())
    
    def update_navigation_status(self, status):
        """Update navigation system status"""
        status_msg = String()
        status_msg.data = status
        self.nav_status_pub.publish(status_msg)
        
        # Also update in mission control
        self.mission_control.system_status['navigation'] = status
```

### 4. Perception System

Implementing the perception and object detection:

```python
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import tensorflow as tf

class PerceptionController:
    def __init__(self, mission_control_node):
        self.mission_control = mission_control_node
        self.cv_bridge = CvBridge()
        
        # Subscriber for camera feed
        self.camera_sub = self.mission_control.create_subscription(
            Image, '/robot_head_camera/image_raw', self.camera_callback, 10)
        
        # Publisher for perception status
        self.perception_status_pub = self.mission_control.create_publisher(
            String, 'perception_status', 10)
        
        # Load object detection model
        self.load_detection_model()
        
        # Current camera image
        self.current_image = None
        self.detection_active = False
        
        # Detected objects cache
        self.detected_objects = []
    
    def load_detection_model(self):
        """Load the object detection model"""
        # In a real system, this would load a trained model like YOLO or SSD
        # For this example, we'll simulate detection
        self.detection_model = None  # Placeholder
        self.mission_control.get_logger().info("Perception model loaded")
    
    def camera_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
            
            # If detection is active, process the image
            if self.detection_active:
                self.process_image_for_detection(cv_image)
        except Exception as e:
            self.mission_control.get_logger().error(f"Camera callback error: {e}")
    
    def process_image_for_detection(self, image):
        """Run object detection on the image"""
        # In a real system, this would run the loaded model
        # For simulation, we'll detect some predefined objects
        detected_objects = self.simulate_object_detection(image)
        
        if detected_objects:
            self.detected_objects = detected_objects
            self.mission_control.get_logger().info(f"Detected objects: {[obj['name'] for obj in detected_objects]}")
    
    def simulate_object_detection(self, image):
        """Simulate object detection for the project"""
        # This is a simulation - in a real system, this would use an actual detection model
        # For demo purposes, we'll return some predefined objects
        height, width = image.shape[:2]
        
        # Predefined objects with positions
        objects = [
            {
                'name': 'red_cup',
                'bbox': [int(width*0.3), int(height*0.4), int(width*0.4), int(height*0.5)],
                'confidence': 0.92,
                'center': (int(width*0.35), int(height*0.45)),
                'position_3d': self.estimate_3d_position([int(width*0.35), int(height*0.45)])
            },
            {
                'name': 'blue_book',
                'bbox': [int(width*0.6), int(height*0.5), int(width*0.7), int(height*0.6)],
                'confidence': 0.87,
                'center': (int(width*0.65), int(height*0.55)),
                'position_3d': self.estimate_3d_position([int(width*0.65), int(height*0.55)])
            }
        ]
        
        return objects
    
    def estimate_3d_position(self, pixel_coords):
        """Estimate 3D position from 2D pixel coordinates"""
        # Rough estimation - in a real system this would use depth information
        # For simulation purposes:
        return {
            'x': (pixel_coords[0] - 320) * 0.002,  # Approximate conversion from pixels to meters
            'y': (240 - pixel_coords[1]) * 0.002,  # Invert Y axis
            'z': 1.0  # Assume object is 1 meter away
        }
    
    def execute_perception_task(self, task):
        """Execute a perception-related task"""
        self.mission_control.get_logger().info(f"Executing perception task: {task['action']}")
        
        self.update_perception_status('processing')
        
        if task['action'] == 'scan_area':
            # Actively scan surroundings for objects
            self.detection_active = True
            
            # Wait for detections (in simulation, we'll just wait a bit)
            import time
            time.sleep(2)
            
            # Return detection results
            detection_results = {
                'objects_found': [obj['name'] for obj in self.detected_objects],
                'object_details': self.detected_objects
            }
            
            self.mission_control.get_logger().info(f"Perception results: {detection_results}")
            self.update_perception_status('completed')
            
            return detection_results
        elif task['action'] == 'locate_object':
            # Find a specific object
            target_obj = task.get('target_object', 'unknown')
            
            # Look for object in cached detections
            for obj in self.detected_objects:
                if target_obj in obj['name']:
                    self.mission_control.get_logger().info(f"Located {target_obj}")
                    return obj
            
            self.mission_control.get_logger().info(f"{target_obj} not found in view")
            return None
    
    def update_perception_status(self, status):
        """Update perception system status"""
        status_msg = String()
        status_msg.data = status
        self.perception_status_pub.publish(status_msg)
        
        # Update mission control
        self.mission_control.system_status['perception'] = status
```

### 5. Manipulation Controller

Implementing the manipulation system:

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

class ManipulationController:
    def __init__(self, mission_control_node):
        self.mission_control = mission_control_node
        
        # Publishers for manipulation commands
        self.arm_controller_pub = self.mission_control.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_controller_pub = self.mission_control.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # Publisher for manipulation status
        self.manip_status_pub = self.mission_control.create_publisher(
            String, 'manipulation_status', 10)
        
        # Subscriber for joint states
        self.joint_state_sub = self.mission_control.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Current joint states
        self.current_joints = {}
        
        # Robot kinematics parameters (simplified)
        self.link_lengths = {
            'upper_arm': 0.3,
            'forearm': 0.25
        }
    
    def joint_state_callback(self, msg):
        """Update current joint states"""
        for name, position in zip(msg.name, msg.position):
            self.current_joints[name] = position
    
    def execute_manipulation_task(self, task):
        """Execute a manipulation task"""
        self.mission_control.get_logger().info(f"Executing manipulation: {task['action']}")
        
        self.update_manipulation_status('executing')
        
        if task['action'] == 'pick_up':
            # Locate the target object
            target_obj = task['target_object']
            
            # In a real system, this would get the 3D position from perception
            # For simulation, we'll use a default position
            target_position = self.get_default_object_position(target_obj)
            
            # Plan and execute pick motion
            success = self.execute_pick_motion(target_position)
            
            if success:
                self.mission_control.get_logger().info(f"Successfully picked up {target_obj}")
                self.update_manipulation_status('completed')
                return True
            else:
                self.mission_control.get_logger().error(f"Failed to pick up {target_obj}")
                self.update_manipulation_status('failed')
                return False
        
        elif task['action'] == 'place':
            # Place object at specified location
            place_position = task.get('destination', self.get_default_place_position())
            success = self.execute_place_motion(place_position)
            
            if success:
                self.mission_control.get_logger().info("Object placed successfully")
                self.update_manipulation_status('completed')
                return True
            else:
                self.mission_control.get_logger().error("Failed to place object")
                self.update_manipulation_status('failed')
                return False
        
        return False
    
    def get_default_object_position(self, obj_name):
        """Get default position for a known object"""
        # In a real system, this would come from perception
        default_positions = {
            'cup': {'x': 0.5, 'y': 0.2, 'z': 0.8},
            'book': {'x': 0.6, 'y': -0.1, 'z': 0.8},
            'toy': {'x': 0.4, 'y': 0.0, 'z': 0.8}
        }
        return default_positions.get(obj_name, {'x': 0.5, 'y': 0.0, 'z': 0.8})
    
    def get_default_place_position(self):
        """Get default placement position"""
        return {'x': 0.0, 'y': -0.5, 'z': 0.8}  # Place on a surface in front
    
    def execute_pick_motion(self, target_position):
        """Execute motion sequence to pick up an object"""
        try:
            # Calculate inverse kinematics
            joint_angles = self.inverse_kinematics(
                target_position['x'], 
                target_position['y'], 
                target_position['z']
            )
            
            if joint_angles is None:
                self.mission_control.get_logger().error("IK solution not found")
                return False
            
            # Open gripper
            self.open_gripper()
            
            # Move to position above object
            approach_pos = {
                'x': target_position['x'],
                'y': target_position['y'],
                'z': target_position['z'] + 0.1  # 10cm above object
            }
            approach_angles = self.inverse_kinematics(
                approach_pos['x'], approach_pos['y'], approach_pos['z'])
            
            if approach_angles:
                self.move_arm_to_joints(approach_angles)
            
            # Move down to object
            self.move_arm_to_joints(joint_angles)
            
            # Close gripper to grasp object
            self.close_gripper()
            
            # Lift object slightly
            lift_pos = {
                'x': target_position['x'],
                'y': target_position['y'],
                'z': target_position['z'] + 0.15
            }
            lift_angles = self.inverse_kinematics(
                lift_pos['x'], lift_pos['y'], lift_pos['z'])
            
            if lift_angles:
                self.move_arm_to_joints(lift_angles)
            
            return True
            
        except Exception as e:
            self.mission_control.get_logger().error(f"Error in pick motion: {e}")
            return False
    
    def execute_place_motion(self, place_position):
        """Execute motion sequence to place an object"""
        try:
            # Calculate inverse kinematics for placement position
            joint_angles = self.inverse_kinematics(
                place_position['x'], 
                place_position['y'], 
                place_position['z']
            )
            
            if joint_angles is None:
                self.mission_control.get_logger().error("IK solution not found for placement")
                return False
            
            # Move to placement position
            self.move_arm_to_joints(joint_angles)
            
            # Open gripper to release object
            self.open_gripper()
            
            # Move arm away from placed object
            retract_pos = {
                'x': place_position['x'],
                'y': place_position['y'] - 0.1,  # Move backward
                'z': place_position['z'] + 0.1   # Move up slightly
            }
            retract_angles = self.inverse_kinematics(
                retract_pos['x'], retract_pos['y'], retract_pos['z'])
            
            if retract_angles:
                self.move_arm_to_joints(retract_angles)
            
            return True
            
        except Exception as e:
            self.mission_control.get_logger().error(f"Error in place motion: {e}")
            return False
    
    def inverse_kinematics(self, x, y, z):
        """Simple 2D inverse kinematics for planar arm"""
        # Simplified 2-joint inverse kinematics
        # This is a greatly simplified version - real IK would be more complex
        try:
            # Calculate distance from origin to target
            dist_sq = x*x + y*y
            dist = math.sqrt(dist_sq)
            
            # Check reachability
            arm_length = self.link_lengths['upper_arm'] + self.link_lengths['forearm']
            if dist > arm_length:
                return None  # Target not reachable
            
            # Calculate joint angles
            upper_length = self.link_lengths['upper_arm']
            forearm_length = self.link_lengths['forearm']
            
            # Cosine law to find elbow angle
            cos_angle_elbow = (upper_length*upper_length + forearm_length*forearm_length - dist_sq) / (2 * upper_length * forearm_length)
            angle_elbow = math.acos(cos_angle_elbow)
            
            # Calculate shoulder angle
            k1 = upper_length + forearm_length * math.cos(angle_elbow)
            k2 = forearm_length * math.sin(angle_elbow)
            
            angle_shoulder = math.atan2(y, x) - math.atan2(k2, k1)
            
            # Return joint angles (shoulder, elbow)
            return [angle_shoulder, math.pi - angle_elbow]
            
        except Exception as e:
            self.mission_control.get_logger().error(f"IK calculation error: {e}")
            return None
    
    def move_arm_to_joints(self, joint_angles):
        """Move arm to specified joint angles"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['shoulder_joint', 'elbow_joint']  # Simplified
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 2  # 2 seconds to reach position
        
        traj_msg.points.append(point)
        
        self.arm_controller_pub.publish(traj_msg)
        
        # Wait for execution to complete (simplified)
        import time
        time.sleep(2.5)
    
    def open_gripper(self):
        """Open the gripper"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.05]  # Open position
        point.time_from_start.sec = 1
        
        traj_msg.points.append(point)
        self.gripper_controller_pub.publish(traj_msg)
        
        import time
        time.sleep(1.2)
    
    def close_gripper(self):
        """Close the gripper to grasp"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.01]  # Closed position
        point.time_from_start.sec = 1
        
        traj_msg.points.append(point)
        self.gripper_controller_pub.publish(traj_msg)
        
        import time
        time.sleep(1.2)
    
    def update_manipulation_status(self, status):
        """Update manipulation system status"""
        status_msg = String()
        status_msg.data = status
        self.manip_status_pub.publish(status_msg)
        
        # Update mission control
        self.mission_control.system_status['manipulation'] = status
```

## Project Integration and Testing

### Main Project Node

Combine all components in the main project node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():
    rclpy.init()
    
    # Create mission control node
    mission_control = MissionControlNode()
    
    # Initialize subsystems
    voice_interface = VoiceInterface(mission_control)
    nav_controller = NavigationController(mission_control)
    perception_controller = PerceptionController(mission_control)
    manipulation_controller = ManipulationController(mission_control)
    
    # Start the system
    mission_control.get_logger().info("Autonomous Humanoid System Online")
    mission_control.respond_to_user("System is ready. Please give me a command.")
    
    try:
        rclpy.spin(mission_control)
    except KeyboardInterrupt:
        pass
    finally:
        mission_control.listening_active = False
        mission_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing Scenarios

### Basic Interaction Test

1. **Voice Command**: "Robot, go to the kitchen"
   - Validates: Voice recognition → Command parsing → Navigation
   - Expected: Robot navigates to kitchen area

2. **Voice Command**: "Pick up the red cup"
   - Validates: Perceptio→ Object recognition → Manipulation
   - Expected: Robot locates and grasps the cup

3. **Voice Command**: "Bring me the cup"
   - Validates: Composite task execution
   - Expected: Robot retrieves cup and brings it to user

### Advanced Scenario Test

**Scenario**: "Please clean up the living room by putting books and toys on the shelf"

This complex command should trigger:
1. Navigation to living room
2. Area scanning to identify books and toys
3. Sequential picking up of identified objects
4. Navigation to shelf location
5. Placing objects on shelf appropriately
6. Returning to home position

## Safety Considerations

### Emergency Stop Integration

```python
class SafetyController:
    def __init__(self, mission_control_node):
        self.mission_control = mission_control_node
        self.emergency_stop_sub = self.mission_control.create_subscription(
            String, 'emergency_stop', self.emergency_stop_callback, 10)
        self.system_armed = True
        
    def emergency_stop_callback(self, msg):
        """Handle emergency stop commands"""
        if msg.data == 'EMERGENCY_STOP':
            self.trigger_emergency_stop()
    
    def trigger_emergency_stop(self):
        """Safely halt all robot motion"""
        self.mission_control.get_logger().error("EMERGENCY STOP TRIGGERED")
        self.system_armed = False
        
        # Stop all motion
        self.stop_all_motion()
        
        # Move to safe position if possible
        self.move_to_safe_position()
        
    def check_safety_conditions(self):
        """Check if it's safe to continue operations"""
        # Check for obstacles
        # Check robot health
        # Check environment safety
        return self.system_armed
```

## Performance Metrics

### Evaluation Criteria

1. **Task Completion Rate**: Percentage of commands successfully executed
2. **Response Time**: Time from command receipt to start of execution
3. **Navigation Accuracy**: Precision of reaching goal locations
4. **Object Manipulation Success**: Successful grasping and placement ratio
5. **Voice Recognition Accuracy**: Correct understanding of commands
6. **System Recovery**: Ability to recover from errors

### Logging and Analytics

Implement comprehensive logging to track system performance:

```python
class SystemAnalytics:
    def __init__(self, mission_control_node):
        self.mission_control = mission_control_node
        self.session_start_time = None
        self.commands_processed = 0
        self.success_count = 0
        self.error_count = 0
        
    def log_command(self, command, result):
        """Log command processing results"""
        self.commands_processed += 1
        if result:
            self.success_count += 1
        else:
            self.error_count += 1
        
        # Log to file for analysis
        self.write_to_logfile(command, result)
    
    def get_performance_stats(self):
        """Return current performance metrics"""
        total_time = time.time() - self.session_start_time if self.session_start_time else 0
        
        return {
            'total_commands': self.commands_processed,
            'successful_commands': self.success_count,
            'error_count': self.error_count,
            'success_rate': self.success_count / self.commands_processed if self.commands_processed > 0 else 0,
            'session_duration': total_time,
            'commands_per_minute': self.commands_processed / (total_time / 60) if total_time > 0 else 0
        }
```

## Summary

The Autonomous Humanoid capstone project demonstrates the integration of all the concepts covered in this textbook. It showcases how voice commands can be processed through an AI system to generate robot behaviors, how navigation systems guide the robot through environments, how perception systems identify objects, and how manipulation systems interact with the physical world.

Successfully completing this project requires:
- Integration of multiple ROS 2 nodes and subsystems
- Proper handling of concurrency and asynchronous operations
- Implementation of safety considerations and error recovery
- Design of a user-friendly interaction model
- Validation of the system in simulated and eventually real environments

The project serves as a foundation for further exploration in humanoid robotics, with opportunities for enhancement in areas like machine learning, advanced manipulation, improved navigation, and richer human-robot interaction models.

## Exercises

1. Enhance the voice command processing to handle more complex multi-step instructions
2. Implement a learning mechanism that adapts to user preferences over time
3. Add computer vision object recognition using a trained model (like YOLO)
4. Implement a more sophisticated navigation system with dynamic obstacle avoidance
5. Create a GUI interface for non-verbal interaction with the robot
6. Integrate the system with a mobile base for full mobility
7. Add tactile feedback to improve manipulation success rates