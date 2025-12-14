---
sidebar_position: 3
title: Integrating Python Agents with ROS Controllers
---

# Integrating Python Agents with ROS Controllers using rclpy

One of the key strengths of ROS 2 is its support for multiple programming languages. Python, being the language of choice for many AI and machine learning frameworks, becomes an ideal bridge between AI agents and robot control systems. The `rclpy` client library allows us to interface Python AI agents with ROS controllers.

## Understanding rclpy

`rclpy` is the Python client library for ROS 2, providing bindings for the ROS client library (rcl). It abstracts the complexity of ROS 2's underlying communication systems and provides a simple interface for creating nodes, publishers, subscribers, services, and actions.

### Core Concepts

- **Node**: The basic unit of computation in ROS 2
- **Executor**: Manages the execution of callbacks for a node
- **Context**: Provides lifecycle management for a set of ROS entities

### Basic rclpy Node Structure

```python
import rclpy
from rclpy.node import Node

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        # Initialize AI agent and ROS interfaces
        self.init_ai_agent()
        self.init_ros_interfaces()
    
    def init_ai_agent(self):
        # Initialize your AI agent here
        pass
    
    def init_ros_interfaces(self):
        # Create publishers, subscribers, services, etc.
        pass

def main(args=None):
    rclpy.init(args=args)
    
    ai_agent_node = AIAgentNode()
    
    try:
        rclpy.spin(ai_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bridging AI Agents to ROS Controllers

Consider an AI agent trained to recognize objects in camera images. The agent needs to:
1. Receive image data from a ROS camera topic
2. Process the image to detect objects
3. Publish the results to a ROS topic for the controller to act upon

### AI Agent Node Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

class ObjectDetectionAgent(Node):
    def __init__(self):
        super().__init__('object_detection_agent')
        
        # Create subscription to camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Create publisher for detection results
        self.publisher = self.create_publisher(
            String,
            '/object_detection/results',
            10)
        
        # Initialize your AI model here
        self.model = self.initialize_ai_model()
    
    def initialize_ai_model(self):
        # Load your pre-trained AI model
        # Example: return torch.load('object_detection_model.pth')
        return None
    
    def image_callback(self, msg):
        # Convert ROS Image message to format suitable for AI model
        image_array = self.ros_image_to_numpy(msg)
        
        # Process image with AI model
        detection_results = self.process_with_ai_model(image_array)
        
        # Publish results
        self.publish_results(detection_results)
    
    def ros_image_to_numpy(self, img_msg):
        # Convert ROS Image message to numpy array
        # Implementation depends on encoding type
        pass
    
    def process_with_ai_model(self, image):
        # Apply AI model to image
        # This is where your AI processing happens
        return "Sample detection results"
    
    def publish_results(self, results):
        msg = String()
        msg.data = str(results)
        self.publisher.publish(msg)
```

## Real-time Control Systems

For real-time control systems, timing is critical. The AI agent might need to:
- Respond to sensor data within certain time constraints
- Coordinate with other control systems
- Handle asynchronous events efficiently

### Using Timers in rclpy

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ControlAgent(Node):
    def __init__(self):
        super().__init__('control_agent')
        
        # Create publisher for control commands
        self.publisher = self.create_publisher(Float64, '/motor/cmd_vel', 10)
        
        # Create a timer to execute control loop periodically
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        # Initialize control state
        self.current_speed = 0.0
    
    def control_loop(self):
        # Get AI agent's decision based on current state
        speed_command = self.ai_decision_logic()
        
        # Publish control command
        msg = Float64()
        msg.data = speed_command
        self.publisher.publish(msg)
        
        self.current_speed = speed_command
    
    def ai_decision_logic(self):
        # Your AI logic to determine next action
        # Could involve state evaluation, neural network inference, etc.
        return self.current_speed + 0.1  # Simple example
```

## Integrating with ROS Controllers

ROS controllers are typically implemented as nodes that handle low-level control tasks. To integrate AI agents with these controllers:

1. **Input**: Subscribe to sensor data published by hardware interfaces or other nodes
2. **Processing**: Apply AI algorithms to interpret sensor data and decide on actions
3. **Output**: Publish commands to controller topics or call services

### Example: Integration with Joint State Controller

```python
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

class AIBasedController(Node):
    def __init__(self):
        super().__init__('ai_based_controller')
        
        qos_profile = QoSProfile(depth=10)
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile)
        
        # Publisher for joint trajectory commands
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectoryControllerState,
            '/position_controller/commands',
            qos_profile)
        
        # Store joint state data
        self.joint_positions = {}
        self.joint_velocities = {}
    
    def joint_state_callback(self, msg):
        # Update internal joint state
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self.joint_positions[name] = pos
            self.joint_velocities[name] = vel
        
        # Apply AI-based control logic
        if self.should_update_control():
            self.apply_control_command()
    
    def should_update_control(self):
        # Define conditions under which to update control
        return True  # Simplified example
    
    def apply_control_command(self):
        # Use AI to determine control action based on current state
        command = self.ai_control_algorithm(self.joint_positions)
        self.publish_control_command(command)
    
    def ai_control_algorithm(self, joint_pos):
        # Your AI algorithm implementation
        return []  # Placeholder for actual command
    
    def publish_control_command(self, command):
        # Publish command to ROS controller
        pass
```

## Performance Considerations

When bridging AI agents with ROS controllers:

1. **Latency**: Minimize processing delays to ensure real-time performance
2. **Frequency**: Match the update frequency to the requirements of the control system
3. **Resource Management**: Efficiently manage computational resources
4. **Thread Safety**: Ensure safe access to shared data between ROS callbacks and AI processing

## Summary

The integration of Python AI agents with ROS controllers through `rclpy` enables sophisticated robot behaviors by combining the power of AI with ROS's flexible communication architecture. This bridge allows us to leverage state-of-the-art machine learning models in robotic control systems.

## Exercises

1. Create a simple AI agent that subscribes to sensor data and publishes commands
2. Implement a neural network inference pipeline within an rclpy node
3. Design a reinforcement learning agent that interacts with a simulated robot