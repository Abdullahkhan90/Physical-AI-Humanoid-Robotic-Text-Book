---
sidebar_position: 2
title: ROS 2 Nodes, Topics, and Services
---

# ROS 2 Nodes, Topics, and Services

In ROS 2, nodes communicate through a publish-subscribe model using topics, services, and actions. Understanding these communication patterns is essential for building distributed robotic systems.

## Nodes

A node is a process that performs computation, and is the fundamental unit of a ROS 2 program. Multiple nodes are combined together to form a complete robot application. Nodes are designed to be modular, meaning that each node should have a specific task (e.g., controlling a wheel motor, reading sensor data, etc.).

Nodes in ROS 2 are designed to be lightweight, and the ROS graph (the network of nodes) can have many nodes connected at once. A node is a collection of one or more topics, services, and actions.

### Creating a Node

In Python, using the `rclpy` client library:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Topics

Topics enable asynchronous message passing between nodes using a publish-subscribe paradigm. A node publishes messages to a topic, and other nodes subscribe to that topic to receive the messages.

Topics in ROS 2 are type-checked, meaning all messages published to a topic must be of a specific type. Message types are defined using `.msg` files, which are compiled into language-specific code for Python, C++, and other supported languages.

### Publisher Example

```python
import rclpy
from std_msgs.msg import String

def publisher_member_function():
    msg = String()
    msg.data = 'Hello World'
    
    publisher.publish(msg)
```

### Subscriber Example

```python
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Services

Services provide synchronous request-response communication between nodes. A service consists of two parts:
- Request: Sent from the client to the server
- Response: Sent from the server back to the client

Services are used when you need to ensure that a response is returned from a node and you want to wait for that response.

### Service Definition

In a `.srv` file:
```
# Request
string name
---
# Response
bool success
string message
```

### Service Client Example

```python
import rclpy
from example_interfaces.srv import AddTwoInts

def send_request():
    request = AddTwoInts.Request()
    request.a = 1
    request.b = 2
    future = cli.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()
```

## Actions

Actions are another communication pattern, designed for long-running tasks that require feedback. Actions have three components:
- Goal: Request sent to the action server
- Result: Response sent from the action server
- Feedback: Periodic updates sent from the action server during execution

## Communication Patterns Comparison

| Pattern | Type | Use Case |
|---------|------|----------|
| Topics | Asynchronous | Continuous data streams like sensor readings |
| Services | Synchronous | Request-response like calculating a trajectory |
| Actions | Synchronous with feedback | Long-running tasks like navigating to a location |

## Best Practices

1. **Node Design**: Keep nodes focused on a single responsibility
2. **Topic Naming**: Use descriptive and consistent names
3. **Message Types**: Define clear message structures
4. **Error Handling**: Implement appropriate error handling for each communication pattern
5. **Resource Management**: Properly clean up publishers, subscribers, services, and clients

## Summary

ROS 2's communication patterns provide the necessary tools for building distributed robotic systems. Understanding when to use each pattern is crucial for developing robust and maintainable robot applications.

## Exercises

1. Implement a publisher node that publishes velocity commands to a robot
2. Create a subscriber node that listens to sensor data and logs it
3. Design a service that calculates inverse kinematics solutions

In the next section, we'll explore how to bridge Python-based AI agents with ROS controllers using `rclpy`.