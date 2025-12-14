---
sidebar_position: 4
title: URDF for Humanoid Robots
---

# URDF (Unified Robot Description Format) for Humanoids

URDF (Unified Robot Description Format) is an XML format for representing a robot model. URDF is used extensively in ROS to describe robots in terms of their joints, links, visual and collision properties, and more. When designing humanoid robots, URDF plays a crucial role in specifying the physical structure and kinematic properties.

## What is URDF?

URDF stands for Unified Robot Description Format. It is an XML file format used to describe robots in ROS. A URDF file contains information about:
- Robot's physical structure (links)
- How parts are connected (joints)
- Visual appearance (collision and visual properties)
- Kinematic properties (inertial properties)
- Sensors attached to the robot

## URDF Structure for Humanoid Robots

A humanoid robot URDF typically contains multiple limbs (arms, legs), a torso, and a head. Each limb is modeled as a series of links connected by joints.

### Basic URDF Elements

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define the physical parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints define how links connect -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="link1">
    <!-- Link definition similar to base_link -->
  </link>
</robot>
```

## Designing a Humanoid Robot in URDF

### Basic Structure

A humanoid robot typically includes:
- Torso / Pelvis (base link)
- Two legs (hip, knee, ankle, foot)
- Two arms (shoulder, elbow, wrist, hand)
- Neck and head

### Example URDF for a Simple Humanoid

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso / Base Link -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.4"/>  <!-- Width x Depth x Height -->
      </geometry>
      <origin xyz="0 0 0.2"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.3 0.4"/>
      </geometry>
      <origin xyz="0 0 0.2"/>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.3" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Joint connecting torso and head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="1" velocity="2"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <box size="0.1 0.15 0.15"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.15 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting torso and shoulder -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.1 0.2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="3"/>
  </joint>

  <!-- Right arm, legs, and other body parts would follow similar pattern -->
</robot>
```

## Advanced URDF Features for Humanoids

### Transmission Elements

To connect the robot to a control system, transmission elements are added:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo-Specific Elements

For simulation in Gazebo, additional elements are often needed:

```xml
<gazebo reference="torso">
  <material>Gazebo/Gray</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

## URDF Tools and Verification

### Using check_urdf

After creating a URDF file, verify its validity:

```bash
check_urdf path/to/robot.urdf
```

### Using urdf_to_graphiz

Visualize the robot structure:

```bash
urdf_to_graphviz path/to/robot.urdf
```

## Best Practices for Humanoid URDF

1. **Use proper units**: All dimensions in meters, mass in kilograms
2. **Realistic inertial properties**: Accurate mass and inertia values improve simulation
3. **Joint limits**: Define appropriate limits for each joint
4. **Collision vs. Visual**: Use simple shapes for collision and detailed for visuals
5. **Naming convention**: Use consistent naming across all links and joints
6. **Kinematic chain**: Ensure proper parent-child relationships
7. **Mounting points**: Define attachment points for accessories

## Creating URDF with Python

While URDF is an XML format, it can be programmatically generated using Python:

```python
import xml.etree.ElementTree as ET

def create_simple_link(name, mass, geometry_type="box", size=[0.1, 0.1, 0.1]):
    link = ET.Element("link", name=name)
    
    # Visual element
    visual = ET.SubElement(link, "visual")
    vis_geom = ET.SubElement(visual, "geometry")
    ET.SubElement(vis_geom, geometry_type, size=" ".join(map(str, size)))
    
    # Collision element
    collision = ET.SubElement(link, "collision")
    coll_geom = ET.SubElement(collision, "geometry")
    ET.SubElement(coll_geom, geometry_type, size=" ".join(map(str, size)))
    
    # Inertial element
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value=str(mass))
    ET.SubElement(inertial, "inertia", 
                  ixx="0.01", ixy="0", ixz="0", 
                  iyy="0.01", iyz="0", izz="0.01")
    
    return link

# Example usage
robot = ET.Element("robot", name="humanoid")
torso = create_simple_link("torso", 5.0, "box", [0.2, 0.3, 0.4])
robot.append(torso)
```

## Integration with ROS

Once the URDF is created, it can be used in ROS nodes:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math

class URDFRobotNode(Node):
    def __init__(self):
        super().__init__('urdf_robot_node')
        
        # Initialize joint state publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Initialize TF broadcaster
        self.br = TransformBroadcaster(self)
        
        # Timer for publishing state
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Sample joint positions
        self.joint_names = ['left_shoulder_joint', 'neck_joint']
        self.joint_positions = [0.0, 0.0]
    
    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        self.joint_pub.publish(msg)
```

## Summary

URDF is a critical component in modeling humanoid robots for ROS. Properly constructed URDF files enable accurate simulation, kinematic analysis, and motion planning. Understanding how to effectively describe complex human-like structures in URDF is essential for developing humanoid robots in ROS.

## Exercises

1. Create a URDF file for a simplified humanoid with at least 6 degrees of freedom
2. Add transmission elements to your URDF for position control
3. Visualize your URDF using RViz
4. Simulate your humanoid in Gazebo