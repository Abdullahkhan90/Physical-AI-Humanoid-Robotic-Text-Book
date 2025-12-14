---
sidebar_position: 4
title: Nav2 for Path Planning in Humanoid Movement
---

# Nav2: Path Planning for Bipedal Humanoid Movement

Navigation 2 (Nav2) is the next-generation navigation system for ROS 2, designed to provide path planning and navigation capabilities for mobile robots. This section focuses on adapting Nav2 for the specific challenges of bipedal humanoid movement, which requires specialized approaches to path planning and locomotion compared to wheeled robots.

## Introduction to Nav2

Nav2 is a complete rewrite of the ROS 1 navigation stack, designed specifically for ROS 2. It provides:
- Map management and localization utilities
- Path planning for static and dynamic obstacles
- Path following with adaptive control
- Behavior trees for complex navigation tasks
- Modular design allowing custom components

## Challenges in Humanoid Navigation

Bipedal humanoid robots face unique navigation challenges compared to wheeled robots:

### Kinematic Differences
- Bipedal gait requires coordinated leg movements
- Center of mass considerations for stability
- Different turning mechanics compared to differential drive
- Need for footstep planning

### Terrain Requirements
- Negotiating stairs and obstacles requiring stepping
- Dynamic balance on uneven surfaces
- Need to find stable footholds
- Maintaining balance during navigation

### Safety Considerations
- Fall prevention mechanisms
- Stability margins during walking
- Reaction to unexpected disturbances
- Controlled stopping for balance recovery

## Nav2 Architecture for Humanoids

### Core Components

Nav2 consists of several core components that can be adapted for humanoid navigation:

```
[Map Server] -> [Localizer] -> [Path Planner] -> [Controller] -> [Robot]
      ↑              ↑             ↑            ↑
[Static Map] <- [Sensor Data] <- [Costmaps] <- [Path Tracker]
```

### Humanoid-Specific Modifications

The standard Nav2 pipeline requires adaptations for humanoid navigation:

1. **Custom Costmaps**: Account for bipedal-specific constraints like step height limits
2. **Footstep Planners**: Generate stable footstep sequences instead of continuous paths
3. **Stability Constraints**: Ensure center of mass remains within support polygon
4. **Balance Controllers**: Maintain balance during movement execution

## Footstep Planning

### Introduction to Footstep Planning

Unlike wheeled robots that can follow smooth paths, bipedal robots require discrete footstep planning that ensures stability at each step. The footstep planner generates a sequence of foot placements that:
- Maintain the robot's balance
- Reach the goal location
- Avoid obstacles
- Respect terrain constraints

### Basic Footstep Representation

```cpp
struct FootStep {
    // 3D position of the foot placement
    double x, y, z;
    
    // Orientation of the foot
    double roll, pitch, yaw;
    
    // Support polygon (for balance)
    std::vector<Eigen::Vector3d> support_vertices;
    
    // Timing information
    double duration;  // Expected time to execute this step
    
    // Support type (left foot, right foot, double support)
    enum SupportType { LEFT_FOOT, RIGHT_FOOT, DOUBLE_SUPPORT } support_type;
};
```

### Footstep Planning Algorithm

```python
class HumanoidFootstepPlanner:
    def __init__(self, robot_params):
        self.robot_params = robot_params
        self.step_height_limit = robot_params.step_height_limit
        self.step_width_limit = robot_params.step_width_limit
        self.com_height = robot_params.com_height
        self.foot_size = robot_params.foot_size

    def plan_footsteps(self, start_pose, goal_pose, costmap):
        """
        Plan a sequence of footsteps from start to goal
        """
        footsteps = []
        
        # Initial foot positions based on start pose
        left_foot = self.compute_initial_left_foot(start_pose)
        right_foot = self.compute_initial_right_foot(start_pose)
        
        # Path planning in footsteps instead of continuous space
        while not self.reached_goal(left_foot, right_foot, goal_pose):
            # Determine next foot to move based on gait pattern
            next_support_foot = self.determine_support_foot(left_foot, right_foot)
            
            # Plan next footstep considering stability constraints
            next_footstep = self.plan_next_step(
                next_support_foot, 
                left_foot, 
                right_foot, 
                goal_pose, 
                costmap
            )
            
            # Verify stability of the planned step
            if self.is_stable(next_footstep, next_support_foot):
                footsteps.append(next_footstep)
                
                # Update current feet positions
                if next_footstep.support_type == 'LEFT_FOOT':
                    left_foot = next_footstep
                else:
                    right_foot = next_footstep
            else:
                # Handle unstable situation - replan or adjust
                next_footstep = self.adjust_for_stability(
                    next_footstep, 
                    next_support_foot, 
                    costmap
                )
                footsteps.append(next_footstep)
                
        return footsteps
    
    def is_stable(self, next_step, support_foot):
        """
        Check if the next step maintains stability
        Uses Zero Moment Point (ZMP) or Center of Mass (CoM) criteria
        """
        # Calculate if the projected CoM stays within the support polygon
        com_projection = self.project_com_to_ground(next_step, support_foot)
        support_polygon = self.calculate_support_polygon(support_foot)
        
        return self.point_in_polygon(com_projection, support_polygon)
    
    def plan_next_step(self, current_support_foot, left_foot, right_foot, goal, costmap):
        """
        Plan the next footstep based on goal direction, obstacles, and stability
        """
        # Heuristic for step direction toward goal while avoiding obstacles
        goal_direction = self.calculate_direction_to_goal(goal, current_support_foot)
        
        # Consider terrain traversability
        candidate_steps = self.generate_candidate_steps(current_support_foot, goal_direction)
        
        # Evaluate candidates for safety and efficiency
        best_step = self.select_best_candidate(candidate_steps, costmap, left_foot, right_foot)
        
        return best_step
```

## Nav2 Behavior Trees for Humanoid Navigation

Nav2 uses behavior trees to orchestrate navigation tasks. For humanoids, these need to be adapted:

### Custom Behavior Tree Nodes

```xml
<!-- humanoid_bt.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="nav2_sequence">
            <ClearEntirelyCostmap name="global_clear" service_name="clear_costmap_global"/>
            <GlobalPlanner goal="{goal}" path="{path}"/>
            <ComputePathToPose goal="{goal}" path="{path}"/>
            <FootstepPlanner path="{path}" footsteps="{footsteps}"/>
            
            <!-- Humanoid-specific recovery behaviors -->
            <KeepAliveRecovery/>
            <RebalanceOnPath path="{path}" footsteps="{footsteps}"/>
            
            <Sequence name="follow_waypoints">
                <Fallback name="follow_fallback">
                    <FollowFootsteps path="{path}" footsteps="{footsteps}"/>
                    <RecoveryNode name="balance_recovery">
                        <RecoverBalance/>
                        <TruncPath path="{path}"/>
                        <ComputePathToPose goal="{goal}" path="{path}"/>
                    </RecoveryNode>
                </Fallback>
            </Sequence>
        </Sequence>
    </BehaviorTree>
</root>
```

### Humanoid-Specific Recovery Behaviors

```python
# Recovery behaviors for humanoid robots
class RebalanceOnPath(BehaviorTreeLeaf):
    def __init__(self, name, options):
        super().__init__(name, options)
        self.balance_threshold = options.balance_threshold
        
    def tick(self):
        # Check if robot is approaching instability
        balance_state = self.get_balance_state()
        
        if balance_state < self.balance_threshold:
            # Execute rebalancing maneuver
            self.execute_rebalance()
            return py_trees.common.Status.SUCCESS
        else:
            # Robot is stable, continue normal navigation
            return py_trees.common.Status.FAILURE

class RecoverBalance(BehaviorTreeLeaf):
    def __init__(self, name, options):
        super().__init__(name, options)
        
    def tick(self):
        # Execute balance recovery routine
        # This might include stepping in place, widening stance, etc.
        return py_trees.common.Status.SUCCESS
```

## Humanoid Path Planning Approaches

### Topological Path Planning

For humanoid robots, a topological approach often works better than traditional grid-based planning:

```python
class HumanoidTopologicalPlanner:
    def __init__(self):
        self.topological_map = TopologicalMap()
        self.footstep_planner = HumanoidFootstepPlanner()
        
    def plan_path(self, start, goal):
        # Find topologically distinct paths in configuration space
        topological_nodes = self.find_traversable_regions(start, goal)
        
        # For each topological node, plan footstep sequence
        path_candidates = []
        for node in topological_nodes:
            footstep_sequence = self.footstep_planner.plan_footsteps(start, node, self.costmap)
            if footstep_sequence:
                path_candidates.append({
                    'path': [start, node, goal],
                    'footsteps': footstep_sequence,
                    'cost': self.calculate_path_cost(footstep_sequence)
                })
        
        # Select optimal path based on stability, safety, and efficiency
        return self.select_optimal_path(path_candidates)
```

### Dynamic Path Adjustment

Humanoid robots need to dynamically adjust paths based on balance state:

```python
class DynamicHumanoidPathAdjuster:
    def __init__(self):
        self.original_path = None
        self.current_balance_margins = []
        
    def adjust_path_dynamically(self, robot_state, environment_changes):
        """
        Adjust the path based on current robot state and environment
        """
        if self.should_adjust_path(robot_state, environment_changes):
            # Recalculate safest path based on current conditions
            adjusted_footsteps = self.recalculate_with_constraints(
                robot_state,
                self.original_path,
                environment_changes
            )
            
            # Smoothly transition from current execution to new path
            return self.smooth_transition_to_new_path(adjusted_footsteps)
        else:
            # Continue on original path
            return self.original_path
            
    def should_adjust_path(self, robot_state, env_changes):
        """
        Determine if path adjustment is needed
        """
        # Check balance margins
        com_margin = robot_state.com_to_support_polygon_distance
        if com_margin < self.critical_balance_threshold:
            return True
            
        # Check for new obstacles in path
        if self.obstacles_in_current_path(env_changes):
            return True
            
        # Check energy efficiency (if current path is too costly)
        if self.current_path_energy_excessive(robot_state):
            return True
            
        return False
```

## Configuring Nav2 for Humanoid Navigation

### Costmap Configuration

Humanoid-specific costmap parameters:

```yaml
# humanoid_costmap_params.yaml
global_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: map
    robot_base_frame: base_link
    use_sim_time: false
    rolling_window: false
    
    plugins: ["static_layer", "obstacle_layer", "inflation_layer", "step_height_layer"]
    
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_topic: "map"
      transform_tolerance: 0.3
      max_occ_dist: 0.01
    
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: true
      observation_sources: scan
      scan:
        topic: /laser_scan
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"
        inf_is_valid: false
    
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 1.5  # Increased for safety
      inflation_radius: 0.8     # Larger for humanoid stability
    
    step_height_layer:  # Humanoid-specific layer
      plugin: "nav2_humanoid_layers::StepHeightLayer"
      max_step_height: 0.2    # Maximum traversable step height
      max_down_height: 0.25   # Maximum step-down height

local_costmap:
  ros__parameters:
    update_frequency: 10.0
    publish_frequency: 5.0
    global_frame: odom
    robot_base_frame: base_link
    rolling_window: true
    width: 6
    height: 6
    resolution: 0.05
    transform_tolerance: 0.3
    
    plugins: ["voxel_layer", "inflation_layer"]
    
    voxel_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
      enabled: true
      publish_voxel_map: true
      origin_z: 0.0
      z_resolution: 0.2
      z_voxels: 10
      max_obstacle_height: 2.0
      mark_threshold: 0
      observation_sources: scan
      scan:
        topic: /laser_scan
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"
        inf_is_valid: false
    
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 2.0  # Even higher for local safety
      inflation_radius: 0.5
```

### Controller Configuration

Controllers adapted for humanoid locomotion:

```yaml
# humanoid_controllers.yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.05
    min_theta_velocity_threshold: 0.05
    
    # Humanoid-specific controllers
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    
    controller_plugins: ["HumanoidFollowPath"]
    
    HumanoidFollowPath:
      plugin: "nav2_mppi::HumanoidController"
      # Humanoid-specific parameters
      step_size_max: 0.3        # Maximum step size
      foot_lift_height: 0.05    # Height to lift foot when stepping
      step_timing: 0.8          # Duration of each step
      balance_margin: 0.1       # Safety margin for COM over support polygon
      gait_pattern: "natural_walk"  # Desired walking pattern
```

## Implementation Considerations

### Simulation Integration

Before deploying on a real humanoid, extensive simulation is crucial:

```python
class HumanoidNavigationSimulator:
    def __init__(self):
        self.simulator = GazeboSimulator()
        self.robot_model = self.load_humanoid_model("atlas", "op3", "romeo")
        self.terrain_generator = VariedTerrainGenerator()
        self.nav_system = Nav2System()
        
    def simulate_navigation_scenario(self, scenario_params):
        """
        Simulate navigation in various challenging scenarios
        """
        # Generate varied terrain with stairs, slopes, obstacles
        terrain = self.terrain_generator.generate(scenario_params)
        
        # Set up humanoid model with correct dynamics
        humanoid = self.robot_model.spawn(terrain)
        
        # Define navigation task
        start = self.sample_free_space(terrain)
        goal = self.sample_free_space(terrain, min_distance=5.0)
        
        # Run navigation
        start_time = time.time()
        result = self.nav_system.navigate(humanoid, start, goal)
        end_time = time.time()
        
        # Evaluate performance
        performance = {
            'success': result.success,
            'time': end_time - start_time,
            'energy': self.calculate_energy_consumption(result),
            'stability': self.calculate_stability_metrics(result),
            'path_efficiency': self.calculate_path_efficiency(result, start, goal)
        }
        
        return performance
```

## Safety and Fail-Safe Mechanisms

### Balance Monitoring

Continuous monitoring of balance state:

```python
class BalanceMonitor:
    def __init__(self):
        self.com_subscriber = rospy.Subscriber('/robot/com_state', ComState, self.com_callback)
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.balance_threshold = 0.05  # meters from support polygon
        
    def com_callback(self, data):
        self.current_com = [data.x, data.y, data.z]
        
    def imu_callback(self, data):
        self.imu_orientation = [data.orientation.x, data.orientation.y, 
                                data.orientation.z, data.orientation.w]
        
    def is_balanced(self):
        """Check if robot is currently balanced"""
        support_polygon = self.calculate_current_support_polygon()
        com_proj = self.project_com_to_ground()
        
        # Calculate distance from CoM projection to edge of support polygon
        distance_to_edge = self.distance_com_to_polygon_edge(com_proj, support_polygon)
        
        return distance_to_edge > self.balance_threshold
```

### Emergency Procedures

```python
class HumanoidEmergencyHandler:
    def __init__(self):
        self.emergency_publisher = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        self.balance_monitor = BalanceMonitor()
        
    def monitor_safety(self):
        """Continuously monitor for emergency situations"""
        while not rospy.is_shutdown():
            if not self.balance_monitor.is_balanced():
                self.execute_emergency_procedure("imbalance_detected")
                
            if self.detect_collision():
                self.execute_emergency_procedure("collision_detected")
                
            rospy.sleep(0.01)  # 100Hz monitoring
            
    def execute_emergency_procedure(self, event_type):
        """Execute appropriate emergency procedure"""
        if event_type == "imbalance_detected":
            # Attempt dynamic stabilization
            if self.can_stabilize():
                self.attempt_dynamic_stabilization()
            else:
                # Safely lower to ground
                self.exec_safe_landing_procedure()
        elif event_type == "collision_detected":
            # Stop all motion and assess damage
            self.emergency_stop()
            
    def can_stabilize(self):
        """Determine if robot can recover balance"""
        # Check if center of mass is recoverable
        # Check if feasible foot placement exists
        # Check if actuators have available torque
        return self.check_recoverability()
```

## Performance Optimization

### Adaptive Parameter Adjustment

Adjust navigation parameters based on terrain and robot state:

```python
class AdaptiveHumanoidNavigator:
    def __init__(self):
        self.terrain_classifier = TerrainClassifier()
        self.robot_state_observer = RobotStateObserver()
        self.path_adjuster = DynamicHumanoidPathAdjuster()
        
    def adjust_parameters_for_terrain(self, current_terrain):
        """Adjust Nav2 parameters based on detected terrain"""
        terrain_type = self.terrain_classifier.classify(current_terrain)
        
        if terrain_type == "uneven":
            # Increase safety margins and slow down
            self.nav_system.set_param("inflation_radius", 0.8)  # Wider safety buffer
            self.nav_system.set_param("step_size_max", 0.15)   # Shorter steps
            self.nav_system.set_param("balance_margin", 0.15)  # More conservative balance
            
        elif terrain_type == "stairs":
            # Configure for stair climbing
            self.nav_system.set_param("step_size_max", 0.25)  # Adjust for step heights
            self.nav_system.set_param("foot_lift_height", 0.1)  # Higher foot lift
            
        elif terrain_type == "narrow_passage":
            # Configure single-file walking
            self.nav_system.set_param("step_size_max", 0.1)   # Very conservative
            self.nav_system.set_param("balance_margin", 0.2)  # Extra stability
        
        # Update other relevant parameters
        self.update_localization_parameters(terrain_type)
```

## Integration with Whole Body Control

Humanoid navigation must integrate with whole-body control systems:

```python
class IntegratedHumanoidNavigation:
    def __init__(self):
        self.navigation_system = Nav2System()
        self.whole_body_controller = WholeBodyController()
        self.footstep_planner = HumanoidFootstepPlanner()
        
    def coordinated_navigation(self, goal):
        """
        Coordinate navigation path following with whole-body control
        """
        # Plan global path
        path = self.navigation_system.plan_path(goal)
        
        # Convert to footstep sequence
        footsteps = self.footstep_planner.plan_footsteps_from_path(path)
        
        # Execute with whole-body coordination
        for footstep in footsteps:
            # Plan whole-body motion to execute footstep
            whole_body_plan = self.whole_body_controller.plan_motion_to_footstep(footstep)
            
            # Execute coordinated motion
            self.whole_body_controller.execute(whole_body_plan)
            
            # Monitor for errors and adjust if needed
            if not self.verify_execution_success():
                self.replan_from_current_state()
```

## Summary

Adapting Nav2 for humanoid navigation requires significant modifications to account for the unique challenges of bipedal locomotion. The key differences include the need for footstep planning instead of continuous path following, stability considerations during movement, and integration with whole-body control systems. With proper implementation of these humanoid-specific components, Nav2 can effectively provide safe and efficient navigation for bipedal robots.

## Exercises

1. Implement a basic footstep planner for a simple humanoid model
2. Configure Nav2 with humanoid-specific costmap layers
3. Develop a simulation scenario with stairs and uneven terrain
4. Implement balance monitoring and emergency procedures
5. Create adaptive parameter adjustment based on terrain classification