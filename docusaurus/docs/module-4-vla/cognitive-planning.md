---
sidebar_position: 3
title: Cognitive Planning with LLMs
---

# Cognitive Planning: Using LLMs to Translate Natural Language into ROS 2 Actions

Large Language Models (LLMs) have emerged as powerful tools for natural language understanding and generation. In robotics, they can serve as high-level cognitive planners that interpret natural language commands and translate them into sequences of executable robot actions. This section explores how to integrate LLMs into robotic systems for cognitive planning.

## Introduction to Cognitive Planning in Robotics

Cognitive planning in robotics involves high-level reasoning that bridges natural language commands to low-level robot actions. It requires:
- Understanding natural language commands in context
- Breaking down complex tasks into manageable steps
- Maintaining situational awareness of the environment
- Generating executable sequences of robot actions
- Handling unexpected situations and exceptions

## Role of LLMs in Cognitive Planning

### Advantages of LLMs for Planning

LLMs bring several advantages to cognitive planning:
- **Commonsense Reasoning**: LLMs possess general world knowledge that can inform action planning
- **Natural Language Understanding**: Direct interpretation of human commands without specialized grammars
- **Flexibility**: Ability to handle varied and novel instruction formats
- **Context Awareness**: Understanding of temporal and spatial relationships

### Limitations and Mitigations

However, LLMs also present challenges:
- **Hallucinations**: Producing plausible-sounding but incorrect plans
- **Lack of Real-Time Awareness**: Not aware of current robot state or environment
- **Safety Concerns**: Potential to generate unsafe action sequences
- **Computational Demands**: Resource-intensive inference

## Architecture for LLM-Based Cognitive Planning

### System Components

A typical LLM-based cognitive planning system includes:

```
[User Command] ->
[Natural Language Parser] ->
[LLM Planner] ->
[Action Sequence Generator] ->
[Robot Execution Layer] ->
[Environment Perception] <- Feedback Loop
```

### Integration with ROS 2

The system typically integrates with ROS 2 through:
- Services for high-level command requests
- Action servers for long-running tasks
- Publishers/subscribers for state monitoring
- Transform system for spatial reasoning

## Implementation Approaches

### Prompt Engineering

Effective cognitive planning relies heavily on well-engineered prompts:

```python
def create_planning_prompt(user_command, robot_capabilities, environment_state):
    """
    Creates a structured prompt for the LLM cognitive planner
    """
    prompt = f"""
    You are a cognitive planner for a robot. Your job is to translate human commands into executable robot actions.
    
    Current environment state:
    {environment_state}
    
    Available robot capabilities:
    {robot_capabilities}
    
    Human command: "{user_command}"
    
    Provide the sequence of actions to execute the command. Format your response as:
    1. Action: [action_name] Parameters: [parameters]
    2. Action: [action_name] Parameters: [parameters]
    ...
    
    Only provide actions that the robot is capable of performing.
    """
    
    return prompt
```

### Example Implementation

```python
import openai  # or another LLM API
from typing import List, Dict, Any
import json

class LLMBasedCognitivePlanner:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        self.api_key = api_key
        self.model = model
        openai.api_key = api_key
        
        # Define known robot capabilities
        self.known_capabilities = [
            "move_to_location",
            "pick_object",
            "place_object",
            "navigate",
            "detect_object",
            "grasp",
            "release",
            "follow_route"
        ]
        
    def plan_action_sequence(self, natural_language_command: str, 
                           robot_state: Dict[str, Any],
                           environment_state: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Generate action sequence from natural language command
        """
        # Create planning prompt
        prompt = self._create_planning_prompt(
            natural_language_command,
            robot_state,
            environment_state
        )
        
        # Call LLM
        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}]
        )
        
        # Parse response into action sequence
        response_text = response.choices[0].message['content']
        actions = self._parse_actions(response_text)
        
        # Validate actions
        valid_actions = self._validate_action_sequence(actions)
        
        return valid_actions
    
    def _create_planning_prompt(self, command: str, 
                                robot_state: Dict[str, Any],
                                env_state: Dict[str, Any]) -> str:
        """Create a structured prompt for the LLM"""
        # Implementation of prompt creation
        # Include robot capabilities, environment context, etc.
        prompt = f"""
        As a cognitive planning system for a robot, translate this natural language command into a sequence of actions:
        Command: "{command}"
        
        Environment context:
        - Objects: {[obj['name'] for obj in env_state.get('objects', [])]}
        - Locations: {[loc['name'] for loc in env_state.get('locations', [])]}
        - Robot position: {robot_state.get('position', 'unknown')}
        - Robot battery: {robot_state.get('battery_level', 'unknown')}%
        
        Available actions:
        - move_to_location: Move robot to a named location
        - pick_object: Pick up an object by name
        - place_object: Place held object at location
        - navigate: Navigate to coordinates
        - detect_object: Look for an object
        - grasp: Grasp current object
        - release: Release current object
        - follow_route: Follow predefined route
        
        Provide the plan as numbered steps in this format:
        1. move_to_location: location="kitchen"
        2. detect_object: object="cup"
        3. pick_object: object="cup"
        4. move_to_location: location="table"
        5. place_object: location="table"

        Return only the action sequence, nothing else.
        """
        return prompt
    
    def _parse_actions(self, response_text: str) -> List[Dict[str, Any]]:
        """Parse the LLM response into executable actions"""
        actions = []
        
        # Simple regex-based parser (more sophisticated approaches exist)
        import re
        
        # Match pattern like: "1. action_name: param1=value1, param2=value2"
        pattern = r'(\d+)\.\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*:\s*(.*)'
        
        matches = re.findall(pattern, response_text)
        
        for match in matches:
            step_num, action_name, params_str = match
            
            # Parse parameters
            params = {}
            if params_str.strip():
                # Split on commas that are not within quotes
                param_pairs = [p.strip() for p in params_str.split(',')]
                for pair in param_pairs:
                    if '=' in pair:
                        key, value = pair.split('=', 1)
                        key = key.strip()
                        value = value.strip().strip('"\'')
                        
                        # Try to convert to appropriate type
                        if value.lower() == 'true':
                            value = True
                        elif value.lower() == 'false':
                            value = False
                        elif value.isdigit():
                            value = int(value)
                        else:
                            try:
                                value = float(value)
                            except ValueError:
                                pass  # Keep as string
                        
                        params[key] = value
            
            action = {
                "name": action_name,
                "parameters": params,
                "step_number": int(step_num)
            }
            actions.append(action)
        
        return actions
    
    def _validate_action_sequence(self, action_sequence: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Validate that actions are executable by the robot"""
        valid_actions = []
        
        for action in action_sequence:
            action_name = action['name']
            
            if action_name not in self.known_capabilities:
                print(f"Warning: Unknown action '{action_name}' in sequence, skipping")
                continue
            
            # Validate parameters if needed
            valid_actions.append(action)
        
        return valid_actions
```

## Integration with Robot Control Systems

### ROS 2 Action Server Pattern

For complex tasks, integrate with ROS 2 action servers:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from your_robot_interfaces.action import ExecuteCommand
from std_msgs.msg import String

class CognitivePlanningActionServer(Node):
    def __init__(self):
        super().__init__('cognitive_planning_server')
        
        # Initialize the LLM planner
        self.planner = LLMBasedCognitivePlanner(api_key=your_api_key)
        
        # Create action server
        self._action_server = ActionServer(
            self,
            ExecuteCommand,
            'execute_command',
            self.execute_command_callback)
        
        # Robot state and environment publishers/subscribers
        self.state_publisher = self.create_publisher(String, 'cognitive_planner/state', 10)
        
    def execute_command_callback(self, goal_handle):
        """Execute a natural language command"""
        self.get_logger().info(f'Executing command: {goal_handle.request.command}')
        
        # Get current robot and environment state
        robot_state = self._get_robot_state()
        env_state = self._get_environment_state()
        
        # Plan action sequence
        try:
            action_sequence = self.planner.plan_action_sequence(
                goal_handle.request.command,
                robot_state,
                env_state
            )
            
            # Execute the sequence
            for i, action in enumerate(action_sequence):
                # Update feedback
                feedback_msg = ExecuteCommand.Feedback()
                feedback_msg.current_step = f"Executing: {action['name']} ({i+1}/{len(action_sequence)})"
                goal_handle.publish_feedback(feedback_msg)
                
                # Execute action
                success = self._execute_single_action(action)
                if not success:
                    goal_handle.abort()
                    result = ExecuteCommand.Result()
                    result.success = False
                    result.message = f"Failed to execute action: {action}"
                    return result
        
            # Complete successfully
            goal_handle.succeed()
            result = ExecuteCommand.Result()
            result.success = True
            result.message = f"Successfully executed command with {len(action_sequence)} actions"
            return result
            
        except Exception as e:
            self.get_logger().error(f'Error executing command: {str(e)}')
            goal_handle.abort()
            result = ExecuteCommand.Result()
            result.success = False
            result.message = f"Error during planning: {str(e)}"
            return result
    
    def _execute_single_action(self, action: Dict[str, Any]) -> bool:
        """Execute a single action using ROS 2 interfaces"""
        # This would contain the actual ROS 2 service calls and action executions
        action_name = action['name']
        params = action['parameters']
        
        # Example mapping to ROS 2 services
        if action_name == "move_to_location":
            return self._move_to_location(params.get("location"))
        elif action_name == "pick_object":
            return self._pick_object(params.get("object"))
        elif action_name == "place_object":
            return self._place_object(params.get("location"))
        # Additional action mappings...
        
        return False  # Unknown action
    
    def _get_robot_state(self) -> Dict[str, Any]:
        """Get current robot state"""
        # Implementation would query robot state topics
        return {
            "position": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "battery_level": 85,
            "gripper_status": "open"
        }
    
    def _get_environment_state(self) -> Dict[str, Any]:
        """Get current environment state"""
        # Implementation would query environment mapping topics
        return {
            "objects": [
                {"name": "cup", "position": {"x": 1.0, "y": 1.0}},
                {"name": "box", "position": {"x": 2.0, "y": 0.5}}
            ],
            "locations": [
                {"name": "kitchen", "coordinates": {"x": 5.0, "y": 5.0}},
                {"name": "table", "coordinates": {"x": 3.0, "y": 2.0}}
            ]
        }
```

## Safety and Validation

### Action Validation

Before executing commands generated by LLMs, implement validation steps:

```python
class ActionSafetyValidator:
    def __init__(self):
        # Define safety constraints
        self.forbidden_objects = ["person", "face", "hand", "eye"]  # Objects that shouldn't be grabbed
        self.safe_zones = []  # Define allowed navigation areas
        self.max_velocity = 0.5  # Restrict robot velocities
        self.critical_actions = ["open_valve", "close_door"]  # Actions requiring extra validation
    
    def validate_action_sequence(self, action_sequence, current_state, environment_state):
        """Validate an action sequence for safety"""
        for action in action_sequence:
            if not self._is_action_safe(action, current_state, environment_state):
                return False, f"Unsafe action: {action['name']} with parameters {action['parameters']}"
        
        return True, "All actions are safe"
    
    def _is_action_safe(self, action, current_state, environment_state):
        """Check if a single action is safe to execute"""
        action_name = action['name']
        params = action['parameters']
        
        # Check for forbidden object names
        if 'object' in params:
            obj_name = params['object'].lower()
            if any(forbidden in obj_name for forbidden in self.forbidden_objects):
                return False
        
        # Check navigation targets
        if action_name == 'move_to_location' and 'location' in params:
            if not self._is_location_safe(params['location'], environment_state):
                return False
        
        # Check critical actions
        if action_name in self.critical_actions:
            # Require additional human confirmation for critical actions
            return False  # For now, block all critical actions
        
        return True
    
    def _is_location_safe(self, location, environment_state):
        """Check if a location is in safe zones"""
        # Implementation would check if location is in predefined safe zones
        return True
```

## Performance and Efficiency Optimization

### Caching

Cache frequent command patterns to reduce LLM API calls:

```python
from functools import lru_cache

class CachedCognitivePlanner(LLMBasedCognitivePlanner):
    @lru_cache(maxsize=100)
    def plan_with_cache(self, command: str, robot_state_hash: str, env_state_hash: str):
        """Cached version of planning method"""
        # Convert hashes back to actual state objects
        # Call the parent planning method
        pass
```

### Local Models

For privacy and efficiency, consider using local LLM implementations:

```python
from transformers import pipeline

class LocalCognitivePlanner:
    def __init__(self, model_name="microsoft/DialoGPT-medium"):
        self.generator = pipeline(
            "text-generation",
            model=model_name,
            tokenizer=model_name
        )
    
    def plan_local(self, command: str):
        """Plan using local model"""
        # Implementation would use local LLM
        pass
```

## Error Handling and Recovery

### Plan Failure Handling

Implement strategies for when generated plans fail:

```python
class RobustCognitivePlanner:
    def __init__(self):
        self.planner = LLMBasedCognitivePlanner()
        self.validator = ActionSafetyValidator()
    
    def execute_with_recovery(self, command: str):
        """Execute command with error recovery"""
        max_retries = 3
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                # Plan and validate
                action_seq = self.planner.plan_action_sequence(
                    command,
                    self._get_current_state(),
                    self._get_env_state()
                )
                
                is_safe, reason = self.validator.validate_action_sequence(
                    action_seq, 
                    self._get_current_state(), 
                    self._get_env_state()
                )
                
                if not is_safe:
                    raise ValueError(f"Plan unsafe: {reason}")
                
                # Execute with monitoring
                result = self._execute_with_monitoring(action_seq)
                
                if result.success:
                    return result
                else:
                    # Plan execution failed, try again with environmental context
                    retry_count += 1
                    command = self._refine_command_for_retry(command, result.failure_reason)
            
            except Exception as e:
                retry_count += 1
                if retry_count >= max_retries:
                    raise RuntimeError(f"Command failed after {max_retries} attempts: {str(e)}")
    
    def _refine_command_for_retry(self, original_command: str, failure_reason: str):
        """Adjust command based on failure reason"""
        return f"{original_command}. Note: {failure_reason}. Please adjust your plan accordingly."
```

## Evaluation and Improvement

### Metrics for Cognitive Planning

Evaluate the effectiveness of your cognitive planning system:

- **Task Success Rate**: Percentage of commands successfully executed
- **Plan Accuracy**: How closely generated plans match intended actions
- **Response Time**: Time from command to initial action
- **User Satisfaction**: Subjective rating of how well the system understood commands

### Continuous Learning

Implement feedback mechanisms to improve over time:

```python
class SelfImprovingPlanner:
    def __init__(self):
        self.planner = LLMBasedCognitivePlanner()
        self.feedback_archive = []
    
    def record_interaction(self, command, plan, execution_result, human_feedback):
        """Record interaction for future learning"""
        interaction = {
            "command": command,
            "generated_plan": plan,
            "execution_result": execution_result,
            "feedback": human_feedback,
            "timestamp": time.time()
        }
        
        self.feedback_archive.append(interaction)
    
    def refine_with_feedback(self):
        """Use feedback to improve future planning"""
        # Implementation would analyze successful patterns in feedback archive
        pass
```

## Summary

LLM-based cognitive planning offers significant potential for natural human-robot interaction by directly translating natural language commands into robot actions. However, successful implementation requires careful attention to safety, validation, and error handling. The approach works best when combined with traditional robotics systems that provide environmental awareness and safety guarantees.

## Exercises

1. Implement a simple cognitive planner that converts "Bring me the red cup from the kitchen" into a sequence of ROS 2 actions
2. Add safety validation to prevent the robot from attempting to grasp forbidden objects
3. Create a feedback mechanism to refine plans based on execution failures
4. Implement caching for frequently executed commands
5. Design a multimodal cognitive planner that considers both speech and visual input