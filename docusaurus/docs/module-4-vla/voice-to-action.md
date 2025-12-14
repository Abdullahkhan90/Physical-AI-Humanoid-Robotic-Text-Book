---
sidebar_position: 2
title: Voice-to-Action with OpenAI Whisper
---

# Voice-to-Action: Using OpenAI Whisper for Voice Commands

Natural human-machine interaction relies heavily on the ability for humans to communicate with robots using natural language. This section focuses on implementing voice-to-action systems that allow users to control robots using spoken commands, leveraging OpenAI Whisper for voice recognition and interpretation.

## Introduction to Voice-to-Action Systems

Voice-to-action systems are crucial for natural human-robot interaction. They enable:
- Intuitive user interfaces that don't rely on screens or buttons
- Hands-free operation for tasks that require manual attention
- Accessibility improvements for users with motor impairments
- Enhanced productivity in industrial and domestic environments

The process involves several components:
- Voice recognition (converting speech to text)
- Natural language understanding (interpreting the command semantics)
- Action mapping (translating to robot actions)
- Execution and feedback

## OpenAI Whisper for Voice Recognition

### Overview

OpenAI Whisper is an automatic speech recognition (ASR) system trained on a large dataset of diverse audio. It demonstrates strong performance across multiple languages and accents, making it suitable for multilingual robotics applications.

Key features of Whisper:
- Robustness to accents, background noise, and technical language
- Multilingual support (speech-to-text in multiple languages)
- Speaker identification capabilities
- Time-stamped transcription alignment

### Whisper Architecture

Whisper uses a Transformer-based encoder-decoder architecture:
- Audio encoder: Processes mel-scaled spectrograms
- Text decoder: Generates text tokens conditional on audio
- Tasks: Transcription, translation, language identification

### Whisper Variants

Different model sizes offer various trade-offs between accuracy and performance:
- `tiny` and `base`: Fast, lighter models for real-time or edge applications
- `small` and `medium`: Balanced between accuracy and performance
- `large`: Most accurate, suitable for demanding applications

## Implementing Voice Recognition

### Installation and Setup

```bash
# Install Whisper dependencies
pip install openai-whisper
# Note: Whisper requires PyTorch and certain system libraries (ffmpeg, etc.)
```

### Basic Whisper Usage

```python
import whisper

# Load model - specify size based on your requirements
model = whisper.load_model("base")

# Transcribe audio file
result = model.transcribe("audio.mp3")

# Access the transcribed text
command_text = result["text"]
print(command_text)
```

### Real-time Voice Processing

For real-time applications, you'll need to handle streaming audio:

```python
import pyaudio
import wave
import whisper
import numpy as np
from threading import Thread
import queue

class WhisperVoiceProcessor:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.command_queue = queue.Queue()
        
        # Audio recording parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024
        self.record_seconds = 5
        
    def record_audio(self, filename="temp_record.wav"):
        """Record audio to file for processing"""
        p = pyaudio.PyAudio()
        
        stream = p.open(format=self.format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        frames_per_buffer=self.chunk)
        
        print("Recording...")
        frames = []
        
        for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)
        
        print("Finished recording")
        
        # Stop and close stream
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        # Save recorded audio to WAV file
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        return filename
    
    def process_audio(self, audio_file):
        """Process audio file with Whisper"""
        result = self.model.transcribe(audio_file)
        return result["text"]
    
    def listen_and_process(self):
        """Continuously listen for voice commands"""
        while True:
            audio_file = self.record_audio()
            command = self.process_audio(audio_file)
            
            # Add processed command to queue
            self.command_queue.put(command)
            
            # (Optional) delete temporary file
            import os
            os.remove(audio_file)
            
            print(f"Recognized command: {command}")

# Example usage
processor = WhisperVoiceProcessor()
# processor.listen_and_process()  # Start listening
```

## Natural Language Understanding Integration

After converting speech to text, the system needs to understand the intent:

### Intent Classification

For command-based robotics, you may use classification:

```python
def classify_intent(text):
    """Classify the intent of the voice command"""
    text_lower = text.lower().strip()
    
    # Define command patterns and mappings
    if "move" in text_lower and "forward" in text_lower:
        return {"intent": "MOVE_FORWARD", "params": {}}
    elif "turn" in text_lower and ("left" in text_lower or "right" in text_lower):
        direction = "left" if "left" in text_lower else "right"
        return {"intent": "TURN", "params": {"direction": direction}}
    elif "stop" in text_lower or "halt" in text_lower:
        return {"intent": "STOP", "params": {}}
    elif "pick" in text_lower or "grasp" in text_lower:
        return {"intent": "PICK_UP_OBJECT", "params": {}}
    elif "place" in text_lower or "put" in text_lower:
        return {"intent": "PLACE_OBJECT", "params": {}}
    else:
        return {"intent": "UNKNOWN", "params": {"text": text}}
```

### Named Entity Recognition

For more complex commands with specific objects or locations:

```python
import re
from typing import Dict, List

def extract_entities(text: str) -> Dict[str, List[str]]:
    """Extract named entities from command text"""
    entities = {
        "objects": [],
        "locations": [],
        "quantities": [],
        "people": []
    }
    
    # Define object patterns
    object_patterns = [
        r"cube|sphere|box|ball|cylinder|cone|pyramid",  # Basic shapes
        r"red|green|blue|yellow|white|black",           # Colors
        r"small|large|medium",                         # Size descriptors
        r"book|cup|plate|apple|banana"                 # Specific objects
    ]
    
    location_patterns = [
        r"kitchen|bedroom|living room|office|bathroom", # Rooms
        r"shelf|table|counter|desk|floor"               # Locations
    ]
    
    # Apply patterns to extract entities
    for pattern in object_patterns:
        matches = re.findall(pattern, text, re.IGNORECASE)
        entities["objects"].extend(matches)
    
    for pattern in location_patterns:
        matches = re.findall(pattern, text, re.IGNORECASE)
        entities["locations"].extend(matches)
    
    # Extract number entities
    quantity_matches = re.findall(r"\b\d+\b", text)
    entities["quantities"] = [int(q) for q in quantity_matches]
    
    return entities
```

## Mapping Commands to Actions

The system needs to translate understood commands into robot actions:

```python
class CommandMapper:
    def __init__(self):
        self.robot_actions = {
            "MOVE_FORWARD": self.move_forward,
            "TURN": self.turn,
            "STOP": self.stop,
            "PICK_UP_OBJECT": self.pick_up_object,
            "PLACE_OBJECT": self.place_object
        }
    
    def execute_intent(self, intent_result):
        """Execute the action corresponding to the recognized intent"""
        intent = intent_result["intent"]
        params = intent_result["params"]
        
        if intent in self.robot_actions:
            action_func = self.robot_actions[intent]
            return action_func(params)
        else:
            print(f"Unknown intent: {intent}")
            return False
    
    def move_forward(self, params):
        """Move robot forward"""
        # Implementation depends on robot platform
        print("Moving forward")
        # Example: send command via ROS
        # self.move_publisher.publish(Float64(0.5))  # move forward at 0.5 m/s
        return True
    
    def turn(self, params):
        """Turn robot left or right"""
        direction = params.get("direction", "left")
        angle = params.get("angle", 90)  # degrees
        print(f"Turning {direction}")
        return True
    
    def stop(self, params):
        """Stop robot movement"""
        print("Stopping robot")
        return True
    
    def pick_up_object(self, params):
        """Pick up an object"""
        print("Attempting to pick up object")
        # Implementation would involve:
        # 1. Object detection
        # 2. Positioning robot/arm
        # 3. Gripping mechanism activation
        return True
    
    def place_object(self, params):
        """Place object at specified location"""
        location = params.get("location", "table")
        print(f"Placing object at {location}")
        return True
```

## Integration with ROS

For integration with ROS-based robot control:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import queue
import threading

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        
        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Queue for incoming voice commands
        self.command_queue = queue.Queue()
        
        # Start voice processing thread
        self.voice_processor = WhisperVoiceProcessor()
        self.processing_thread = threading.Thread(target=self.process_voice_commands)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        # Start command execution timer
        self.timer = self.create_timer(0.1, self.execute_commands)
        
        # Command mapper
        self.command_mapper = CommandMapper()
        
    def process_voice_commands(self):
        """Continuously process voice commands"""
        while True:
            try:
                # Continuously listen for voice commands
                audio_file = self.voice_processor.record_audio()
                command_text = self.voice_processor.process_audio(audio_file)
                
                # Classify intent
                intent_result = classify_intent(command_text)
                
                # Add to execution queue
                self.command_queue.put(intent_result)
                
                # Clean up temp file
                import os
                os.remove(audio_file)
            except Exception as e:
                self.get_logger().error(f"Error processing voice command: {e}")
    
    def execute_commands(self):
        """Execute commands from the queue"""
        try:
            while True:
                intent_result = self.command_queue.get_nowait()
                success = self.command_mapper.execute_intent(intent_result)
                if success:
                    self.get_logger().info(f"Executed command: {intent_result['intent']}")
        except queue.Empty:
            pass  # No commands to process

def main(args=None):
    rclpy.init(args=args)
    voice_control_node = VoiceControlNode()
    
    try:
        rclpy.spin(voice_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Privacy and Security Considerations

Voice commands inherently involve capturing audio from the environment, raising important privacy concerns:

### Data Protection
- Local processing when possible to avoid sending audio to cloud services
- Encrypted storage of any recorded commands for debugging
- Clear user consent for data collection and processing
- Automatic deletion of audio data after processing

### Command Validation
- Authentication for commands that control sensitive functions
- Confirmation prompts for critical actions
- Validation of command parameters to prevent unintended behavior

## Performance Optimization

### Latency Reduction
- Efficient microphone buffering to reduce input latency
- Model optimization for faster inference
- Parallel processing where possible
- Edge computing to reduce network latency

### Accuracy Improvements
- Custom fine-tuning on domain-specific vocabulary
- Acoustic model adaptation for specific environments
- Speaker adaptation for consistent users
- Context-aware language modeling

## Error Handling and Robustness

### Recognition Failures
- Graceful degradation when audio quality is poor
- Voice prompts for clarification when command is uncertain
- Ability to repeat command or cancel action
- Fallback to alternative input methods

### Network Resilience
- Offline capability for core commands when possible
- Caching of recent commands
- Graceful failure when network is unavailable

## Evaluation Metrics

To assess the effectiveness of the voice-to-action system:

### Recognition Accuracy
- Word error rate (WER) for transcribed commands
- Intent detection accuracy
- Entity extraction precision and recall

### User Experience
- Average time to execute command
- Number of repetitions needed
- User satisfaction ratings
- Task completion rates

## Summary

The voice-to-action system using OpenAI Whisper provides an intuitive interface for human-robot interaction. By combining robust voice recognition with natural language understanding, robots can respond to human commands in a more natural way. The implementation requires careful consideration of privacy, performance, and accuracy to create a user-friendly system.

## Exercises

1. Implement a voice-controlled robot that moves forward when commanded "go forward"
2. Extend the system to recognize and manipulate specific colored objects
3. Create a multimodal interface that accepts both voice and gesture commands
4. Implement speaker identification to customize responses based on user identity