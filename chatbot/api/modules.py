from fastapi import APIRouter, HTTPException
from typing import List
from ..models.module_model import TextbookModule

router = APIRouter(prefix="/modules", tags=["modules"])

# Mock data - in real implementation this would come from a database
modules_data: List[TextbookModule] = [
    TextbookModule(
        id="module-1-ros2",
        title="The Robotic Nervous System (ROS 2)",
        description="Middleware for robot control",
        topics=["ROS 2 Nodes, Topics, and Services", "Bridging Python Agents to ROS controllers using rclpy", "URDF (Unified Robot Description Format) for Humanoids"],
        learningObjectives=["Understanding of the ROS 2 architecture", "Practical application of rclpy", "Ability to model humanoid robots using URDF"],
        contentPath="/docs/module-1-ros2/",
        order=1,
        isActive=True
    ),
    TextbookModule(
        id="module-2-digital-twin",
        title="The Digital Twin (Gazebo & Unity)",
        description="Physics simulation and environment building",
        topics=["Simulating physics, gravity, and collisions in Gazebo", "High-fidelity rendering and human-robot interaction in Unity", "Simulating sensors: LiDAR, Depth Cameras, and IMUs"],
        learningObjectives=["Ability to set up and configure a realistic simulation environment", "Understanding how to implement and simulate various robot sensors"],
        contentPath="/docs/module-2-digital-twin/",
        order=2,
        isActive=True
    ),
    TextbookModule(
        id="module-3-ai-brain",
        title="The AI-Robot Brain (NVIDIA Isaac)",
        description="Advanced perception and training",
        topics=["NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation", "Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation", "Nav2: Path planning for bipedal humanoid movement"],
        learningObjectives=["Ability to work with Isaac Sim to create photorealistic robot environments", "Understanding of Visual SLAM and its applications in robot perception", "Application of Nav2 for humanoid robot movement and navigation"],
        contentPath="/docs/module-3-ai-brain/",
        order=3,
        isActive=True
    ),
    TextbookModule(
        id="module-4-vla",
        title="Vision-Language-Action (VLA)",
        description="The convergence of LLMs and Robotics",
        topics=["Voice-to-Action: Using OpenAI Whisper for voice commands", "Cognitive Planning: Using LLMs to translate natural language into a sequence of ROS 2 actions", "Capstone Project: The Autonomous Humanoid"],
        learningObjectives=["Integration of OpenAI Whisper for voice-based commands", "Understanding and application of Cognitive Planning for robots to follow natural language instructions", "Successful completion of the Autonomous Humanoid project"],
        contentPath="/docs/module-4-vla/",
        order=4,
        isActive=True
    )
]


@router.get("/", response_model=List[TextbookModule])
async def get_all_modules():
    """
    Retrieve all textbook modules
    """
    return modules_data


@router.get("/{module_id}", response_model=TextbookModule)
async def get_module_by_id(module_id: str):
    """
    Retrieve a specific module by its ID
    """
    for module in modules_data:
        if module.id == module_id:
            return module
    
    raise HTTPException(status_code=404, detail=f"Module with id {module_id} not found")