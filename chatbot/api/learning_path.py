from fastapi import APIRouter, HTTPException
from typing import List, Dict, Any
from pydantic import BaseModel
from datetime import datetime

from ..models.learning_path_model import LearningPath

router = APIRouter(prefix="/learning-path", tags=["learning-path"])

# Mock data storage - in a real implementation this would use a database
learning_paths: Dict[str, LearningPath] = {}

# Define request and response models
class CreateLearningPathRequest(BaseModel):
    studentID: str
    moduleSequence: List[str]
    personalized: bool = False


class UpdateLearningPathRequest(BaseModel):
    moduleSequence: List[str] = None
    personalized: bool = None


class LearningPathResponse(BaseModel):
    id: str
    studentID: str
    moduleSequence: List[str]
    personalized: bool
    createdAt: str
    lastModified: str


@router.post("/", response_model=LearningPathResponse)
async def create_learning_path(request: CreateLearningPathRequest):
    """
    Create a new learning path for a student
    """
    try:
        from uuid import uuid4
        
        # Generate a unique ID for the learning path
        path_id = str(uuid4())
        
        # Create a new learning path
        learning_path = LearningPath(
            id=path_id,
            studentID=request.studentID,
            moduleSequence=request.moduleSequence,
            personalized=request.personalized,
            createdAt=datetime.utcnow(),
            lastModified=datetime.utcnow()
        )
        
        # Store the learning path
        learning_paths[path_id] = learning_path
        
        return LearningPathResponse(
            id=learning_path.id,
            studentID=learning_path.studentID,
            moduleSequence=learning_path.moduleSequence,
            personalized=learning_path.personalized,
            createdAt=learning_path.createdAt.isoformat(),
            lastModified=learning_path.lastModified.isoformat()
        )
    
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error creating learning path: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while creating the learning path"
        )


@router.get("/{student_id}", response_model=List[LearningPathResponse])
async def get_learning_paths(student_id: str):
    """
    Retrieve all learning paths for a specific student
    """
    try:
        # Find all learning paths for the student
        student_paths = [
            path for path in learning_paths.values() 
            if path.studentID == student_id
        ]
        
        # Convert to response format
        response_paths = [
            LearningPathResponse(
                id=path.id,
                studentID=path.studentID,
                moduleSequence=path.moduleSequence,
                personalized=path.personalized,
                createdAt=path.createdAt.isoformat(),
                lastModified=path.lastModified.isoformat()
            )
            for path in student_paths
        ]
        
        return response_paths
    
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error getting learning paths for student {student_id}: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving learning paths"
        )


@router.get("/{student_id}/current", response_model=LearningPathResponse)
async def get_current_learning_path(student_id: str):
    """
    Retrieve the current/active learning path for a student
    In this implementation, we return the most recently created path
    """
    try:
        # Find all learning paths for the student
        student_paths = [
            path for path in learning_paths.values() 
            if path.studentID == student_id
        ]
        
        if not student_paths:
            raise HTTPException(
                status_code=404,
                detail=f"No learning paths found for student {student_id}"
            )
        
        # Return the most recently created path
        current_path = max(student_paths, key=lambda p: p.createdAt)
        
        return LearningPathResponse(
            id=current_path.id,
            studentID=current_path.studentID,
            moduleSequence=current_path.moduleSequence,
            personalized=current_path.personalized,
            createdAt=current_path.createdAt.isoformat(),
            lastModified=current_path.lastModified.isoformat()
        )
    
    except HTTPException:
        raise
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error getting current learning path for student {student_id}: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving the current learning path"
        )


@router.put("/{path_id}", response_model=LearningPathResponse)
async def update_learning_path(path_id: str, request: UpdateLearningPathRequest):
    """
    Update an existing learning path
    """
    try:
        # Check if the learning path exists
        if path_id not in learning_paths:
            raise HTTPException(
                status_code=404,
                detail=f"Learning path with id {path_id} not found"
            )
        
        learning_path = learning_paths[path_id]
        
        # Update the learning path based on the request
        if request.moduleSequence is not None:
            learning_path.moduleSequence = request.moduleSequence
            
        if request.personalized is not None:
            learning_path.personalized = request.personalized
        
        # Update the last modified timestamp
        learning_path.lastModified = datetime.utcnow()
        
        # Save the updated path
        learning_paths[path_id] = learning_path
        
        return LearningPathResponse(
            id=learning_path.id,
            studentID=learning_path.studentID,
            moduleSequence=learning_path.moduleSequence,
            personalized=learning_path.personalized,
            createdAt=learning_path.createdAt.isoformat(),
            lastModified=learning_path.lastModified.isoformat()
        )
    
    except HTTPException:
        raise
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error updating learning path {path_id}: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while updating the learning path"
        )


@router.delete("/{path_id}")
async def delete_learning_path(path_id: str):
    """
    Delete a learning path
    """
    try:
        # Check if the learning path exists
        if path_id not in learning_paths:
            raise HTTPException(
                status_code=404,
                detail=f"Learning path with id {path_id} not found"
            )
        
        # Remove the learning path
        del learning_paths[path_id]
        
        return {
            "status": "Learning path deleted successfully",
            "pathId": path_id
        }
    
    except HTTPException:
        raise
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error deleting learning path {path_id}: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while deleting the learning path"
        )


@router.post("/{student_id}/recommend")
async def recommend_learning_path(student_id: str, interests: List[str] = None):
    """
    Generate a recommended learning path for a student based on interests
    """
    try:
        # This is a simplified recommendation algorithm
        # In a real implementation, this would use more sophisticated logic
        
        # Default module sequence if no interests are specified
        if not interests or len(interests) == 0:
            default_modules = [
                "module-1-ros2",    # ROS 2 - foundational
                "module-2-digital-twin",  # Simulation
                "module-3-ai-brain",      # AI/Ros components
                "module-4-vla"            # Advanced integration
            ]
        else:
            # In a real implementation, this would match interests to modules
            # For now, return a standard sequence
            default_modules = [
                "module-1-ros2",
                "module-2-digital-twin",
                "module-3-ai-brain",
                "module-4-vla"
            ]
        
        # Create a new learning path
        from uuid import uuid4
        path_id = str(uuid4())
        
        recommended_path = LearningPath(
            id=path_id,
            studentID=student_id,
            moduleSequence=default_modules,
            personalized=True,
            createdAt=datetime.utcnow(),
            lastModified=datetime.utcnow()
        )
        
        # Store the recommended path
        learning_paths[path_id] = recommended_path
        
        return LearningPathResponse(
            id=recommended_path.id,
            studentID=recommended_path.studentID,
            moduleSequence=recommended_path.moduleSequence,
            personalized=recommended_path.personalized,
            createdAt=recommended_path.createdAt.isoformat(),
            lastModified=recommended_path.lastModified.isoformat()
        )
    
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error generating recommended learning path for student {student_id}: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while generating the recommended learning path"
        )