from fastapi import APIRouter, HTTPException
from typing import Dict, Any
from pydantic import BaseModel
from datetime import datetime

from ..models.student_model import StudentProfile

router = APIRouter(prefix="/user", tags=["user"])

# Mock data storage - in a real implementation this would use a database
student_profiles: Dict[str, StudentProfile] = {}

# Define request and response models
class UpdatePreferencesRequest(BaseModel):
    preferredLanguage: str = None
    learningPace: str = None  # beginner, intermediate, advanced
    preferredTopics: list = None
    notificationSettings: Dict[str, Any] = None
    personalizationSettings: Dict[str, Any] = None


class PreferencesResponse(BaseModel):
    preferences: Dict[str, Any]
    progress: Dict[str, Any]
    personalizationSettings: Dict[str, Any]
    preferredLanguage: str


@router.get("/{student_id}/preferences", response_model=PreferencesResponse)
async def get_user_preferences(student_id: str):
    """
    Retrieve a student's preferences and settings
    """
    try:
        if student_id not in student_profiles:
            # Create a default profile if it doesn't exist
            default_profile = StudentProfile(
                id=student_id,
                preferences={},
                progress={},
                personalizationSettings={},
                preferredLanguage="en",
                createdAt=datetime.utcnow(),
                lastAccessed=datetime.utcnow()
            )
            student_profiles[student_id] = default_profile
        
        profile = student_profiles[student_id]
        profile.lastAccessed = datetime.utcnow()  # Update last accessed time
        
        return PreferencesResponse(
            preferences=profile.preferences,
            progress=profile.progress,
            personalizationSettings=profile.personalizationSettings,
            preferredLanguage=profile.preferredLanguage
        )
    
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error getting preferences for student {student_id}: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving user preferences"
        )


@router.put("/{student_id}/preferences")
async def update_user_preferences(student_id: str, request: UpdatePreferencesRequest):
    """
    Update a student's preferences
    """
    try:
        # Check if student profile exists, create if not
        if student_id not in student_profiles:
            profile = StudentProfile(
                id=student_id,
                preferences={},
                progress={},
                personalizationSettings={},
                preferredLanguage="en",
                createdAt=datetime.utcnow(),
                lastAccessed=datetime.utcnow()
            )
            student_profiles[student_id] = profile
        
        profile = student_profiles[student_id]
        
        # Update preferences based on the request
        if request.preferredLanguage is not None:
            profile.preferredLanguage = request.preferredLanguage
            
        if request.learningPace is not None:
            profile.preferences["learningPace"] = request.learningPace
            
        if request.preferredTopics is not None:
            profile.preferences["preferredTopics"] = request.preferredTopics
            
        if request.notificationSettings is not None:
            profile.preferences["notificationSettings"] = request.notificationSettings
            
        if request.personalizationSettings is not None:
            profile.personalizationSettings.update(request.personalizationSettings)
        
        profile.lastAccessed = datetime.utcnow()
        
        return {
            "status": "Preferences updated successfully",
            "studentId": profile.id,
            "updatedAt": profile.lastAccessed.isoformat()
        }
    
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error updating preferences for student {student_id}: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while updating user preferences"
        )


@router.get("/{student_id}/progress")
async def get_user_progress(student_id: str):
    """
    Retrieve a student's learning progress
    """
    try:
        if student_id not in student_profiles:
            raise HTTPException(
                status_code=404,
                detail=f"Student profile with id {student_id} not found"
            )
        
        profile = student_profiles[student_id]
        profile.lastAccessed = datetime.utcnow()
        
        return {
            "studentId": profile.id,
            "progress": profile.progress,
            "preferredLanguage": profile.preferredLanguage
        }
    
    except HTTPException:
        raise
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error getting progress for student {student_id}: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving user progress"
        )


@router.post("/{student_id}/progress/{module_id}")
async def update_module_progress(student_id: str, module_id: str, completion_percentage: int):
    """
    Update progress for a specific module
    """
    try:
        if student_id not in student_profiles:
            raise HTTPException(
                status_code=404,
                detail=f"Student profile with id {student_id} not found"
            )
        
        profile = student_profiles[student_id]
        
        # Validate completion percentage
        if completion_percentage < 0 or completion_percentage > 100:
            raise HTTPException(
                status_code=400,
                detail="Completion percentage must be between 0 and 100"
            )
        
        # Initialize progress dict if not exists
        if profile.progress is None:
            profile.progress = {}
        
        # Update progress for the module
        if 'modules' not in profile.progress:
            profile.progress['modules'] = {}
        
        profile.progress['modules'][module_id] = {
            "completionPercentage": completion_percentage,
            "updatedAt": datetime.utcnow().isoformat()
        }
        
        profile.lastAccessed = datetime.utcnow()
        
        return {
            "status": "Progress updated successfully",
            "studentId": profile.id,
            "module": module_id,
            "completionPercentage": completion_percentage,
            "updatedAt": profile.lastAccessed.isoformat()
        }
    
    except HTTPException:
        raise
    except Exception as e:
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error updating progress for student {student_id}, module {module_id}: {str(e)}")
        
        raise HTTPException(
            status_code=500,
            detail="An error occurred while updating user progress"
        )