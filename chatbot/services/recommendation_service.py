from typing import List, Dict, Optional
from ..models.learning_path_model import LearningPath
from ..models.student_model import StudentProfile
from ..services.progress_service import ProgressTrackingService
from ..utils.logging import get_logger
import random


class RecommendationService:
    """
    Service for creating and managing personalized learning path recommendations
    """
    
    def __init__(self, progress_service: ProgressTrackingService = None):
        self.logger = get_logger(__name__)
        self.progress_service = progress_service or ProgressTrackingService()
        
        # Define module dependencies and prerequisites
        self.module_dependencies = {
            "module-1-ros2": [],  # ROS 2 - foundational, no prerequisites
            "module-2-digital-twin": ["module-1-ros2"],  # Simulation builds on ROS knowledge
            "module-3-ai-brain": ["module-1-ros2"],  # AI components also build on ROS
            "module-4-vla": ["module-1-ros2", "module-2-digital-twin", "module-3-ai-brain"]  # Advanced integration requires all previous
        }
        
        # Define module difficulty levels
        self.module_difficulty = {
            "module-1-ros2": "beginner",
            "module-2-digital-twin": "intermediate", 
            "module-3-ai-brain": "intermediate",
            "module-4-vla": "advanced"
        }
        
        # Define module topics for interest matching
        self.module_topics = {
            "module-1-ros2": ["ros", "middleware", "robotics", "control", "nodes", "topics", "services"],
            "module-2-digital-twin": ["simulation", "gazebo", "unity", "physics", "rendering", "environment"],
            "module-3-ai-brain": ["ai", "isaac", "navigation", "perception", "slam", "path planning"],
            "module-4-vla": ["vision", "language", "action", "llm", "cognitive", "whisper"]
        }

    def generate_learning_path(
        self, 
        student_id: str, 
        interests: Optional[List[str]] = None, 
        learning_pace: str = "intermediate",
        preferred_modules: Optional[List[str]] = None
    ) -> Optional[LearningPath]:
        """
        Generate a personalized learning path for a student based on their interests and progress
        """
        try:
            # Get student's current progress
            student_progress = self.progress_service.get_student_progress(student_id)
            completed_modules = set(self.progress_service.get_completed_modules(student_id))
            
            # Determine available modules (not completed and prerequisites met)
            available_modules = self._get_available_modules(completed_modules)
            
            # Filter based on interests if provided
            if interests:
                available_modules = self._filter_by_interests(available_modules, interests)
            
            # Filter based on preferred modules if provided
            if preferred_modules:
                available_modules = [mod for mod in available_modules if mod in preferred_modules]
            
            # Adjust based on learning pace
            available_modules = self._filter_by_pace(available_modules, learning_pace)
            
            # Generate sequence based on dependencies and learning goals
            path_sequence = self._create_sequence(completed_modules, available_modules)
            
            # Create learning path object
            from datetime import datetime
            learning_path = LearningPath(
                id=f"lp_{student_id}_{int(datetime.utcnow().timestamp())}",
                studentID=student_id,
                moduleSequence=path_sequence,
                personalized=True,
                createdAt=datetime.utcnow(),
                lastModified=datetime.utcnow()
            )
            
            self.logger.info(f"Generated learning path for student {student_id} with sequence: {path_sequence}")
            return learning_path
            
        except Exception as e:
            self.logger.error(f"Error generating learning path for student {student_id}: {str(e)}")
            return None

    def _get_available_modules(self, completed_modules: set) -> List[str]:
        """
        Get modules that the student can take (prerequisites met and not completed)
        """
        available = []
        for module_id, dependencies in self.module_dependencies.items():
            # Check if module is not completed
            if module_id not in completed_modules:
                # Check if all prerequisites are completed
                prereqs_met = all(dep in completed_modules for dep in dependencies)
                if prereqs_met:
                    available.append(module_id)
        
        return available

    def _filter_by_interests(self, modules: List[str], interests: List[str]) -> List[str]:
        """
        Filter modules based on student interests
        """
        if not interests:
            return modules
        
        # Normalize interests to lowercase for comparison
        normalized_interests = [interest.lower().strip() for interest in interests]
        
        # Score modules based on interest match
        scored_modules = []
        for module in modules:
            module_topics = self.module_topics.get(module, [])
            score = 0
            
            # Count how many interest words appear in module topics
            for interest in normalized_interests:
                for topic in module_topics:
                    if interest in topic or topic in interest:
                        score += 1
            
            scored_modules.append((module, score))
        
        # Sort by score descending
        scored_modules.sort(key=lambda x: x[1], reverse=True)
        
        # Return modules with at least one match, or all if none match
        filtered_modules = [module for module, score in scored_modules if score > 0]
        if not filtered_modules:
            # If no matches, return all original modules
            return modules
        
        return [module for module, _ in scored_modules]

    def _filter_by_pace(self, modules: List[str], pace: str) -> List[str]:
        """
        Filter modules based on learning pace
        """
        if pace == "beginner":
            # Only include beginner and intermediate modules
            return [m for m in modules if self.module_difficulty[m] in ["beginner", "intermediate"]]
        elif pace == "intermediate":
            # Include all except advanced modules at the end
            return modules
        elif pace == "advanced":
            # Include all modules, but prioritize advanced ones
            filtered = [m for m in modules if self.module_difficulty[m] == "advanced"]
            filtered.extend([m for m in modules if self.module_difficulty[m] in ["beginner", "intermediate"]])
            return filtered
        else:
            # Default to intermediate
            return modules

    def _create_sequence(self, completed_modules: set, available_modules: List[str]) -> List[str]:
        """
        Create a recommended sequence of modules
        """
        # If there are available modules, return them in a logical order
        if available_modules:
            # Sort modules by difficulty level if needed
            # In this simple implementation, we'll order by dependencies
            ordered_modules = []
            
            # Add modules in dependency order
            for module in self.module_dependencies.keys():
                if module in available_modules and module not in ordered_modules:
                    # Add dependencies first
                    for dep in self.module_dependencies[module]:
                        if dep in available_modules and dep not in ordered_modules:
                            ordered_modules.append(dep)
                    
                    # Add current module if not already added
                    if module not in ordered_modules:
                        ordered_modules.append(module)
            
            # Add any remaining modules not handled by dependency order
            for module in available_modules:
                if module not in ordered_modules:
                    ordered_modules.append(module)
            
            return ordered_modules
        
        # If no available modules, return empty list
        return []

    def update_recommendation_for_student(self, student_id: str) -> Optional[LearningPath]:
        """
        Update recommendation based on student's ongoing progress
        """
        try:
            # Get student's current progress
            completed_modules = set(self.progress_service.get_completed_modules(student_id))
            
            # Get current learning path if available
            # In a real implementation, we would fetch this from storage
            # For this example, we'll just generate a new one
            
            # Determine next recommended modules based on progress
            available_modules = self._get_available_modules(completed_modules)
            
            # Create updated learning path
            remaining_modules = [m for m in available_modules if m not in completed_modules]
            
            if remaining_modules:
                return LearningPath(
                    id=f"updated_lp_{student_id}",
                    studentID=student_id,
                    moduleSequence=remaining_modules,
                    personalized=True,
                    createdAt=self.progress_service.get_student_progress(student_id)["lastUpdated"],
                    lastModified=self.progress_service.get_student_progress(student_id).get("lastUpdated", "")
                )
            
            return None
            
        except Exception as e:
            self.logger.error(f"Error updating recommendation for student {student_id}: {str(e)}")
            return None

    def suggest_next_module(self, student_id: str) -> Optional[str]:
        """
        Suggest the next module a student should tackle
        """
        try:
            # Get student's progress
            completed_modules = set(self.progress_service.get_completed_modules(student_id))
            
            # Get available modules
            available_modules = self._get_available_modules(completed_modules)
            
            if available_modules:
                # For now, return the first available module
                # In a more sophisticated system, this would involve more complex ranking
                return available_modules[0]
            
            # If no available modules, student has completed everything
            return None
            
        except Exception as e:
            self.logger.error(f"Error suggesting next module for student {student_id}: {str(e)}")
            return None

    def adjust_path_for_difficulty(self, student_id: str, learning_pace: str) -> Optional[LearningPath]:
        """
        Adjust learning path based on difficulty level
        """
        try:
            # Get the base learning path
            base_path = self.generate_learning_path(student_id, learning_pace=learning_pace)
            
            return base_path
            
        except Exception as e:
            self.logger.error(f"Error adjusting path for difficulty for student {student_id}: {str(e)}")
            return None