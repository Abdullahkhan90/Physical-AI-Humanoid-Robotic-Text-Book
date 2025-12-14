from typing import Dict, List, Optional
from datetime import datetime
from ..models.student_model import StudentProfile
from ..utils.logging import get_logger


class ProgressTrackingService:
    """
    Service for tracking and managing student progress through the textbook modules
    """
    
    def __init__(self):
        self.logger = get_logger(__name__)
        # In a real implementation, this would connect to a database
        self.student_progress: Dict[str, Dict] = {}
    
    def initialize_student_progress(self, student_id: str) -> bool:
        """
        Initialize progress tracking for a new student
        """
        try:
            if student_id not in self.student_progress:
                self.student_progress[student_id] = {
                    "modules": {},
                    "totalProgress": 0,
                    "lastUpdated": datetime.utcnow().isoformat(),
                    "completedModules": [],
                    "currentModule": None
                }
                self.logger.info(f"Initialized progress tracking for student {student_id}")
                return True
            return True  # Already initialized
        except Exception as e:
            self.logger.error(f"Error initializing progress for student {student_id}: {str(e)}")
            return False
    
    def update_module_progress(self, student_id: str, module_id: str, 
                              completion_percentage: int, 
                              content_completed: Optional[List[str]] = None) -> bool:
        """
        Update progress for a specific module
        """
        try:
            if student_id not in self.student_progress:
                if not self.initialize_student_progress(student_id):
                    return False
            
            if completion_percentage < 0 or completion_percentage > 100:
                self.logger.error(f"Invalid completion percentage: {completion_percentage}")
                return False
            
            # Update module-specific progress
            if "modules" not in self.student_progress[student_id]:
                self.student_progress[student_id]["modules"] = {}
            
            module_progress = self.student_progress[student_id]["modules"].get(module_id, {})
            module_progress.update({
                "completionPercentage": completion_percentage,
                "lastUpdated": datetime.utcnow().isoformat(),
                "contentCompleted": content_completed or []
            })
            
            # Mark as completed if 100% done
            if completion_percentage == 100:
                if "completedModules" not in self.student_progress[student_id]:
                    self.student_progress[student_id]["completedModules"] = []
                
                if module_id not in self.student_progress[student_id]["completedModules"]:
                    self.student_progress[student_id]["completedModules"].append(module_id)
                
                # Update current module (set to next module or None if all completed)
                self.student_progress[student_id]["currentModule"] = self._get_next_module(student_id)
            elif completion_percentage > 0:
                # If progress is greater than 0 but less than 100, set as current module
                self.student_progress[student_id]["currentModule"] = module_id
            
            # Update the module progress
            self.student_progress[student_id]["modules"][module_id] = module_progress
            
            # Recalculate total progress
            self._recalculate_total_progress(student_id)
            
            self.logger.info(f"Updated progress for student {student_id}, module {module_id}: {completion_percentage}%")
            return True
            
        except Exception as e:
            self.logger.error(f"Error updating progress for student {student_id}, module {module_id}: {str(e)}")
            return False
    
    def get_student_progress(self, student_id: str) -> Optional[Dict]:
        """
        Get complete progress information for a student
        """
        try:
            if student_id in self.student_progress:
                return self.student_progress[student_id]
            else:
                # Initialize and return empty progress
                self.initialize_student_progress(student_id)
                return self.student_progress[student_id]
        except Exception as e:
            self.logger.error(f"Error getting progress for student {student_id}: {str(e)}")
            return None
    
    def get_module_progress(self, student_id: str, module_id: str) -> Optional[Dict]:
        """
        Get progress information for a specific module
        """
        try:
            student_progress = self.get_student_progress(student_id)
            if student_progress and "modules" in student_progress:
                return student_progress["modules"].get(module_id)
            return None
        except Exception as e:
            self.logger.error(f"Error getting progress for student {student_id}, module {module_id}: {str(e)}")
            return None
    
    def get_completed_modules(self, student_id: str) -> List[str]:
        """
        Get list of modules completed by the student
        """
        try:
            student_progress = self.get_student_progress(student_id)
            if student_progress:
                return student_progress.get("completedModules", [])
            return []
        except Exception as e:
            self.logger.error(f"Error getting completed modules for student {student_id}: {str(e)}")
            return []
    
    def get_learning_path_progress(self, student_id: str, path_modules: List[str]) -> Dict:
        """
        Get progress specifically for modules in a learning path
        """
        try:
            student_progress = self.get_student_progress(student_id)
            if not student_progress:
                return {}
            
            path_progress = {}
            modules = student_progress.get("modules", {})
            
            for module_id in path_modules:
                if module_id in modules:
                    path_progress[module_id] = modules[module_id]
                else:
                    path_progress[module_id] = {
                        "completionPercentage": 0,
                        "lastUpdated": None,
                        "contentCompleted": []
                    }
            
            return path_progress
        except Exception as e:
            self.logger.error(f"Error getting learning path progress for student {student_id}: {str(e)}")
            return {}
    
    def _get_next_module(self, student_id: str) -> Optional[str]:
        """
        Determine the next module a student should work on
        This is a simplified implementation - in a real system, this would be more sophisticated
        """
        # This would depend on the specific learning path and prerequisites
        # For now, we'll return a placeholder
        all_modules = ["module-1-ros2", "module-2-digital-twin", "module-3-ai-brain", "module-4-vla"]
        completed_modules = set(self.get_completed_modules(student_id))
        
        for module in all_modules:
            if module not in completed_modules:
                return module
        
        # If all modules are completed, return None
        return None
    
    def _recalculate_total_progress(self, student_id: str):
        """
        Recalculate the overall progress percentage for a student
        """
        try:
            student_progress = self.student_progress[student_id]
            modules = student_progress.get("modules", {})
            
            if not modules:
                student_progress["totalProgress"] = 0
                return
            
            # Calculate total progress as average of module progress
            total_module_progress = sum(
                module.get("completionPercentage", 0) for module in modules.values()
            )
            
            num_modules = len(modules)
            if num_modules > 0:
                student_progress["totalProgress"] = round(total_module_progress / num_modules, 2)
            else:
                student_progress["totalProgress"] = 0
            
            # Update last updated timestamp
            student_progress["lastUpdated"] = datetime.utcnow().isoformat()
            
        except Exception as e:
            self.logger.error(f"Error recalculating total progress for student {student_id}: {str(e)}")
    
    def mark_content_completed(self, student_id: str, module_id: str, content_ids: List[str]) -> bool:
        """
        Mark specific content pieces as completed within a module
        """
        try:
            if student_id not in self.student_progress:
                if not self.initialize_student_progress(student_id):
                    return False
            
            # Get current module progress
            module_progress = self.student_progress[student_id]["modules"].get(module_id, {})
            
            # Get existing completed content
            existing_content = set(module_progress.get("contentCompleted", []))
            
            # Add new content IDs
            for content_id in content_ids:
                existing_content.add(content_id)
            
            # Update the module progress
            module_progress["contentCompleted"] = list(existing_content)
            self.student_progress[student_id]["modules"][module_id] = module_progress
            
            # Update overall module completion percentage based on content completion
            # This is a simplified calculation - in reality, content weight might vary
            content_count = len(existing_content)
            # Assuming 10 content pieces per module for example (this would be dynamic in real implementation)
            estimated_completion = min(100, (content_count * 100) // 10)
            
            module_progress["completionPercentage"] = estimated_completion
            
            # Recalculate total progress
            self._recalculate_total_progress(student_id)
            
            self.logger.info(f"Marked content as completed for student {student_id}, module {module_id}: {content_ids}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error marking content completed for student {student_id}: {str(e)}")
            return False