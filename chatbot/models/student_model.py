from pydantic import BaseModel
from typing import Dict, Optional
from datetime import datetime


class StudentProfile(BaseModel):
    id: str
    preferences: Dict
    progress: Dict
    personalizationSettings: Dict
    preferredLanguage: str = "en"
    createdAt: datetime
    lastAccessed: datetime