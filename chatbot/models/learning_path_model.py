from pydantic import BaseModel
from typing import List
from datetime import datetime


class LearningPath(BaseModel):
    id: str
    studentID: str
    moduleSequence: List[str]
    personalized: bool
    createdAt: datetime
    lastModified: datetime