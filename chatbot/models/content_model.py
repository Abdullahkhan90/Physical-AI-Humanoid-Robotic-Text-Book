from pydantic import BaseModel
from typing import List
from .citation_model import Citation  # Importing Citation model


class LearningContent(BaseModel):
    id: str
    title: str
    content: str
    moduleID: str
    contentType: str
    wordCount: int
    estimatedReadingTime: int
    requiredCitations: List[Citation]