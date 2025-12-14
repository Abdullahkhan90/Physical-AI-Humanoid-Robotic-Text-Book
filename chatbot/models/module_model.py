from pydantic import BaseModel
from typing import List


class TextbookModule(BaseModel):
    id: str
    title: str
    description: str
    topics: List[str]
    learningObjectives: List[str]
    contentPath: str
    order: int
    isActive: bool = True