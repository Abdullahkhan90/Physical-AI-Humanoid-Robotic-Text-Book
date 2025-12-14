from pydantic import BaseModel
from typing import List, Dict, Optional


class RAGKnowledgeBase(BaseModel):
    id: str
    moduleID: str
    contentID: str
    chunkText: str
    embedding: List[float]
    metadata: Dict