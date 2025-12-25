from pydantic import BaseModel, Field
from typing import List, Dict
from datetime import datetime
from uuid import UUID, uuid4

class GeneratedResponse(BaseModel):
    """
    Generated Response: The AI-generated answer based on the retrieved context and user's query, including citations
    """
    id: UUID = Field(default_factory=uuid4)
    query_id: UUID  # Foreign key to Query
    response_text: str  # The AI-generated response
    citations: List[Dict]  # List of cited sections from the textbook
    confidence_score: float  # Confidence in the accuracy of the response, 0-1
    created_at: datetime = Field(default_factory=datetime.now)
    response_time_ms: int  # Time taken to generate the response