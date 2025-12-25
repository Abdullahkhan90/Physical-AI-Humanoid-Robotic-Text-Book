from pydantic import BaseModel, Field
from typing import Optional, Dict
from datetime import datetime
from uuid import UUID, uuid4

class RetrievedContext(BaseModel):
    """
    Retrieved Context: Relevant textbook content retrieved from the vector store based on the user's query
    """
    id: UUID = Field(default_factory=uuid4)
    query_id: UUID  # Foreign key to Query
    source_text: str  # Source text must not exceed 4000 characters
    metadata: Optional[Dict] = None  # Dictionary for information about where the text came from
    similarity_score: Optional[float] = None  # How similar this context is to the query, 0-1
    created_at: datetime = Field(default_factory=datetime.now)