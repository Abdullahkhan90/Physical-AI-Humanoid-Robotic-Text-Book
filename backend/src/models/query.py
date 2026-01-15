from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime
from uuid import UUID, uuid4

class Query(BaseModel):
    """
    Query: A user's question input that triggers the RAG process;
    contains the question text and context information (e.g., selected text if applicable)
    """
    id: UUID = Field(default_factory=uuid4)
    question_text: str = Field(..., min_length=1, max_length=1000)  # Question text must be 1-1000 characters
    selected_text: Optional[str] = None  # Optional text selected by user for context, max 5000 chars per spec
    context_used: Optional[str] = None  # Text that was used for retrieval
    timestamp: datetime = Field(default_factory=datetime.now)
    user_id: Optional[UUID] = None  # Optional, for tracking user sessions