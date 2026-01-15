from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
from uuid import UUID, uuid4

class ChatSession(BaseModel):
    """
    Chat Session: A collection of interactions between the user and the chatbot during a single session
    """
    id: UUID = Field(default_factory=uuid4)
    session_token: str  # Identifier for the user's session
    user_id: Optional[UUID] = None  # Optional, for registered users
    created_at: datetime = Field(default_factory=datetime.now)
    last_activity: datetime = Field(default_factory=datetime.now)
    queries: List[UUID]  # List of query IDs in this session