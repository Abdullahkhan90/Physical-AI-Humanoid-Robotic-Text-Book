from pydantic import BaseModel, Field
from typing import Optional, List, Dict
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