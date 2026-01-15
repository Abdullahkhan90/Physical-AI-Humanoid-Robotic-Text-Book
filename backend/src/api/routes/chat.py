from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from uuid import UUID, uuid4
import logging
from ...services.rag_service import RAGService
from ...services.citation_service import CitationService

router = APIRouter()
logger = logging.getLogger(__name__)

# Request models
class ChatMessage(BaseModel):
    role: str = Field(..., description="Role of the message sender (user or assistant)")
    content: str = Field(..., min_length=1, max_length=2000)

class ChatRequest(BaseModel):
    messages: List[ChatMessage] = Field(..., min_items=1)
    session_token: Optional[str] = None
    selected_text: Optional[str] = ""

class QueryRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=1000)
    session_token: Optional[str] = None

class QuerySelectedRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=1000)
    selected_text: str = Field(..., min_length=1, max_length=5000)
    session_token: Optional[str] = None

class Citation(BaseModel):
    section: str
    url: str
    text: str

class ChatResponse(BaseModel):
    response_id: UUID
    content: str
    citations: List[Citation]
    confidence: float
    response_time_ms: int

class QueryResponse(BaseModel):
    response_id: UUID
    answer: str
    citations: List[Citation]
    confidence: float
    response_time_ms: int


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    try:
        # Initialize services when needed
        rag_service = RAGService()
        citation_service = CitationService()

        # Extract the last user message as the question
        user_messages = [msg for msg in request.messages if msg.role == "user"]
        if not user_messages:
            # If no user messages found, check if there are any messages at all and take the last one as user input
            if request.messages:
                # Assume the last message is from user if no explicit user role found
                last_user_message = request.messages[-1].content
            else:
                raise HTTPException(status_code=400, detail="No user message found in the conversation")
        else:
            last_user_message = user_messages[-1].content

        # Process the query using RAG service
        result = rag_service.query(last_user_message, selected_text=request.selected_text)

        # Format citations
        citations = citation_service.format_citations(result["retrieved_contexts"])

        # Validate citations
        if not citation_service.validate_citations(citations):
            logger.warning("Some citations are missing required fields")

        # For now, we'll generate a dummy response ID
        # In a full implementation, we would create and store the response in the database
        from uuid import uuid4
        response_id = uuid4()

        return ChatResponse(
            response_id=response_id,
            content=result["response_text"],
            citations=citations,
            confidence=result["confidence_score"],
            response_time_ms=result["response_time_ms"]
        )
    except Exception as e:
        logger.error(f"Error processing chat: {e}")
        # Return a more user-friendly error message
        from uuid import uuid4
        return ChatResponse(
            response_id=uuid4(),
            content="Sorry, I'm having trouble processing your request right now. Please try again.",
            citations=[],
            confidence=0.0,
            response_time_ms=0
        )


