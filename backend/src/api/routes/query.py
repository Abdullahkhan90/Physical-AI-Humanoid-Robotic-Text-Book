from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import List, Optional, Dict
from uuid import UUID, uuid4
import logging
from ...services.rag_service import RAGService
from ...services.citation_service import CitationService

router = APIRouter()
logger = logging.getLogger(__name__)

# Request models
class QueryRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=1000)
    session_token: Optional[str] = None

class QuerySelectedRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=1000)
    selected_text: str = Field(..., min_length=1, max_length=5000)
    session_token: Optional[str] = None

# Response models
class Citation(BaseModel):
    section: str
    url: str
    text: str

class QueryResponse(BaseModel):
    response_id: UUID
    answer: str
    citations: List[Citation]
    confidence: float
    response_time_ms: int


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    try:
        # Initialize services when needed
        rag_service = RAGService()
        citation_service = CitationService()

        # Process the query using RAG service
        result = rag_service.query(request.question)

        # Format citations
        citations = citation_service.format_citations(result["retrieved_contexts"])

        # Validate citations
        if not citation_service.validate_citations(citations):
            logger.warning("Some citations are missing required fields")

        # For now, we'll generate a dummy response ID
        # In a full implementation, we would create and store the response in the database
        from uuid import uuid4
        response_id = uuid4()

        return QueryResponse(
            response_id=response_id,
            answer=result["response_text"],
            citations=citations,
            confidence=result["confidence_score"],
            response_time_ms=result["response_time_ms"]
        )
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        # Return a more user-friendly error message
        from uuid import uuid4
        return QueryResponse(
            response_id=uuid4(),
            answer="Sorry, I'm having trouble processing your request right now. Please try again.",
            citations=[],
            confidence=0.0,
            response_time_ms=0
        )


@router.post("/query-selected", response_model=QueryResponse)
async def query_selected_endpoint(request: QuerySelectedRequest):
    try:
        # Initialize services when needed
        rag_service = RAGService()
        citation_service = CitationService()

        # Process the query with selected text using RAG service
        result = rag_service.query(request.question, selected_text=request.selected_text)

        # Format citations
        citations = citation_service.format_citations(result["retrieved_contexts"])

        # Validate citations
        if not citation_service.validate_citations(citations):
            logger.warning("Some citations are missing required fields")

        # For now, we'll generate a dummy response ID
        from uuid import uuid4
        response_id = uuid4()

        return QueryResponse(
            response_id=response_id,
            answer=result["response_text"],
            citations=citations,
            confidence=result["confidence_score"],
            response_time_ms=result["response_time_ms"]
        )
    except Exception as e:
        logger.error(f"Error processing selected text query: {e}")
        # Return a more user-friendly error message
        from uuid import uuid4
        return QueryResponse(
            response_id=uuid4(),
            answer="Sorry, I'm having trouble processing your request right now. Please try again.",
            citations=[],
            confidence=0.0,
            response_time_ms=0
        )