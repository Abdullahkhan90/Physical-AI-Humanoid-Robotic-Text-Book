from fastapi import APIRouter, HTTPException
from typing import Optional
from pydantic import BaseModel
import asyncio

from ..services.rag_service import RAGService

router = APIRouter(prefix="/chatbot", tags=["chatbot"])

# Initialize the RAG service
rag_service = RAGService()

# Define request and response models
class ChatQueryRequest(BaseModel):
    query: str
    userId: Optional[str] = None  # Not currently used, but kept for future extension
    context: Optional[str] = None  # Module ID to limit search scope


class SourceInfo(BaseModel):
    moduleId: str
    contentId: str
    title: str
    relevanceScore: float


class CitationInfo(BaseModel):
    id: str
    title: str
    authors: List[str]
    source: str
    year: int
    doi: Optional[str] = None
    url: Optional[str] = None
    citationType: str
    apaFormatted: str
    relevanceScore: float


class ChatQueryResponse(BaseModel):
    query: str
    answer: str
    sources: List[SourceInfo]
    citations: List[CitationInfo]
    confidence: float


@router.post("/query", response_model=ChatQueryResponse)
async def query_chatbot(request: ChatQueryRequest):
    """
    Submit a query to the RAG chatbot and receive a response
    """
    try:
        # Process the query through the RAG pipeline
        result = await rag_service.process_query(
            query=request.query,
            module_filter=request.context  # Use context as module filter
        )

        # Process sources and citations from the result
        sources = [
            SourceInfo(
                moduleId=src.get("moduleId", ""),
                contentId=src.get("contentId", ""),
                title=src.get("title", ""),
                relevanceScore=src.get("relevanceScore", 0.0)
            )
            for src in result.get("sources", [])
        ]

        citations = [
            CitationInfo(
                id=cit.get("id", ""),
                title=cit.get("title", ""),
                authors=cit.get("authors", []),
                source=cit.get("source", ""),
                year=cit.get("year", 0),
                doi=cit.get("doi"),
                url=cit.get("url"),
                citationType=cit.get("citationType", ""),
                apaFormatted=cit.get("apaFormatted", ""),
                relevanceScore=cit.get("relevanceScore", 0.0)
            )
            for cit in result.get("citations", [])
        ]

        return ChatQueryResponse(
            query=result["query"],
            answer=result["answer"],
            sources=sources,
            citations=citations,
            confidence=result["confidence"]
        )

    except Exception as e:
        # Log the error
        from ..utils.logging import get_logger
        logger = get_logger(__name__)
        logger.error(f"Error processing chatbot query '{request.query[:50]}...': {str(e)}")

        # Return a user-friendly error response
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your query. Please try again."
        )