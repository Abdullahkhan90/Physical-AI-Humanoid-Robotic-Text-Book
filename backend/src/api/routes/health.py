from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Dict, Any
import time
import httpx
import datetime

router = APIRouter()

# Response models
class HealthResponse(BaseModel):
    status: str
    timestamp: str
    dependencies: Dict[str, str]

class StatsResponse(BaseModel):
    total_queries: int
    avg_response_time_ms: float
    queries_last_24h: int
    accuracy_rate: float
    timestamp: str

@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Check the health status of the service and its dependencies
    """
    from ...services.cohere_service import CohereService
    from ...services.qdrant_service import QdrantService

    dependencies = {
        "cohere": "unhealthy",  # Default to unhealthy
        "qdrant": "unhealthy"   # Default to unhealthy
    }

    # Test Cohere connectivity
    try:
        cohere_service = CohereService()
        # Test a simple embedding to verify API key works
        test_embedding = cohere_service.embed_texts(["test"])
        if test_embedding and len(test_embedding) > 0:
            dependencies["cohere"] = "healthy"
    except Exception as e:
        dependencies["cohere"] = f"error: {str(e)}"

    # Test Qdrant connectivity
    try:
        qdrant_service = QdrantService()
        # Test by trying to get collection info
        qdrant_service.client.get_collection(qdrant_service.collection_name)
        dependencies["qdrant"] = "healthy"
    except Exception as e:
        dependencies["qdrant"] = f"error: {str(e)}"

    # Overall status depends on all dependencies
    overall_status = "healthy" if all(status == "healthy" for status in dependencies.values()) else "degraded"

    return HealthResponse(
        status=overall_status,
        timestamp=datetime.datetime.now().isoformat(),
        dependencies=dependencies
    )

@router.get("/stats", response_model=StatsResponse)
async def get_stats():
    """
    Get statistics about the service usage
    """
    import datetime
    
    # In a real implementation, this would pull from a database or metrics service
    # For now, returning placeholder data
    return StatsResponse(
        total_queries=0,
        avg_response_time_ms=0.0,
        queries_last_24h=0,
        accuracy_rate=0.0,
        timestamp=datetime.datetime.now().isoformat()
    )