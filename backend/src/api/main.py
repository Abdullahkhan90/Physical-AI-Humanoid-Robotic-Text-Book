#!/usr/bin/env python3
"""
Main FastAPI application for the RAG Chatbot API
"""
import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from .routes import query, health, chat
from ..services.ingestion_service import IngestionService

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG Chatbot that answers questions from Physical AI and Humanoid Robotics textbook content",
    version="1.0.0"
)

# Add TrustedHostMiddleware to handle proxy headers properly (important for Hugging Face Spaces)
app.add_middleware(TrustedHostMiddleware, allowed_hosts=["*"])

# Add CORS middleware with specified origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://abdullahkhan90.github.io",
        "https://abdullahkhan90.github.io/Physical-AI-Humanoid-Robotic-Text-Book/",
        "http://localhost:3000",
        "http://localhost:3001",
        "http://localhost:8080",
        "https://localhost:3000",
        "https://localhost:3001",
        "https://localhost:8080",
        "https://hafizabdullah9-backend-rag-chatbot.hf.space",
        "*"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Additional settings to handle preflight requests properly
    max_age=3600,
)

# Include routers
app.include_router(query.router, prefix="/api", tags=["query"])
app.include_router(chat.router, prefix="/api", tags=["chat"])
app.include_router(health.router, prefix="/api", tags=["health"])

# Auto-run ingestion on startup if collection is empty
@app.on_event("startup")
async def startup_event():
    logger.info("Application starting up...")
    try:
        logger.info("Initializing ingestion service...")
        ingestion_service = IngestionService()

        # Check if collection is empty
        if ingestion_service.qdrant_client:
            try:
                logger.info(f"Checking collection '{ingestion_service.collection_name}' status...")
                collection_info = ingestion_service.qdrant_client.get_collection(ingestion_service.collection_name)
                logger.info(f"Collection has {collection_info.points_count} vectors")

                if collection_info.points_count == 0 or collection_info.points_count < 10:
                    logger.info("Qdrant collection is nearly empty, running full ingestion...")
                    result = ingestion_service.ingest_documents()
                    logger.info(f"Ingestion completed: {result}")

                    # Double-check the collection after ingestion
                    collection_info_after = ingestion_service.qdrant_client.get_collection(ingestion_service.collection_name)
                    logger.info(f"Collection now has {collection_info_after.points_count} vectors after ingestion")
                else:
                    logger.info(f"Qdrant collection already has {collection_info.points_count} vectors, skipping ingestion")
            except Exception as e:
                logger.error(f"Error checking collection: {e}")
                logger.info("Running full ingestion...")
                result = ingestion_service.ingest_documents()
                logger.info(f"Ingestion completed: {result}")

                # Double-check the collection after ingestion
                try:
                    collection_info_after = ingestion_service.qdrant_client.get_collection(ingestion_service.collection_name)
                    logger.info(f"Collection now has {collection_info_after.points_count} vectors after ingestion")
                except Exception as e2:
                    logger.error(f"Error checking collection after ingestion: {e2}")
        else:
            logger.warning("Qdrant client not initialized, cannot check collection status")
    except Exception as e:
        logger.error(f"Error during startup ingestion: {e}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")

# Manual endpoint to trigger ingestion
@app.post("/api/ingest")
async def trigger_ingestion():
    """
    Endpoint to trigger document ingestion
    """
    ingestion_service = IngestionService()
    result = ingestion_service.ingest_documents()
    return result

@app.get("/")
async def root():
    """
    Root endpoint to confirm the API is running
    """
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "message": "API is running and accessible"}

@app.post("/api/ingest")
async def trigger_ingestion():
    """
    Endpoint to trigger document ingestion
    """
    ingestion_service = IngestionService()
    result = ingestion_service.ingest_documents()
    return result

# Add a fallback route for debugging that matches the API paths without the prefix
# This can help if there are proxy configuration issues
@app.post("/query")
async def query_fallback(request: dict):
    """
    Fallback query endpoint in case of proxy issues
    """
    from .routes.query import QueryRequest, query_endpoint
    # Convert dict to the expected Pydantic model
    query_request = QueryRequest(**request)
    return await query_endpoint(query_request)

@app.post("/query-selected")
async def query_selected_fallback(request: dict):
    """
    Fallback query-selected endpoint in case of proxy issues
    """
    from .routes.query import QuerySelectedRequest, query_selected_endpoint
    # Convert dict to the expected Pydantic model
    query_request = QuerySelectedRequest(**request)
    return await query_selected_endpoint(query_request)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)