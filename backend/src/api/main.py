#!/usr/bin/env python3
"""
Main FastAPI application for the RAG Chatbot API
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from .routes import query, health, chat

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