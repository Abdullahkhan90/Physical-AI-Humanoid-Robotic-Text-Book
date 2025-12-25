#!/usr/bin/env python3
"""
Main FastAPI application for the RAG Chatbot API
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .routes import query, health, chat

# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG Chatbot that answers questions from Physical AI and Humanoid Robotics textbook content",
    version="1.0.0"
)

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
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
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

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)