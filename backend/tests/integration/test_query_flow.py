import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from src.services.rag_service import RAGService
from unittest.mock import patch, MagicMock

client = TestClient(app)

def test_query_flow_integration():
    """
    Integration test for basic query functionality
    """
    # Mock the RAG service to avoid actual API calls during testing
    with patch('src.api.routes.query.rag_service') as mock_rag_service:
        # Setup mock return value
        mock_rag_service.query.return_value = {
            "response_text": "This is a test response based on textbook content.",
            "citations": [
                {
                    "section": "Chapter 1: Introduction",
                    "url": "/docs/intro",
                    "text": "Humanoid robotics is an interdisciplinary field..."
                }
            ],
            "retrieved_contexts": [
                {
                    "id": "test_id_1",
                    "text": "Humanoid robotics is an interdisciplinary field...",
                    "metadata": {"section": "Chapter 1: Introduction", "url": "/docs/intro"},
                    "score": 0.95
                }
            ],
            "confidence_score": 0.95,
            "response_time_ms": 150
        }
        
        # Make request to the API
        response = client.post(
            "/api/query",
            json={"question": "What is humanoid robotics?", "session_token": "test_token_123"}
        )
        
        # Validate response
        assert response.status_code == 200
        data = response.json()
        
        assert data["answer"] == "This is a test response based on textbook content."
        assert len(data["citations"]) == 1
        assert data["citations"][0]["section"] == "Chapter 1: Introduction"
        assert data["confidence"] == 0.95
        assert data["response_time_ms"] == 150


def test_query_selected_flow_integration():
    """
    Integration test for selected text query functionality
    """
    # Mock the RAG service to avoid actual API calls during testing
    with patch('src.api.routes.query.rag_service') as mock_rag_service:
        # Setup mock return value
        mock_rag_service.query.return_value = {
            "response_text": "Based on the selected text, the key challenges are...",
            "citations": [
                {
                    "section": "Section 2.3: Challenges",
                    "url": "/docs/challenges",
                    "text": "The main challenges in humanoid robotics include..."
                }
            ],
            "retrieved_contexts": [
                {
                    "id": "test_id_2",
                    "text": "The main challenges in humanoid robotics include...",
                    "metadata": {"section": "Section 2.3: Challenges", "url": "/docs/challenges"},
                    "score": 0.98
                }
            ],
            "confidence_score": 0.98,
            "response_time_ms": 180
        }
        
        # Make request to the API with selected text
        response = client.post(
            "/api/query",
            json={
                "question": "What are the challenges mentioned in this text?", 
                "selected_text": "Humanoid robots face several challenges in locomotion and balance control...",
                "session_token": "test_token_456"
            }
        )
        
        # Validate response
        assert response.status_code == 200
        data = response.json()
        
        assert "based on the selected text" in data["answer"].lower()
        assert len(data["citations"]) == 1
        assert data["citations"][0]["section"] == "Section 2.3: Challenges"
        assert data["confidence"] == 0.98
        assert data["response_time_ms"] == 180