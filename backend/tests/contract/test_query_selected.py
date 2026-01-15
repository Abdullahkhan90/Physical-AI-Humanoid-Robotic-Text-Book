import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from unittest.mock import patch, MagicMock

client = TestClient(app)

def test_query_selected_endpoint_contract():
    """
    Contract test for POST /query-selected endpoint
    """
    # Test valid request with selected text
    response = client.post(
        "/api/query-selected",
        json={
            "question": "What are the key points in this text?",
            "selected_text": "Humanoid robots face several challenges in locomotion and balance control...",
            "session_token": "test_token_123"
        }
    )
    
    # Should return 200 OK
    assert response.status_code == 200
    
    # Response should have required fields
    data = response.json()
    assert "response_id" in data
    assert "answer" in data
    assert "citations" in data
    assert "confidence" in data
    assert "response_time_ms" in data
    
    # Validate data types
    assert isinstance(data["response_id"], str)  # UUID as string
    assert isinstance(data["answer"], str)
    assert isinstance(data["citations"], list)
    assert isinstance(data["confidence"], (int, float))
    assert isinstance(data["response_time_ms"], int)
    
    # Validate answer is not empty
    assert len(data["answer"]) > 0


def test_query_selected_endpoint_invalid_request():
    """
    Test query selected endpoint with invalid request
    """
    # Test with empty question
    response = client.post(
        "/api/query-selected",
        json={
            "question": "",
            "selected_text": "Sample selected text",
            "session_token": "test_token_123"
        }
    )
    
    # Should return 422 for validation error
    assert response.status_code == 422
    
    # Test with missing question
    response = client.post(
        "/api/query-selected",
        json={
            "selected_text": "Sample selected text",
            "session_token": "test_token_123"
        }
    )
    
    # Should return 422 for validation error
    assert response.status_code == 422
    
    # Test with empty selected text
    response = client.post(
        "/api/query-selected",
        json={
            "question": "What does this text mean?",
            "selected_text": "",
            "session_token": "test_token_123"
        }
    )
    
    # Should return 422 for validation error
    assert response.status_code == 422