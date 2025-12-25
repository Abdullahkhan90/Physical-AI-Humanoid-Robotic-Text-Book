import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
from src.api.main import app

client = TestClient(app)

def test_query_endpoint_contract():
    """
    Contract test for POST /query endpoint
    """
    # Test valid request
    response = client.post(
        "/api/query",
        json={"question": "What is humanoid robotics?", "session_token": "test_token_123"}
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


def test_query_endpoint_invalid_request():
    """
    Test query endpoint with invalid request
    """
    # Test with empty question
    response = client.post(
        "/api/query",
        json={"question": "", "session_token": "test_token_123"}
    )
    
    # Should return 422 for validation error
    assert response.status_code == 422
    
    # Test with missing question
    response = client.post(
        "/api/query",
        json={"session_token": "test_token_123"}
    )
    
    # Should return 422 for validation error
    assert response.status_code == 422