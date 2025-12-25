import pytest
from src.services.cohere_service import CohereService
from unittest.mock import patch, MagicMock
import numpy as np

def test_cohere_service_initialization():
    """
    Test Cohere service initialization
    """
    # Mock the environment variable
    with patch.dict('os.environ', {'COHERE_API_KEY': 'test-key-123'}):
        service = CohereService()
        assert service.client is not None


def test_cohere_generate_response():
    """
    Unit test for Cohere response generation
    """
    with patch.dict('os.environ', {'COHERE_API_KEY': 'test-key-123'}):
        service = CohereService()
        
        # Mock the client.chat method (new API)
        with patch.object(service.client, 'chat') as mock_chat:
            # Create a mock response that matches the new API structure
            mock_message = MagicMock()
            mock_content = MagicMock()
            mock_content.text = "This is a generated response."
            mock_message.content = [mock_content]
            mock_response = MagicMock()
            mock_response.message = mock_message
            mock_chat.return_value = mock_response

            response = service.generate_response("Test prompt")

            assert response == "This is a generated response."
            mock_chat.assert_called_once()


def test_cohere_embed_texts():
    """
    Unit test for Cohere text embedding
    """
    with patch.dict('os.environ', {'COHERE_API_KEY': 'test-key-123'}):
        service = CohereService()
        
        # Mock the client.embed method
        with patch.object(service.client, 'embed') as mock_embed:
            # Create mock embeddings - each with 1024 dimensions
            mock_embeddings = [[0.1] * 1024, [0.2] * 1024]
            mock_response = MagicMock()
            mock_response.embeddings = mock_embeddings
            mock_embed.return_value = mock_response
            
            texts = ["Text 1", "Text 2"]
            embeddings = service.embed_texts(texts)
            
            assert len(embeddings) == 2
            assert len(embeddings[0]) == 1024
            assert len(embeddings[1]) == 1024
            mock_embed.assert_called_once_with(
                texts=texts,
                model="embed-english-v3.0"
            )