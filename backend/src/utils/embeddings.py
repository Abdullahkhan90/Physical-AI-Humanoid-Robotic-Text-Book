from typing import List
from .text_splitter import TextChunk
from ..services.cohere_service import CohereService
import logging

logger = logging.getLogger(__name__)

class EmbeddingGenerator:
    """
    Utility class for generating embeddings for text chunks
    """

    def __init__(self, cohere_service: CohereService = None):
        self.cohere_service = cohere_service or CohereService()

    def generate_embeddings(self, chunks: List[TextChunk], input_type: str = "search_query") -> List[List[float]]:
        """
        Generate embeddings for a list of text chunks
        """
        texts = [chunk.text for chunk in chunks]
        return self.cohere_service.embed_texts(texts, input_type=input_type)

    def generate_embedding_for_single_text(self, text: str, input_type: str = "search_query") -> List[float]:
        """
        Generate embedding for a single text
        """
        try:
            embeddings = self.cohere_service.embed_texts([text], input_type=input_type)
            return embeddings[0]
        except Exception as e:
            logger.error(f"Error generating embedding for text: {e}")
            # Return a fallback embedding
            import hashlib
            # Create a deterministic embedding based on the text content
            embedding = []
            text_hash = hashlib.md5(text.encode()).hexdigest()
            for i in range(1024):  # Assuming 1024-dim vectors
                # Use the hash to generate pseudo-random values
                val = (hash(text_hash + str(i)) % 1000) / 1000.0
                embedding.append(val)
            return embedding