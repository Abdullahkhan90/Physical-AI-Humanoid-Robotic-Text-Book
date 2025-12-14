import asyncio
from qdrant_client import QdrantClient
from qdrant_client.http import models
from ..utils.config import config
import logging


async def initialize_qdrant():
    """
    Initialize the Qdrant vector database for the RAG system
    """
    logger = logging.getLogger(__name__)
    
    try:
        # Connect to Qdrant client
        client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY,
            timeout=10
        )
        
        # Check if the collection already exists
        collection_name = config.QDRANT_COLLECTION_NAME
        collections = client.get_collections()
        
        collection_exists = any(col.name == collection_name for col in collections.collections)
        
        if collection_exists:
            print(f"Collection '{collection_name}' already exists.")
            return True
        
        # Create a new collection for textbook content
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=384,  # Dimension of sentence-transformer embeddings (MiniLM)
                distance=models.Distance.COSINE
            )
        )
        
        print(f"Collection '{collection_name}' created successfully.")
        
        # Create payload index for faster filtering
        client.create_payload_index(
            collection_name=collection_name,
            field_name="module_id",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        
        client.create_payload_index(
            collection_name=collection_name,
            field_name="content_id",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        
        print("Payload indexes created successfully.")
        
        return True
        
    except Exception as e:
        logger.error(f"Error initializing Qdrant: {e}")
        return False


if __name__ == "__main__":
    success = asyncio.run(initialize_qdrant())
    if success:
        print("Qdrant initialization completed successfully.")
    else:
        print("Qdrant initialization failed.")