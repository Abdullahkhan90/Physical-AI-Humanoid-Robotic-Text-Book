import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class QdrantService:
    def __init__(self):
        self.collection_name = "textbook"  # Updated to match user's requirement

        # Initialize client with cloud credentials - using host and port separately for cloud
        self.client = QdrantClient(
            host="a04cc351-47bd-4c14-9a8f-e0b43f1de657.europe-west3-0.gcp.cloud.qdrant.io",  # Using host instead of URL
            port=6333,  # Port for gRPC/HTTP
            https=True,  # Enable HTTPS for cloud connection
            api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.jHHZvJvY-BdFJrvRFx-ipf1bal1_6sNdBxLo17HZ-G8",
            timeout=60,  # optional - timeout badhane ke liye
            verify=False,  # optional - agar SSL issue aaye to
            check_compatibility=False  # Added to address the compatibility warning
        )
        print("Connected to Qdrant Cloud")
        self._init_collection()

    def _init_collection(self):
        """Initialize the collection if it doesn't exist"""
        if not self.client:
            # If no client, skip initialization but don't raise an error for testing purposes
            print("Qdrant client not initialized. Skipping collection initialization. Please set QDRANT_ENDPOINT and QDRANT_API_KEY environment variables.")
            return

        # Verify the client has the required methods
        if not hasattr(self.client, 'get_collection') or not hasattr(self.client, 'create_collection'):
            print(f"Qdrant client does not have required methods. Available methods: {[attr for attr in dir(self.client) if not attr.startswith('_')]}")
            return

        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            print("Collection already exists - skipping creation")
        except:
            # Create collection if it doesn't exist
            try:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config={"size": 768, "distance": "Cosine"}  # Updated to match user's requirement
                )
                print("Collection created")
            except Exception as e:
                if "already exists" in str(e):
                    print("Collection already exists - skipping creation")
                else:
                    print(f"Collection error: {e}")

    def add_texts(self, texts: List[str], payloads: List[Dict], embeddings: List[List[float]] = None) -> List[str]:
        """Add texts to the vector store with their embeddings"""
        if not self.client:
            # If no client, return dummy IDs for testing purposes
            import uuid
            return [str(uuid.uuid4()) for _ in texts]

        # Verify the client has the upsert method
        if not hasattr(self.client, 'upsert'):
            print(f"Qdrant client does not have 'upsert' method. Available methods: {[attr for attr in dir(self.client) if not attr.startswith('_')]}")
            import uuid
            return [str(uuid.uuid4()) for _ in texts]

        import uuid
        from qdrant_client.http.models import PointStruct

        # If embeddings are not provided, generate dummy vectors
        if embeddings is None:
            embeddings = [[0.0] * 1024 for _ in texts]  # Placeholder vectors

        # Generate unique IDs for each text
        ids = [str(uuid.uuid4()) for _ in texts]

        # Create points
        points = [
            PointStruct(
                id=ids[i],
                vector=embeddings[i],
                payload=payloads[i]
            ) for i in range(len(texts))
        ]

        try:
            # Upload to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
        except Exception as e:
            print(f"Error during Qdrant upsert: {e}")

        return ids

    def search(self, query_vector: List[float], top_k: int = 10) -> List[Dict]:
        """Search for similar texts using vector search"""
        # Check if client is initialized
        if not self.client:
            # If no client due to missing credentials, return dummy results for testing
            return []

        # Check if the client has the required methods
        has_query_points_method = hasattr(self.client, 'query_points')
        has_search_method = hasattr(self.client, 'search')

        # If neither method exists, return empty results
        if not (has_query_points_method or has_search_method):
            print(f"Qdrant client does not have 'query_points' or 'search' method. Available methods: {[attr for attr in dir(self.client) if not attr.startswith('_')]}")
            return []

        try:
            if has_query_points_method:
                # Use the new 'query_points' method for vector search (Qdrant v1.9.0+)
                # Fixed to avoid duplicate with_payload parameter issue by using positional args
                search_result = self.client.query_points(
                    collection_name=self.collection_name,
                    query=query_vector,
                    limit=top_k
                )
                # For the 'query_points' method, results are in a different format
                # The payload should still be available if we didn't explicitly exclude it
                return [
                    {
                        "id": result.id,
                        "payload": result.payload,
                        "score": result.score
                    }
                    for result in search_result.points
                ]
            elif has_search_method:
                # Use the older 'search' method as fallback
                results = self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    limit=top_k,
                    with_payload=True  # Ensure payload is returned with search results
                )

                return [
                    {
                        "id": result.id,
                        "payload": result.payload,
                        "score": result.score
                    }
                    for result in results
                ]
        except Exception as e:
            print(f"Error during Qdrant search: {e}")
            # Return empty results if both methods fail
            return []

    def query_text(self, query_text: str, top_k: int = 6) -> List[Dict]:
        """Search using text query (auto-embedding) - for QdrantFastembedMixin"""
        # Check if client is initialized
        if not self.client:
            # If no client due to missing credentials, return dummy results for testing
            return []

        # Check if the client has the query method for text-based search (auto-embedding)
        has_query_method = hasattr(self.client, 'query')

        if not has_query_method:
            print(f"Qdrant client does not have 'query' method for text search. Available methods: {[attr for attr in dir(self.client) if not attr.startswith('_')]}")
            # Fall back to vector search approach using the embedding generator
            from ..utils.embeddings import EmbeddingGenerator
            from ..services.cohere_service import CohereService
            embedding_gen = EmbeddingGenerator(CohereService())
            query_vector = embedding_gen.generate_embedding_for_single_text(query_text, input_type="search_query")
            return self.search(query_vector, top_k)

        try:
            # Use the 'query' method with query_text for auto-embedding (Qdrant v1.9.0+)
            # Updated to use the correct parameter format and address deprecation warning
            # Using the correct models from Qdrant client
            from qdrant_client.http import models
            # Check if TextQuery exists, otherwise use query with a simple string
            if hasattr(models, 'TextQuery'):
                search_result = self.client.query(
                    collection_name=self.collection_name,
                    query=models.TextQuery(text=query_text),
                    limit=top_k
                )
                # For the 'query' method, results are in QueryResponse format
                return [
                    {
                        "id": result.id,
                        "payload": result.payload,
                        "score": result.score
                    }
                    for result in search_result.points
                ]
            else:
                # If TextQuery doesn't exist, fall back to vector search
                from ..utils.embeddings import EmbeddingGenerator
                from ..services.cohere_service import CohereService
                embedding_gen = EmbeddingGenerator(CohereService())
                query_vector = embedding_gen.generate_embedding_for_single_text(query_text, input_type="search_query")
                return self.search(query_vector, top_k)
        except Exception as e:
            print(f"Error during Qdrant text query: {e}")
            # Fall back to vector search approach using the embedding generator
            from ..utils.embeddings import EmbeddingGenerator
            from ..services.cohere_service import CohereService
            embedding_gen = EmbeddingGenerator(CohereService())
            try:
                query_vector = embedding_gen.generate_embedding_for_single_text(query_text, input_type="search_query")
                return self.search(query_vector, top_k)
            except Exception as fallback_error:
                print(f"Fallback vector search also failed: {fallback_error}")
                # Return empty results if both methods fail
                return []

    def get_point(self, point_id: str) -> Optional[Dict]:
        """Get a specific point by ID"""
        if not self.client:
            # If no client, return None for testing purposes
            return None

        # Verify the client has the retrieve method
        if not hasattr(self.client, 'retrieve'):
            print(f"Qdrant client does not have 'retrieve' method. Available methods: {[attr for attr in dir(self.client) if not attr.startswith('_')]}")
            return None

        try:
            results = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[point_id]
            )

            if results:
                result = results[0]
                return {
                    "id": result.id,
                    "payload": result.payload
                }
        except Exception as e:
            print(f"Error during Qdrant retrieve: {e}")

        return None