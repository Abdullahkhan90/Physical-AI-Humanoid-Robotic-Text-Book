from typing import List, Dict, Any
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http import models
from ..models.rag_model import RAGKnowledgeBase
from ..models.citation_model import Citation
from ..utils.config import config
from ..utils.logging import get_logger
import hashlib
import asyncio


class RAGService:
    """
    Service for implementing the RAG (Retrieval-Augmented Generation) processing pipeline
    """

    def __init__(self):
        self.logger = get_logger(__name__)
        self.encoder = SentenceTransformer('all-MiniLM-L6-v2')  # Lightweight model for embeddings
        self.client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY,
            timeout=10
        )
        self.collection_name = config.QDRANT_COLLECTION_NAME

    def chunk_text(self, text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
        """
        Split text into overlapping chunks for better retrieval
        """
        if len(text) <= chunk_size:
            return [text]
        
        chunks = []
        start = 0
        
        while start < len(text):
            end = start + chunk_size
            
            # Try to break at sentence boundaries
            if end < len(text):
                # Find the nearest sentence ending
                sentence_end = text.rfind('.', start, end)
                if sentence_end != -1 and sentence_end > start + chunk_size // 2:
                    end = sentence_end + 1
                else:
                    # If no sentence ending found, try paragraph
                    para_end = text.rfind('\n\n', start, end)
                    if para_end != -1 and para_end > start + chunk_size // 2:
                        end = para_end
                    # Otherwise, just break at chunk_size
            
            chunks.append(text[start:end].strip())
            
            # Move start forward with overlap
            start = end - overlap if end - overlap > start else end
            if start >= len(text):
                break
        
        # Remove empty chunks
        chunks = [chunk for chunk in chunks if chunk]
        return chunks

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a given text
        """
        try:
            embedding = self.encoder.encode([text])[0]
            return embedding.tolist()
        except Exception as e:
            self.logger.error(f"Error generating embedding for text: {str(e)[:100]}...")
            return []

    async def index_content(self, module_id: str, content_id: str, text: str, metadata: Dict[str, Any] = None) -> bool:
        """
        Index a content piece in the RAG knowledge base
        """
        try:
            # Split text into chunks
            chunks = self.chunk_text(text)
            
            # Process each chunk
            points = []
            for i, chunk in enumerate(chunks):
                # Create a unique ID for this chunk
                chunk_id = f"{content_id}_chunk_{i}"
                
                # Generate embedding for the chunk
                embedding = self.generate_embedding(chunk)
                
                if not embedding:
                    self.logger.warning(f"Failed to generate embedding for chunk {chunk_id}")
                    continue
                
                # Prepare metadata
                chunk_metadata = {
                    "module_id": module_id,
                    "content_id": content_id,
                    "chunk_index": i,
                    "original_text_length": len(chunk)
                }
                
                if metadata:
                    chunk_metadata.update(metadata)
                
                # Create a point for Qdrant
                point = models.PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload={
                        "module_id": module_id,
                        "content_id": content_id,
                        "chunk_text": chunk,
                        "metadata": chunk_metadata
                    }
                )
                
                points.append(point)
            
            if points:
                # Upload points to Qdrant
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )
                
                self.logger.info(f"Indexed {len(points)} chunks for content {content_id} in module {module_id}")
                return True
            else:
                self.logger.warning(f"No valid chunks to index for content {content_id}")
                return False
                
        except Exception as e:
            self.logger.error(f"Error indexing content {content_id}: {str(e)}")
            return False

    async def retrieve_relevant_chunks(self, query: str, limit: int = 5, module_filter: str = None) -> List[Dict[str, Any]]:
        """
        Retrieve relevant text chunks for a given query
        """
        try:
            # Generate embedding for the query
            query_embedding = self.generate_embedding(query)
            
            if not query_embedding:
                self.logger.error("Failed to generate embedding for query")
                return []
            
            # Prepare filter conditions
            search_filter = None
            if module_filter:
                search_filter = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="module_id",
                            match=models.MatchValue(value=module_filter)
                        )
                    ]
                )
            
            # Search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=search_filter,
                limit=limit,
                with_payload=True
            )
            
            # Process results
            relevant_chunks = []
            for result in search_results:
                chunk_data = {
                    "id": result.id,
                    "text": result.payload.get("chunk_text", ""),
                    "module_id": result.payload.get("module_id", ""),
                    "content_id": result.payload.get("content_id", ""),
                    "metadata": result.payload.get("metadata", {}),
                    "score": result.score  # Similarity score
                }
                relevant_chunks.append(chunk_data)
            
            self.logger.info(f"Retrieved {len(relevant_chunks)} relevant chunks for query")
            return relevant_chunks
            
        except Exception as e:
            self.logger.error(f"Error retrieving relevant chunks: {str(e)}")
            return []

    async def process_query(self, query: str, module_filter: str = None) -> Dict[str, Any]:
        """
        Process a query through the RAG pipeline
        """
        try:
            # Retrieve relevant chunks
            relevant_chunks = await self.retrieve_relevant_chunks(query, module_filter=module_filter)

            if not relevant_chunks:
                self.logger.info(f"No relevant content found for query: {query[:50]}...")
                return {
                    "query": query,
                    "answer": "I couldn't find relevant content in the textbook to answer your question.",
                    "sources": [],
                    "citations": [],
                    "confidence": 0.0
                }

            # Combine relevant chunks to form context
            context = "\n\n".join([chunk["text"] for chunk in relevant_chunks])

            # For now, we'll return the context as the answer (in a real implementation,
            # this would be processed by an LLM to generate a synthesized answer)
            answer = f"Based on the textbook content:\n\n{context[:1000]}..."  # Truncate to prevent overly long answers

            # Prepare sources information
            sources = []
            citations = []  # Will store citation information
            for chunk in relevant_chunks:
                source_info = {
                    "moduleId": chunk["module_id"],
                    "contentId": chunk["content_id"],
                    "title": f"Content {chunk['content_id']}",  # Would come from content title in real implementation
                    "relevanceScore": chunk["score"]
                }
                sources.append(source_info)

                # Extract citation information if available
                # In a real implementation, this would come from the indexed content
                # For now, we'll create mock citations
                if chunk["content_id"] == "content-1-1":
                    citations.append({
                        "id": "cit-001",
                        "title": "A survey of robot software frameworks",
                        "authors": ["Smith, J.", "Johnson, A."],
                        "source": "Journal of Robotics",
                        "year": 2022,
                        "doi": "10.1155/2022/1234567",
                        "url": "https://www.hindawi.com/journals/jr/2022/1234567/",
                        "citationType": "journalArticle",
                        "apaFormatted": "Smith, J., & Johnson, A. (2022). A survey of robot software frameworks. Journal of Robotics. https://doi.org/10.1155/2022/1234567",
                        "relevanceScore": chunk["score"]
                    })

            # Calculate confidence based on the highest similarity score
            max_score = max([chunk["score"] for chunk in relevant_chunks]) if relevant_chunks else 0
            confidence = min(max_score, 1.0)  # Normalize to 0-1 range

            self.logger.info(f"Processed query with {len(relevant_chunks)} relevant chunks, confidence: {confidence:.2f}")

            return {
                "query": query,
                "answer": answer,
                "sources": sources,
                "citations": citations,
                "confidence": confidence
            }

        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            return {
                "query": query,
                "answer": "An error occurred while processing your query. Please try again.",
                "sources": [],
                "citations": [],
                "confidence": 0.0
            }