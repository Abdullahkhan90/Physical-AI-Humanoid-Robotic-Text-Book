#!/usr/bin/env python3
"""
Script to index textbook content in the RAG knowledge base
This script processes markdown content from the docusaurus docs directory,
chunks it appropriately, generates embeddings, and stores it in the Qdrant vector database.
"""

import os
import glob
import asyncio
from pathlib import Path
import markdown
from bs4 import BeautifulSoup
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import hashlib
from datetime import datetime


class ContentIndexer:
    """
    Class to handle the indexing of textbook content into the RAG knowledge base
    """
    
    def __init__(self):
        # Initialize the embedding model
        self.encoder = SentenceTransformer('all-MiniLM-L6-v2')
        
        # Initialize Qdrant client - in a real implementation, these would come from config
        self.client = QdrantClient(
            url=os.getenv("QDRANT_URL", "http://localhost:6333"),
            api_key=os.getenv("QDRANT_API_KEY"),
            timeout=10
        )
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "textbook_content")
        
        # Create the collection if it doesn't exist
        self._ensure_collection_exists()
    
    def _ensure_collection_exists(self):
        """
        Ensure the Qdrant collection exists with the proper configuration
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
            
            if not collection_exists:
                # Create a new collection for textbook content
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=384,  # Dimension of sentence-transformer embeddings (MiniLM)
                        distance=models.Distance.COSINE
                    )
                )
                
                # Create payload index for faster filtering
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="module_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )
                
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="content_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )
                
                print(f"Created collection '{self.collection_name}' with proper configuration.")
        except Exception as e:
            print(f"Error ensuring collection exists: {e}")
            raise
    
    def extract_text_from_markdown(self, md_content: str) -> str:
        """
        Extract plain text from markdown content
        """
        # Convert markdown to HTML
        html = markdown.markdown(md_content)
        
        # Parse HTML and extract text
        soup = BeautifulSoup(html, 'html.parser')
        text = soup.get_text()
        
        # Clean up the text
        text = ' '.join(text.split())
        
        return text
    
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
            print(f"Error generating embedding for text: {str(e)[:100]}...")
            return []

    def get_module_id_from_path(self, filepath: str) -> str:
        """
        Extract module ID from the file path
        """
        path_parts = Path(filepath).parts
        module_part = ""
        
        for part in path_parts:
            if part.startswith("module-"):
                module_part = part
                break
        
        # Convert to a standard module ID format
        if module_part:
            # Extract just the numeric part and name (e.g., "module-1-ros2" -> "module-1-ros2")
            return module_part
        else:
            return "unknown-module"

    async def process_file(self, filepath: str) -> int:
        """
        Process a single markdown file and add its content to the knowledge base
        """
        print(f"Processing file: {filepath}")
        
        try:
            # Read the file
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Extract plain text from markdown
            plain_text = self.extract_text_from_markdown(content)
            
            # Extract module ID from path
            module_id = self.get_module_id_from_path(filepath)
            
            # Generate content ID based on file path
            content_id = hashlib.md5(filepath.encode()).hexdigest()[:12]
            
            # Split text into chunks
            chunks = self.chunk_text(plain_text)
            
            # Process each chunk
            points = []
            for i, chunk in enumerate(chunks):
                # Create a unique ID for this chunk
                chunk_id = f"{content_id}_chunk_{i}"
                
                # Generate embedding for the chunk
                embedding = self.generate_embedding(chunk)
                
                if not embedding:
                    print(f"Failed to generate embedding for chunk {chunk_id}")
                    continue
                
                # Prepare metadata
                chunk_metadata = {
                    "module_id": module_id,
                    "content_id": content_id,
                    "original_file": filepath,
                    "chunk_index": i,
                    "original_text_length": len(chunk)
                }
                
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
                
                print(f"Indexed {len(points)} chunks from {filepath}")
                return len(points)
            else:
                print(f"No valid chunks to index for {filepath}")
                return 0
                
        except Exception as e:
            print(f"Error processing file {filepath}: {str(e)}")
            return 0

    async def index_directory(self, directory: str = "docusaurus/docs") -> int:
        """
        Index all markdown files in the specified directory
        """
        total_points = 0
        
        # Find all markdown files in the directory
        md_files = glob.glob(os.path.join(directory, "**/*.md"), recursive=True)
        print(f"Found {len(md_files)} markdown files to index")
        
        for md_file in md_files:
            points_added = await self.process_file(md_file)
            total_points += points_added
        
        print(f"Indexing complete! Added a total of {total_points} points to the knowledge base.")
        return total_points


async def main():
    """
    Main function to run the indexing process
    """
    # Create indexer instance
    indexer = ContentIndexer()
    
    # Index all content in the docusaurus docs directory
    total_points = await indexer.index_directory()
    
    print(f"\nIndexing process completed. Total points added: {total_points}")


if __name__ == "__main__":
    # Run the indexing process
    asyncio.run(main())