import os
import argparse
from typing import List
from ingestion.loaders import MarkdownLoader, TextLoader
from ingestion.processors import ContentCleaner, ContentStructurer
from src.services.qdrant_service import QdrantService
from src.services.cohere_service import CohereService
from src.utils.text_splitter import TextSplitter
from src.utils.embeddings import EmbeddingGenerator
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    parser = argparse.ArgumentParser(description="Ingest textbook content into vector store")
    parser.add_argument("--source", type=str, required=True, help="Source directory containing textbook content")
    parser.add_argument("--target", type=str, default="qdrant", help="Target vector store (currently only qdrant is supported)")

    args = parser.parse_args()

    # Validate source directory
    if not os.path.isdir(args.source):
        raise ValueError(f"Source directory does not exist: {args.source}")

    logger.info(f"Starting ingestion from {args.source} to {args.target}")

    # Initialize services
    qdrant_service = QdrantService()
    cohere_service = CohereService()
    text_splitter = TextSplitter()
    embedding_generator = EmbeddingGenerator(cohere_service)

    # Determine which loader to use based on file types
    loader = MarkdownLoader()  # Default to Markdown loader for textbook content

    # Load documents
    logger.info("Loading documents...")
    documents = loader.load(args.source)
    logger.info(f"Loaded {len(documents)} documents")

    # Process documents
    logger.info("Processing documents...")
    cleaner = ContentCleaner()
    documents = cleaner.process(documents)

    structurer = ContentStructurer()
    documents = structurer.process(documents)
    logger.info("Documents processed")

    # Split documents into chunks
    logger.info("Splitting documents into chunks...")
    all_chunks = []
    for doc in documents:
        content = doc['content']
        metadata = doc['metadata']

        # Add document-specific metadata to each chunk
        chunks = text_splitter.split_text(content, metadata)
        all_chunks.extend(chunks)

    logger.info(f"Split into {len(all_chunks)} text chunks")

    # Generate embeddings for chunks
    logger.info("Generating embeddings...")
    embeddings = embedding_generator.generate_embeddings(all_chunks)
    logger.info("Embeddings generated")

    # Prepare payloads for Qdrant with rich metadata
    payloads = []
    for i, chunk in enumerate(all_chunks):
        # Extract rich metadata from the chunk's metadata
        chunk_metadata = chunk.metadata.copy()

        # Ensure we have rich metadata for citations
        rich_metadata = {
            "text": chunk.text,  # Include the text in metadata for citation purposes
            "file_path": chunk_metadata.get('file_path', ''),
            "file_name": chunk_metadata.get('file_name', ''),
            "directory": chunk_metadata.get('directory', ''),
            "source": chunk_metadata.get('source', ''),
            "section": chunk_metadata.get('section', ''),
            "chapter": chunk_metadata.get('chapter', ''),
            "url": chunk_metadata.get('url', ''),
            "title": chunk_metadata.get('title', ''),
            "sections": chunk_metadata.get('sections', []),
            "full_metadata": chunk_metadata  # Keep all original metadata
        }

        payload = {
            "text": chunk.text,
            "metadata": rich_metadata
        }
        payloads.append(payload)

    # Extract just the text content for embedding
    texts = [chunk.text for chunk in all_chunks]

    # Add texts, embeddings, and payloads to Qdrant
    logger.info("Adding to vector store...")
    qdrant_service.add_texts(texts, payloads, embeddings)
    logger.info("Ingestion completed successfully!")


if __name__ == "__main__":
    main()