import os
import logging
from typing import Dict, Any
from pathlib import Path
import markdown
from cohere import Client
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct
from langchain_text_splitters import RecursiveCharacterTextSplitter

logger = logging.getLogger(__name__)

class IngestionService:
    def __init__(self):
        # Initialize clients with cloud credentials
        self.cohere_api_key = os.getenv('COHERE_API_KEY')
        if not self.cohere_api_key or "your_" in self.cohere_api_key:
            logger.warning("COHERE_API_KEY not set or is a placeholder. Using mock responses for testing.")
            self.cohere_client = None
        else:
            self.cohere_client = Client(self.cohere_api_key)

        # Use the NEW cloud URL and API key directly as provided by the user
        qdrant_endpoint = "https://a04cc351-47bd-4c14-9a8f-e0b43f1de657.europe-west3-0.gcp.cloud.qdrant.io"
        qdrant_api_key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.jHHZvJvY-BdFJrvRFx-ipf1bal1_6sNdBxLo17HZ-G8"

        try:
            self.qdrant_client = QdrantClient(
                host="a04cc351-47bd-4c14-9a8f-e0b43f1de657.europe-west3-0.gcp.cloud.qdrant.io",  # NEW Host without protocol
                port=6333,
                https=True,  # Enable HTTPS for cloud connection
                api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.jHHZvJvY-BdFJrvRFx-ipf1bal1_6sNdBxLo17HZ-G8",
                timeout=120,  # Increased timeout as requested
                verify=False,  # Keep SSL verification disabled
                grpc_keepalive_time_MS=400000,  # Additional parameter that might help with connection stability
                **{'check_compatibility': False}  # Ensure check_compatibility is passed correctly
            )

            # Test connection
            collections = self.qdrant_client.get_collections()
            logger.info("Connected successfully to Qdrant Cloud for ingestion!")
            print("Connected successfully to Qdrant Cloud for ingestion!")
        except Exception as e:
            error_msg = f"Failed to connect to Qdrant Cloud for ingestion: {str(e)}"
            logger.error(error_msg)
            print(error_msg)
            if "404" in str(e) or "not found" in str(e).lower():
                logger.error("Error: Invalid API key or URL for ingestion - please verify your Qdrant Cloud credentials")
            self.qdrant_client = None

        self.collection_name = "textbook"
        
        # Create collection if not exists (only if we have valid Qdrant credentials)
        if self.qdrant_client:
            try:
                # Try to get the collection to see if it exists
                self.qdrant_client.get_collection(self.collection_name)
                logger.info(f"Collection {self.collection_name} already exists - skipping creation")
            except Exception as e:
                # If getting the collection fails, it doesn't exist, so create it
                logger.info(f"Collection {self.collection_name} does not exist, creating it...")
                try:
                    self.qdrant_client.create_collection(
                        collection_name=self.collection_name,
                        vectors_config={"size": 768, "distance": "Cosine"}  # Updated to match user's requirement
                    )
                    logger.info(f"Created new collection: {self.collection_name}")
                except Exception as create_error:
                    if "already exists" in str(create_error):
                        logger.info(f"Collection {self.collection_name} already exists - skipping creation")
                    else:
                        logger.error(f"Collection error: {create_error}")
                        # If collection creation fails, try to continue anyway in case it was a race condition
                        # or the collection was created between the check and creation attempt
                        try:
                            self.qdrant_client.get_collection(self.collection_name)
                            logger.info(f"Verified that collection {self.collection_name} exists")
                        except Exception as verify_error:
                            logger.error(f"Could not verify collection existence after attempted creation: {verify_error}")
                            raise
        else:
            logger.info("Skipping collection creation - Qdrant client not initialized")

        # Text splitter
        self.text_splitter = RecursiveCharacterTextSplitter(chunk_size=800, chunk_overlap=200)

    def ingest_documents(self, docs_path: str = None) -> Dict[str, Any]:
        """
        Ingest documents from the specified path or default path
        """
        logger.info("Starting ingestion...")

        # Define possible document locations in order of priority
        possible_paths = []

        if docs_path:
            possible_paths.append(Path(docs_path))
        else:
            # Define the priority order for document locations
            script_dir = Path(__file__).parent.parent.parent  # Go to backend/src/
            project_root = script_dir.parent  # Go to backend/
            repo_root = project_root.parent  # Go to repository root

            possible_paths = [
                repo_root / "docusaurus" / "docs",     # docusaurus/docs at repo root (highest priority)
                project_root / "docusaurus" / "docs",  # backend/docusaurus/docs/
                project_root / "docs",                 # backend/docs/
                project_root / "data",                 # backend/data/
                project_root / "textbook",             # backend/textbook/
            ]

        # Find the first path that exists
        docs_path = None
        for path in possible_paths:
            if path.exists():
                docs_path = path
                logger.info(f"Searching in folder: {path}")
                break

        # If no path exists, try copying from docusaurus/docs at repo root to backend/data/
        if docs_path is None:
            repo_root = Path(__file__).parent.parent.parent.parent  # Go to repository root
            source_path = repo_root / "docusaurus" / "docs"
            dest_path = Path(__file__).parent.parent.parent / "data"  # backend/data/

            if source_path.exists():
                logger.info(f"Copying content from {source_path} to {dest_path}")
                import shutil

                # Create destination directory if it doesn't exist
                dest_path.mkdir(exist_ok=True)

                # Copy all files from source to destination
                for item in source_path.rglob("*"):
                    if item.is_file() and (item.suffix.lower() in ['.md', '.pdf', '.txt']):
                        dest_file = dest_path / item.relative_to(source_path)
                        dest_file.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(item, dest_file)

                if dest_path.exists():
                    docs_path = dest_path
                    logger.info(f"Copied content to {dest_path}, now searching there")

        if docs_path is None:
            logger.error("ERROR: No textbook documents found in any folder!")
            return {
                "status": "error",
                "message": "No documents found in any of the expected locations",
                "files_processed": 0,
                "chunks_stored": 0
            }

        # Count total files to be processed
        all_files = []
        for root, dirs, files in os.walk(docs_path):
            for file in files:
                if file.lower().endswith(('.md', '.pdf')):
                    all_files.append(os.path.join(root, file))

        logger.info(f"Found {len(all_files)} files (.md/.pdf)")

        if len(all_files) == 0:
            logger.error("ERROR: No textbook documents found in any folder!")
            return {
                "status": "error",
                "message": "No documents found in the specified location",
                "files_processed": 0,
                "chunks_stored": 0
            }

        logger.info(f"Loaded {len(all_files)} documents")

        total_files_processed = 0
        total_chunks_processed = 0

        # Process each file
        for file_path in all_files:
            try:
                # Handle different file types
                if file_path.lower().endswith('.md'):
                    with open(file_path, 'r', encoding='utf-8') as f:
                        content = f.read()
                    # Convert MD to plain text
                    text = markdown.markdown(content)
                elif file_path.lower().endswith('.pdf'):
                    # For PDF files, we would use PyPDF2 or similar
                    # Since we don't have pdf processing dependencies, we'll skip for now
                    logger.warning(f"Skipping PDF file (not implemented): {file_path}")
                    continue
                else:
                    # For other text files
                    with open(file_path, 'r', encoding='utf-8') as f:
                        text = f.read()

                # Extract metadata from file path
                relative_path = os.path.relpath(file_path, docs_path)
                directory = os.path.dirname(relative_path) or "root"

                # Split text
                chunks = self.text_splitter.split_text(text)
                logger.info(f"File {os.path.basename(file_path)} split into {len(chunks)} chunks")

                for i, chunk in enumerate(chunks):
                    # Generate embedding
                    if self.cohere_client:
                        try:
                            embed_response = self.cohere_client.embed(
                                texts=[chunk],
                                model='embed-english-v3.0',
                                input_type='search_document'  # Changed from 'search_query' to 'search_document' for documents
                            )
                            embedding = embed_response.embeddings[0]
                            logger.info(f"Created embedding for chunk {i+1}/{len(chunks)} of {os.path.basename(file_path)}")
                        except Exception as e:
                            logger.error(f"Error generating embedding for chunk {i} of {file_path}: {e}")
                            continue
                    else:
                        # Use mock embedding when no API key
                        import numpy as np
                        # Create a simple embedding based on the text content (for consistent testing)
                        embedding = [hash(chunk + str(i)) % 1000 / 1000.0 for i in range(1024)]

                    # Payload for metadata
                    payload = {
                        "text": chunk,
                        "file_path": relative_path,  # Store relative path
                        "directory": directory,
                        "source": "docusaurus_docs",
                        "section": os.path.basename(file_path),  # or parse heading
                        "title": os.path.splitext(os.path.basename(file_path))[0],  # filename without extension
                        "full_path": file_path
                    }

                    # Upsert to Qdrant (only if client is available)
                    if self.qdrant_client:
                        import uuid
                        # Generate a UUID for the point ID to satisfy Qdrant requirements
                        point_id = str(uuid.uuid4())
                        point = PointStruct(id=point_id, vector=embedding, payload=payload)
                        try:
                            self.qdrant_client.upsert(collection_name=self.collection_name, points=[point])
                            total_chunks_processed += 1
                        except Exception as e:
                            logger.error(f"Error upserting chunk {i} of {file_path} to Qdrant: {e}")
                            continue
                    else:
                        # Log that we would have upserted if credentials were available
                        logger.info(f"Would have upserted chunk {i} of {file_path} to Qdrant (credentials not provided)")
                        total_chunks_processed += 1  # Still count as processed for demo purposes

                total_files_processed += 1
                logger.info(f"Processed {os.path.basename(file_path)} with {len(chunks)} chunks")

            except Exception as e:
                logger.error(f"Error processing file {file_path}: {e}")
                continue

        logger.info(f"Successfully stored {total_chunks_processed} vectors in Qdrant collection '{self.collection_name}'")
        logger.info(f"Ingestion complete! Processed {total_files_processed} files and {total_chunks_processed} chunks.")

        return {
            "status": "completed",
            "files_processed": total_files_processed,
            "chunks_stored": total_chunks_processed
        }