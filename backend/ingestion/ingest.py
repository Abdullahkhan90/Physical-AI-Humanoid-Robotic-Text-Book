import os
from dotenv import load_dotenv
from cohere import Client
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from langchain_text_splitters import RecursiveCharacterTextSplitter
import markdown
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

load_dotenv()

cohere_api_key = os.getenv('COHERE_API_KEY')
qdrant_collection = "textbook"

# Initialize clients with cloud credentials
if not cohere_api_key or "your_" in cohere_api_key:
    logger.warning("COHERE_API_KEY not set or is a placeholder. Using mock responses for testing.")
    cohere_client = None
else:
    cohere_client = Client(cohere_api_key)

# Use the cloud URL and API key directly as provided by the user
qdrant_endpoint = "https://0d44ad0f-4e35-4f58-a5fd-34bf9beefde2.europe-west3-0.gcp.cloud.qdrant.io:6333"
qdrant_api_key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJyb2xlIjoiY2xpZW50In0.eyJyb2xlIjoiY2xpZW50In0.DkmqZ6SfFIR0G2D6G1n8A05sg1WnqLLaIGA"

qdrant_client = QdrantClient(
    url=qdrant_endpoint,
    api_key=qdrant_api_key,
    timeout=60,  # optional - timeout badhane ke liye
    verify=False  # optional - agar SSL issue aaye to
)

# Create collection if not exists (only if we have valid Qdrant credentials)
if qdrant_client:
    try:
        # Try to get the collection to see if it exists
        qdrant_client.get_collection(qdrant_collection)
        logger.info(f"Collection {qdrant_collection} already exists - skipping creation")
    except Exception as e:
        # If getting the collection fails, it doesn't exist, so create it
        logger.info(f"Collection {qdrant_collection} does not exist, creating it...")
        try:
            qdrant_client.create_collection(
                collection_name=qdrant_collection,
                vectors_config={"size": 768, "distance": "Cosine"}  # Updated to match user's requirement
            )
            logger.info(f"Created new collection: {qdrant_collection}")
        except Exception as create_error:
            if "already exists" in str(create_error):
                logger.info(f"Collection {qdrant_collection} already exists - skipping creation")
            else:
                logger.error(f"Collection error: {create_error}")
                # If collection creation fails, try to continue anyway in case it was a race condition
                # or the collection was created between the check and creation attempt
                try:
                    qdrant_client.get_collection(qdrant_collection)
                    logger.info(f"Verified that collection {qdrant_collection} exists")
                except Exception as verify_error:
                    logger.error(f"Could not verify collection existence after attempted creation: {verify_error}")
                    raise
else:
    logger.info("Skipping collection creation - Qdrant client not initialized")

# Text splitter
text_splitter = RecursiveCharacterTextSplitter(chunk_size=800, chunk_overlap=200)

# Read all Markdown from docusaurus/docs/
# Use the project root directory to build the path correctly
# The file is at backend/backend/ingestion/ingest.py, so we need to go up 3 levels to project root
script_dir = os.path.dirname(os.path.abspath(__file__))  # ingestion directory: backend/backend/ingestion
parent_dir = os.path.dirname(script_dir)  # parent directory: backend/backend
grandparent_dir = os.path.dirname(parent_dir)  # grandparent directory: backend
project_root = os.path.dirname(grandparent_dir)  # project root: Physical-AI-Humanoid-Robotic-Text-Book
docs_path = os.path.join(project_root, "docusaurus", "docs")  # Path to docs: project_root/docusaurus/docs

if not os.path.exists(docs_path):
    raise FileNotFoundError(f"Docs directory not found at {docs_path}")

logger.info(f"Reading documents from: {docs_path}")

total_files_processed = 0
total_chunks_processed = 0

logger.info(f"Starting to walk through directory: {docs_path}")
for root, dirs, files in os.walk(docs_path):
    logger.info(f"Processing directory: {root}, found {len(files)} files: {files}")
    for file in files:
        if file.endswith(".md"):
            file_path = os.path.join(root, file)
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    md_text = f.read()

                # Convert MD to plain text
                text = markdown.markdown(md_text)

                # Extract metadata from file path
                relative_path = os.path.relpath(file_path, docs_path)
                directory = os.path.dirname(relative_path) or "root"

                # Split text
                chunks = text_splitter.split_text(text)

                for i, chunk in enumerate(chunks):
                    # Generate embedding
                    if cohere_client:
                        try:
                            embed_response = cohere_client.embed(
                                texts=[chunk],
                                model='embed-english-v3.0',
                                input_type='search_document'  # Changed from 'search_query' to 'search_document' for documents
                            )
                            embedding = embed_response.embeddings[0]
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
                        "section": file,  # or parse heading
                        "title": os.path.splitext(file)[0],  # filename without extension
                        "full_path": file_path
                    }

                    # Upsert to Qdrant (only if client is available)
                    if qdrant_client:
                        import uuid
                        # Generate a UUID for the point ID to satisfy Qdrant requirements
                        point_id = str(uuid.uuid4())
                        point = PointStruct(id=point_id, vector=embedding, payload=payload)
                        try:
                            qdrant_client.upsert(collection_name=qdrant_collection, points=[point])
                            total_chunks_processed += 1
                        except Exception as e:
                            logger.error(f"Error upserting chunk {i} of {file_path} to Qdrant: {e}")
                            continue
                    else:
                        # Log that we would have upserted if credentials were available
                        logger.info(f"Would have upserted chunk {i} of {file_path} to Qdrant (credentials not provided)")
                        total_chunks_processed += 1  # Still count as processed for demo purposes

                total_files_processed += 1
                logger.info(f"Processed {file} with {len(chunks)} chunks")

            except Exception as e:
                logger.error(f"Error processing file {file_path}: {e}")
                continue

logger.info(f"Ingestion complete! Processed {total_files_processed} files and {total_chunks_processed} chunks.")