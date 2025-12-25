# RAG Chatbot Project - Running Instructions

## Project Setup and Execution

Follow these steps to run the complete RAG chatbot system for the AI-Native Textbook on Physical AI & Humanoid Robotics:

### 1. Activate Virtual Environment

```bash
cd backend/backend
uv venv
source .venv/Scripts/activate  # On Windows
# or
source .venv/bin/activate     # On Linux/Mac
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Set Up Environment Variables

Create a `.env` file in the `backend/backend` directory with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_CLUSTER_ID=your_qdrant_cluster_id_here
QDRANT_ENDPOINT=your_qdrant_endpoint_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

### 4. Run Data Ingestion

```bash
cd backend/backend
python -m ingestion.main --source ../../docusaurus/docs
```

This will:
- Read Markdown files from the Docusaurus docs directory
- Process and chunk the content
- Store rich metadata in Qdrant payload (no external database)
- Generate embeddings using Cohere

### 5. Start Backend Server

```bash
cd backend/backend
uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload
```

The backend API will be available at `http://localhost:8000`

### 6. Start Docusaurus Frontend

```bash
cd docusaurus
npm install  # if dependencies haven't been installed
npm run start
```

The frontend will be available at `http://localhost:3000`

## Complete System Features

### Textbook Content
- The system uses the Docusaurus documentation from `../docusaurus/docs/`
- Content is ingested and stored in Qdrant vector database with rich metadata

### RAG Chatbot
- Available on the homepage of the Docusaurus site
- Supports both general queries and selected-text queries
- Connects to the backend API at `http://localhost:8000`

### Text Selection Feature
- Select any text on the page to provide context for your query
- The chatbot will automatically detect selected text
- Uses the `/api/query-selected` endpoint when text is selected
- Uses the `/api/query` endpoint for general questions

### Citations
- Responses include citations to specific sections of the textbook
- Citations come directly from Qdrant payload metadata
- No external database required

## API Endpoints

- `GET /` - API root
- `GET /health` - Health check
- `POST /api/query` - General question answering
- `POST /api/query-selected` - Question answering with selected text context

Request format for `/api/query`:
```json
{
  "question": "Your question here"
}
```

Request format for `/api/query-selected`:
```json
{
  "question": "Your question here",
  "selected_text": "The selected text to provide context"
}
```

Response format:
```json
{
  "response_id": "uuid",
  "answer": "The AI-generated answer",
  "citations": [
    {
      "section": "Section name",
      "url": "URL to the section",
      "text": "Preview of the cited text"
    }
  ],
  "confidence": 0.85,
  "response_time_ms": 1200
}
```

## Troubleshooting

1. **CORS Issues**: The backend is configured to allow all origins for local development. In production, restrict origins appropriately.

2. **API Keys**: Ensure all required API keys are set in the `.env` file.

3. **Qdrant Connection**: Verify that the Qdrant endpoint and API key are correct.

4. **Ingestion Issues**: Make sure the source directory contains Markdown files and the path is correct.

## Development Notes

- The system has been optimized for Windows with Python 3.13
- All dependencies use pre-compiled wheels (no source builds or Rust needed)
- No Neon Postgres or SQLAlchemy dependencies remain
- All metadata is stored directly in Qdrant payloads
- The React chatbot component includes text selection functionality
- The system prioritizes a fully working live demo with textbook + RAG chatbot + selected-text query + accurate citations