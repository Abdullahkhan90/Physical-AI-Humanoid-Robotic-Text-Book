# Quickstart Guide: RAG Chatbot for AI-Native Textbook

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus)
- Access to Cohere API (with the API key provided)
- Access to Qdrant Cloud (with the cluster details provided)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd Physical-AI-Humanoid-Robotic-Text-Book
```

### 2. Backend Setup
```bash
# Navigate to the backend directory
cd backend

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create environment file
cp .env.example .env

# Edit .env with your credentials:
# COHERE_API_KEY=PHqQyG8aNDMWlBOswUIhHXmcBNpaqZ2wLW4DnSIr
# QDRANT_CLUSTER_ID=044f23c3-5b09-4644-bc92-76606856da36
# QDRANT_ENDPOINT=https://044f23c3-5b09-4644-bc92-76606856da36.us-east4-0.gcp.cloud.qdrant.io
# QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.INTSFmlRaK6Z_r4aTYvc7LUbGvwYreCllPlsCx6BAm0
```

### 3. Ingest Textbook Content
```bash
# Run the ingestion script to process textbook content into the vector store
python -m ingestion.main --source docs/ --target qdrant
```

### 4. Start the Backend
```bash
# Run the FastAPI server
uvicorn src.api.main:app --reload --port 8000
```

### 5. Frontend Setup
```bash
# In a new terminal, navigate to the docusaurus directory
cd docusaurus

# Install dependencies
npm install

# Start the Docusaurus development server
npm start
```

## Usage

### Interacting with the Chatbot
1. Navigate to the Docusaurus textbook site in your browser (typically http://localhost:3000)
2. You'll see a floating chatbot icon on the pages
3. Click the icon to open the chat interface
4. Type your question about Physical AI and Humanoid Robotics
5. For selected-text queries, highlight text in the textbook and click the "Ask about selected text" option that appears

### API Direct Usage
If you want to interact with the API directly:

```bash
# General query
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain the basics of humanoid robotics",
    "session_token": "optional-session-token"
  }'

# Selected text query
curl -X POST http://localhost:8000/query-selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the key challenges mentioned in this text?",
    "selected_text": "Humanoid robots face several challenges in locomotion and balance control...",
    "session_token": "optional-session-token"
  }'
```

## Testing

### Run Backend Tests
```bash
# From the backend directory
pytest tests/
```

### Check Service Health
```bash
curl http://localhost:8000/health
```

## Deployment

### Backend (to Vercel or Render)
1. Set environment variables in your deployment platform
2. Deploy the FastAPI application following your platform's instructions

### Frontend (to Vercel or GitHub Pages)
1. Build the Docusaurus site: `npm run build`
2. Deploy the `build/` directory to your hosting platform

## Troubleshooting

### Common Issues

1. **API Keys Not Working**:
   - Verify your Cohere and Qdrant credentials are correct
   - Check that the API keys have not expired

2. **Ingestion Fails**:
   - Verify the source directory contains readable documents
   - Check that the Qdrant cluster is accessible

3. **No Citations in Responses**:
   - Ensure the ingestion completed successfully
   - Check that the vector store contains the expected content