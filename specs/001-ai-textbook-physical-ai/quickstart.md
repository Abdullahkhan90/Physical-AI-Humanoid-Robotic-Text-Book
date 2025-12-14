# Quickstart Guide: AI Native Textbook on Physical AI & Humanoid Robotics

**Date**: 2025-12-12  
**Feature**: AI Native Textbook on Physical AI & Humanoid Robotics  
**Branch**: `001-ai-textbook-physical-ai`

## Overview
This guide provides instructions for setting up the development environment to work on the AI Native Textbook on Physical AI & Humanoid Robotics project. The project consists of a Docusaurus-based frontend for the textbook content and a FastAPI-based backend for the RAG chatbot functionality.

## Prerequisites
- Node.js (v18 or higher)
- Python (v3.9 or higher)
- Git
- Access to a Qdrant vector database (either local instance or cloud)
- (Optional) Docker for containerized development

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
git checkout 001-ai-textbook-physical-ai
```

### 2. Set Up the Frontend (Docusaurus)
```bash
# Navigate to the docusaurus directory
cd docusaurus

# Install dependencies
npm install

# Create environment file
cp .env.example .env

# Edit .env to add any required configuration
# (currently no special configuration needed for basic setup)
```

### 3. Set Up the Backend (RAG Chatbot)
```bash
# Navigate to the chatbot directory
cd chatbot

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create environment file
cp .env.example .env

# Edit .env to add your Qdrant configuration
# Example:
# QDRANT_URL=http://localhost:6333
# QDRANT_API_KEY=your-api-key-if-any
```

### 4. Environment Variables
Create the following environment files:

**docusaurus/.env**
```
# No special configuration needed for basic setup
# Add any custom API keys or settings here if needed
```

**chatbot/.env**
```
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-api-key-if-any
QDRANT_COLLECTION_NAME=textbook_content
DEBUG=true
```

### 5. Running Locally

#### Frontend Only (for content development)
```bash
cd docusaurus
npm start
```
The textbook will be available at http://localhost:3000

#### Backend Only (for RAG chatbot development)
```bash
cd chatbot
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn api.main:app --reload --port 8000
```
The RAG chatbot API will be available at http://localhost:8000

#### Full Stack
In separate terminals:
```bash
# Terminal 1: Start Qdrant (if using local instance)
docker run -p 6333:6333 -v $(pwd)/qdrant_storage:/qdrant/storage qdrant/qdrant

# Terminal 2: Start the backend
cd chatbot
source venv/bin/activate
uvicorn api.main:app --reload --port 8000

# Terminal 3: Start the frontend
cd docusaurus
npm start
```

### 6. Adding Content
Content for the textbook is organized by modules in the `docusaurus/docs/` directory:

```
docusaurus/
├── docs/
│   ├── module-1-ros2/      # ROS 2 content
│   ├── module-2-digital-twin/  # Gazebo & Unity content
│   ├── module-3-ai-brain/      # NVIDIA Isaac content
│   ├── module-4-vla/           # Vision-Language-Action content
│   └── assets/          # Images, diagrams, and other assets
```

To add new content:
1. Create a new Markdown file in the appropriate module directory
2. Add the content following the Docusaurus documentation format
3. Update the `sidebar.js` file to include the new content in the navigation

### 7. Initializing the RAG Knowledge Base
To populate the RAG system with textbook content:

```bash
cd chatbot
source venv/bin/activate
python scripts/index_textbook_content.py
```

This script will read the textbook content from the Docusaurus docs directory, chunk it appropriately, generate embeddings, and store it in the Qdrant vector database.

### 8. Running Tests
```bash
# Frontend tests
cd docusaurus
npm test

# Backend tests
cd chatbot
source venv/bin/activate
pytest
```

### 9. Building for Production
```bash
# Build the frontend
cd docusaurus
npm run build

# The built files will be in the `build` directory
# Deploy these to GitHub Pages

# For the backend, containerize with Docker or deploy directly
```

## Troubleshooting

1. **Qdrant Connection Issues**: Ensure the Qdrant service is running and accessible at the configured URL
2. **Content Not Loading**: Check that the Docusaurus sidebar configuration includes your new content
3. **API Calls Failing**: Verify that the frontend is properly configured to call the backend API
4. **Translation Issues**: Check that Urdu translations are properly placed in the `i18n/ur/` directory

## Next Steps
- Review the detailed architecture documentation in `plan.md`
- Check the data model in `data-model.md`
- Look at the API contracts in `contracts/`
- Explore the implementation tasks in `tasks.md` (after running `/sp.tasks`)