# Running Instructions for RAG Chatbot Project

This document provides the complete instructions to run the RAG chatbot project for the Physical AI & Humanoid Robotics textbook.

## Prerequisites

1. Python 3.11+ installed
2. Node.js and npm installed
3. Your API keys for Cohere and Qdrant

## Setup

### 1. Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd C:\Users\Lenovo\Desktop\Physical-AI-Humanoid-Robotic-Text-Book\backend\backend
   ```

2. Activate the virtual environment (if using one):
   ```bash
   # If using the existing virtual environment:
   .\.venv\Scripts\activate
   ```

3. Install required Python packages (if not already installed):
   ```bash
   pip install -r requirements.txt
   ```

4. Set up your environment variables:
   - Copy `.env.example` to `.env`
   - Add your actual API keys:
     ```
     COHERE_API_KEY=your_actual_cohere_api_key
     QDRANT_ENDPOINT=your_actual_qdrant_endpoint
     QDRANT_API_KEY=your_actual_qdrant_api_key
     ```

### 2. Frontend Setup

1. Navigate to the docusaurus directory:
   ```bash
   cd C:\Users\Lenovo\Desktop\Physical-AI-Humanoid-Robotic-Text-Book\docusaurus
   ```

2. Install required packages:
   ```bash
   npm install
   ```

## Running the Application

### 1. Run Ingestion

1. Make sure you're in the backend directory:
   ```bash
   cd C:\Users\Lenovo\Desktop\Physical-AI-Humanoid-Robotic-Text-Book\backend\backend
   ```

2. Run the ingestion script:
   ```bash
   python -m ingestion.ingest
   ```

   This will process all the markdown files in the docusaurus/docs directory and store them in the Qdrant vector database.

### 2. Start Backend Server

1. Make sure you're in the backend directory:
   ```bash
   cd C:\Users\Lenovo\Desktop\Physical-AI-Humanoid-Robotic-Text-Book\backend\backend
   ```

2. Start the backend server:
   ```bash
   uvicorn src.api.main:app --reload --port 8000
   ```

### 3. Start Docusaurus Frontend

1. Navigate to the docusaurus directory:
   ```bash
   cd C:\Users\Lenovo\Desktop\Physical-AI-Humanoid-Robotic-Text-Book\docusaurus
   ```

2. Start the Docusaurus development server:
   ```bash
   npm start
   ```

## Complete Command Sequence

Here are the commands to run the entire application in order:

1. **Activate virtual environment (if needed)**:
   ```bash
   cd C:\Users\Lenovo\Desktop\Physical-AI-Humanoid-Robotic-Text-Book\backend\backend
   .\.venv\Scripts\activate
   ```

2. **Run ingestion**:
   ```bash
   python -m ingestion.ingest
   ```

3. **Start backend server** (in a new terminal):
   ```bash
   cd C:\Users\Lenovo\Desktop\Physical-AI-Humanoid-Robotic-Text-Book\backend\backend
   .\.venv\Scripts\activate
   uvicorn src.api.main:app --reload --port 8000
   ```

4. **Start Docusaurus frontend** (in another new terminal):
   ```bash
   cd C:\Users\Lenovo\Desktop\Physical-AI-Humanoid-Robotic-Text-Book\docusaurus
   npm start
   ```

## Features of the RAG Chatbot

- **Floating Chatbot**: Located in the upper right corner of the screen
- **Dark Theme Support**: Automatically adapts to the user's color preference
- **Header**: "Physical AI Assistant" with appropriate subtext
- **Initial Messages**: "Hello! I am your Physical AI assistant. Ask me anything about the course!" and "Sure! Ask me anything about the Physical AI course."
- **Input Placeholder**: "Type a message..."
- **Text Selection**: Users can select text in the textbook and ask questions about the selected text
- **Citations**: Responses include citations to specific sections of the textbook
- **CORS Support**: Properly configured for localhost:3000
- **Error Handling**: Graceful error handling with user-friendly messages

## Troubleshooting

1. **Ingestion fails with "No module named 'langchain.text_splitter'"**: Make sure you've installed langchain-text-splitters:
   ```bash
   pip install langchain-text-splitters
   ```

2. **Backend endpoints return errors**: Check that your .env file contains valid API keys

3. **Chatbot doesn't appear**: Make sure the Chatbot component is included in your Docusaurus layout

4. **CORS errors**: Verify that the origins in src/api/main.py match your frontend URL