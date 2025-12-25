# API Contract: RAG Chatbot for AI-Native Textbook

## Overview
This API provides endpoints for querying the RAG-based chatbot embedded in the AI-Native Textbook. It supports both general queries and queries based on selected text from the textbook.

## Base URL
`https://api.textbook-rag-chatbot.com`

## Authentication
API requests are stateless and do not require authentication for basic functionality. However, clients may pass a session token to maintain conversation history.

## Endpoints

### POST /query
Submit a general query about the textbook content.

#### Request
```json
{
  "question": "string (the user's question)",
  "session_token": "string (optional, for maintaining conversation history)"
}
```

#### Response
```json
{
  "response_id": "UUID",
  "answer": "string (the AI-generated answer)",
  "citations": [
    {
      "section": "string (name of the cited section)",
      "url": "string (URL to the section)",
      "text": "string (quoted text from the source)"
    }
  ],
  "confidence": "float (confidence score between 0 and 1)",
  "response_time_ms": "integer"
}
```

#### Success Response (200 OK)
- **200**: Query processed successfully with a relevant response

#### Error Responses
- **400**: Invalid request format or missing required fields
- **422**: Query contains content that cannot be processed
- **500**: Internal server error during processing
- **503**: Service temporarily unavailable (e.g., LLM API down)

### POST /query-selected
Submit a query based on selected text from the textbook.

#### Request
```json
{
  "question": "string (the user's question)",
  "selected_text": "string (the text selected by the user)",
  "session_token": "string (optional, for maintaining conversation history)"
}
```

#### Response
```json
{
  "response_id": "UUID",
  "answer": "string (the AI-generated answer based only on selected text)",
  "citations": [
    {
      "section": "string (name of the cited section)",
      "url": "string (URL to the section)",
      "text": "string (quoted text from the source)"
    }
  ],
  "confidence": "float (confidence score between 0 and 1)",
  "response_time_ms": "integer"
}
```

#### Success Response (200 OK)
- **200**: Query processed successfully with a relevant response based only on selected text

#### Error Responses
- **400**: Invalid request format or missing required fields
- **422**: Query contains content that cannot be processed or selected text is too long
- **500**: Internal server error during processing
- **503**: Service temporarily unavailable (e.g., LLM API down)

### GET /health
Check the health status of the service.

#### Response
```json
{
  "status": "string (overall health status)",
  "timestamp": "ISO 8601 datetime",
  "dependencies": {
    "cohere": "string (status of Cohere API)",
    "qdrant": "string (status of Qdrant vector store)"
  }
}
```

#### Success Response (200 OK)
- **200**: Service is healthy and all dependencies are accessible

#### Error Responses
- **503**: Service is unhealthy (one or more dependencies are down)

### GET /stats
Get statistics about the service usage.

#### Response
```json
{
  "total_queries": "integer",
  "avg_response_time_ms": "float",
  "queries_last_24h": "integer",
  "accuracy_rate": "float (percentage of accurate responses)",
  "timestamp": "ISO 8601 datetime"
}
```

#### Success Response (200 OK)
- **200**: Statistics retrieved successfully

#### Error Responses
- **500**: Internal server error retrieving statistics