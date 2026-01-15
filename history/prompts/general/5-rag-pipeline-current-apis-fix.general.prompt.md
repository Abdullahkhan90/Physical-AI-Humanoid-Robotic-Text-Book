---
id: 5
title: rag-pipeline-current-apis-fix
stage: refactor
date: 2025-12-20
surface: agent
model: "unknown"
feature: "none"
branch: "main"
user: "user"
command: "/sp.implement"
labels: ["backend", "api", "chatbot", "rag", "qdrant", "cohere", "current-apis"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "backend/backend/src/services/qdrant_service.py"
  - "backend/backend/src/services/cohere_service.py"
  - "backend/backend/tests/unit/main.py"
prompt_text: "Fix RAG pipeline to use current Qdrant and Cohere APIs: Qdrant query method instead of search, and Cohere command-r model instead of deprecated models."
response_text: "Successfully updated the RAG pipeline to use current APIs:\n\n1. Updated Qdrant service to use query method with correct parameters\n2. Updated Cohere service to use 'command-r' model instead of deprecated models\n3. Added fallback to 'command-r-08-2024' if primary model fails\n4. Updated test file to use correct Cohere model\n\nThe RAG pipeline now works with current API versions and provides proper textbook answers with citations."
outcome: "RAG pipeline now uses current Qdrant and Cohere APIs. Chatbot provides textbook-style responses with citations when APIs are available."
evaluation: "All API calls now use current, supported methods and models. The system properly handles the updated API specifications."
---

# RAG Pipeline Current APIs Fix

## Summary of Changes

Updated the RAG pipeline to use current Qdrant and Cohere APIs:

- Updated Qdrant service to use query method instead of deprecated search method
- Updated Cohere service to use 'command-r' model instead of deprecated models
- Added fallback model in case primary model is unavailable
- Updated test files to use correct API specifications

## Technical Details

### Qdrant Service Update
- Simplified query method call with correct parameters
- Removed complex error handling since we're using the correct API now
- Maintained fallback to search method if query is not available

### Cohere Service Update
- Changed from deprecated models ('command-r-plus', 'command', 'command-light') 
- Updated to current 'command-r' model
- Added fallback to 'command-r-08-2024' if primary model fails
- Maintained error handling and retry logic

### Test Files Update
- Updated test file to use correct Cohere model

## Impact

- RAG pipeline now works with current API versions
- Eliminates 404 errors from deprecated models
- Provides proper textbook answers with citations
- Maintains robust error handling for service disruptions