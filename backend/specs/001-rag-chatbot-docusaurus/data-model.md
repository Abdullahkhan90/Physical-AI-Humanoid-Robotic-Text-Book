# Data Model: RAG Chatbot for AI-Native Textbook

## Entities

### Query
- **id**: UUID (primary key)
- **question_text**: String (the user's question)
- **selected_text**: String (optional text selected by user for context)
- **context_used**: String (text that was used for retrieval)
- **timestamp**: DateTime (when the query was submitted)
- **user_id**: UUID (optional, for tracking user sessions)

### Retrieved Context
- **id**: UUID (primary key)
- **query_id**: UUID (foreign key to Query)
- **source_text**: String (the text retrieved from the textbook)
- **metadata**: JSON (information about where the text came from: chapter, section, file_path, headings)
- **similarity_score**: Float (how similar this context is to the query)
- **created_at**: DateTime

### Generated Response
- **id**: UUID (primary key)
- **query_id**: UUID (foreign key to Query)
- **response_text**: String (the AI-generated response)
- **citations**: Array[String] (list of cited sections from the textbook)
- **confidence_score**: Float (confidence in the accuracy of the response)
- **created_at**: DateTime
- **response_time_ms**: Integer (time taken to generate the response)

### Chat Session
- **id**: UUID (primary key)
- **session_token**: String (identifier for the user's session)
- **user_id**: UUID (optional, for registered users)
- **created_at**: DateTime
- **last_activity**: DateTime
- **queries**: Array[UUID] (list of query IDs in this session)

## Relationships

1. **Query** (1) → (1) **Chat Session**: Each query belongs to one chat session
2. **Query** (1) → (N) **Retrieved Context**: Each query can retrieve multiple context chunks
3. **Query** (1) → (1) **Generated Response**: Each query generates one response

## Validation Rules

1. **Query Validation**:
   - Question text must be 1-1000 characters
   - Selected text (if provided) must be 1-5000 characters
   - Must not contain malicious code or inappropriate content

2. **Retrieved Context Validation**:
   - Source text must not exceed 4000 characters
   - Metadata must contain at least source file and section information
   - Similarity score must be between 0 and 1

3. **Generated Response Validation**:
   - Response text must be provided and not empty
   - Citations must reference actual textbook sections
   - Confidence score must be between 0 and 1

## State Transitions

### Query State Transitions
- `PENDING`: Query received, retrieval process starting
- `RETRIEVING`: Context is being retrieved from vector store
- `GENERATING`: AI is generating response based on context
- `COMPLETED`: Response generated and returned to user
- `FAILED`: Error occurred during processing

## Indexes

1. **Query.timestamp**: For time-based queries and analytics
2. **Retrieved Context.query_id**: For linking contexts with queries
3. **Generated Response.query_id**: For linking responses with queries
4. **Chat Session.session_token**: For quickly finding user sessions
5. **Retrieved Context.similarity_score**: For efficient retrieval ranking