# Research Findings: RAG Chatbot for AI-Native Textbook

## Technology Decisions

### Backend Framework: FastAPI
**Decision**: Use FastAPI for the backend due to its high performance, built-in async support, and excellent integration with Python data science libraries.
**Rationale**: FastAPI offers excellent performance for RAG applications and provides automatic API documentation. It also has great support for AI/ML workflows and async processing.
**Alternatives considered**: Flask, Django - FastAPI was chosen for performance and async capabilities which are important for LLM API calls.

### LLM Provider: Cohere
**Decision**: Use Cohere's language models as mandated by the project constitution.
**Rationale**: Project constraints require Cohere API usage specifically (no OpenAI or other LLM dependencies). Cohere Command-R+ is optimal for RAG applications due to its RAG capabilities and tool usage support.
**Alternatives considered**: None - mandated by project requirements.

### Vector Database: Qdrant Cloud
**Decision**: Use Qdrant Cloud for vector storage due to its efficient similarity search capabilities.
**Rationale**: Qdrant offers excellent performance for similarity search which is crucial for RAG applications. The cloud version provides managed infrastructure and scalability.
**Alternatives considered**: Pinecone, Weaviate, Chroma - Qdrant chosen based on project constraints.

### Metadata Storage: Qdrant Payloads
**Decision**: Store metadata (file_path, chapter, section) in Qdrant vector store payloads.
**Rationale**: Eliminates the need for an additional database while keeping metadata closely coupled with vector embeddings, simplifying architecture and reducing costs.
**Alternatives considered**: Separate metadata database - Qdrant payloads chosen to reduce complexity and dependencies.

### Frontend Framework: Docusaurus
**Decision**: Integrate chatbot into existing Docusaurus textbook site.
**Rationale**: The project requires embedding the chatbot in an existing Docusaurus-powered textbook site. This provides seamless integration with educational content.
**Alternatives considered**: Next.js, React standalone - Docusaurus integration chosen as required by project.

## Architecture Patterns

### RAG Implementation
**Decision**: Implement a RAG (Retrieval-Augmented Generation) system with vector search.
**Rationale**: RAG is the optimal approach for ensuring responses are grounded in the textbook content, preventing hallucinations.
**Alternatives considered**: Pure LLM generation (rejected due to hallucination risk), fine-tuning (not feasible due to time constraints).

### Text Chunking Strategy
**Decision**: Use intelligent text chunking (500-800 tokens) that preserves document hierarchy.
**Rationale**: Proper chunking ensures context is preserved while maintaining efficient retrieval. Preserving document structure allows for better citations.
**Alternatives considered**: Fixed-size chunking without context preservation (rejected as it would lose important contextual information).

### Caching Strategy
**Decision**: Implement caching for frequently requested content and embeddings.
**Rationale**: Caching will help optimize token usage and reduce response times for common queries, which is important for staying within free-tier limits.
**Alternatives considered**: No caching (rejected as it would increase costs and response times).

## Security Considerations

### API Key Management
**Decision**: Store API keys securely in environment variables, never expose to frontend.
**Rationale**: Security requirement mandates API keys are never exposed to the client. Environment variables are the standard approach for this.
**Alternatives considered**: Storing in config files (rejected as less secure), hardcoding (rejected as insecure).

### Data Privacy
**Decision**: Implement automatic chat history purging after a specified duration.
**Rationale**: Balances the need for session continuity with privacy protection as required by the project constraints.
**Alternatives considered**: Permanent storage (rejected due to privacy concerns), session-only storage (potentially insufficient for debugging).

## Performance Optimizations

### Response Streaming
**Decision**: Implement streaming responses for improved user experience.
**Rationale**: Streaming provides immediate feedback to users and improves perceived response time.
**Alternatives considered**: Full response only (rejected as it provides worse UX).

### Embedding Optimization
**Decision**: Use efficient embedding models and implement caching to minimize API calls.
**Rationale**: Crucial for staying within free-tier limits while maintaining performance.
**Alternatives considered**: No optimization (not feasible given budget constraints).