---
id: 1
title: "const-update"
stage: "constitution"
date_iso: "2025-12-18"
surface: "agent"
model: "default"
feature: "none"
branch: "main"
user: "user"
command: "/sp.constitution"
labels: ["constitution", "update", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - ".specify/memory/constitution.md"
tests_yaml: []
---

# Update to Project Constitution

## PROMPT_TEXT

```
Create or update the project constitution at `.specify/memory/constitution.md` for a RAG Chatbot project with the following requirements:

Project: Integrated RAG Chatbot for an Interactive AI Textbook

Purpose:
To develop a highly reliable, context-grounded Retrieval-Augmented Generation (RAG) chatbot embedded within a published book. The chatbot must assist readers by answering questions strictly based on the book's content or user-selected text, ensuring academic accuracy, transparency, and zero hallucination.

Core Principles:
- Grounded Generation: All responses must be derived exclusively from retrieved book content.
- Context Fidelity: When user-selected text is provided, answers must rely only on that text.
- Hallucination Zero-Tolerance: The system must not fabricate facts, explanations, or references.
- User Trust & Clarity: Responses must clearly state when information is unavailable in the source.
- Academic Integrity: Maintain textbook-level clarity and correctness.

Model & AI Standards:
- Language Model: Cohere (no OpenAI models permitted).
- Retrieval Mechanism: Vector similarity search via Qdrant Cloud.
- Embedding Consistency: Same embedding model used for indexing and querying.
- Generation Constraint: If retrieved context is insufficient, respond with a refusal.

Response Rules:
- Use only retrieved passages from the book.
- Do not use external knowledge or general world facts.
- Clearly reference relevant sections or excerpts when possible.
- If answer is not found, reply with:
  "The selected text or book content does not contain enough information to answer this question."

Technical Constraints:
- Backend Framework: FastAPI
- Vector Database: Qdrant Cloud (Free Tier)
- Relational Database: Neon Serverless PostgreSQL
- Prompt Governance: Specifikit+
- CLI Orchestration & Testing: Qwen CLI
- API Provider: Cohere only

Interaction Standards:
- Responses must be concise, accurate, and reader-friendly.
- Tone should match an academic textbook assistant.
- Avoid speculation, assumptions, or extrapolation beyond the text.

Security & Privacy:
- Do not store sensitive user data.
- Store chat history only for session continuity and debugging.
- Ensure API keys are never exposed to the client.

Success Criteria:
- Zero hallucinations detected during evaluation.
- All answers traceable to retrieved book content.
- Correct handling of text-selection-based queries.
- Clear refusal behavior when context is missing.
- Smooth, interactive user experience within the book interface.

Evaluation Metrics:
- Answer accuracy
- Context relevance
- Latency of retrieval
- User trust and interpretability
- Compliance with RAG constraints
```

## RESPONSE_TEXT

Constitution file updated successfully with all specified requirements. The updated constitution now includes:

- Core principles: Grounded Generation, Context Fidelity, Hallucination Zero-Tolerance, User Trust & Clarity, Academic Integrity, and Model & AI Standards
- Response Rules that enforce using only retrieved passages and refusing when content is unavailable
- Technical Constraints specifying FastAPI, Qdrant Cloud, Neon PostgreSQL, Cohere, etc.
- Interaction Standards for reader-friendly and academically appropriate responses
- Security & Privacy guidelines
- Success Criteria and Evaluation Metrics

Version incremented to 1.1.0 with appropriate governance information added.

## OUTCOME

The constitution has been successfully created for the RAG Chatbot project with all specified requirements implemented. The constitution now serves as the authoritative guide for development practices in the project.

## EVALUATION

- All specified core principles have been incorporated
- Technical constraints and standards are clearly defined
- Response rules ensure zero hallucination as required
- Security and privacy requirements are documented
- Success criteria and evaluation metrics are established
- Version control and governance information is properly set up