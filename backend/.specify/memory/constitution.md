<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: PRINCIPLE_1_NAME → Grounded Generation, PRINCIPLE_2_NAME → Context Fidelity, PRINCIPLE_3_NAME → Hallucination Zero-Tolerance, PRINCIPLE_4_NAME → User Trust & Clarity, PRINCIPLE_5_NAME → Academic Integrity, PRINCIPLE_6_NAME → Model & AI Standards
- Added sections: Response Rules, Technical Constraints, Interaction Standards, Security & Privacy, Success Criteria, Evaluation Metrics
- Removed sections: None
- Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
- Follow-up TODOs: None
-->

# Integrated RAG Chatbot for an Interactive AI Textbook Constitution

## Core Principles

### Grounded Generation
All responses must be derived exclusively from retrieved book content. The system must not generate answers based on general knowledge or external databases.

### Context Fidelity
When user-selected text is provided, answers must rely only on that text. The system must respect the context boundaries set by the user's selection.

### Hallucination Zero-Tolerance
The system must not fabricate facts, explanations, or references. Any uncertainty must result in a clear refusal rather than a potentially incorrect response.

### User Trust & Clarity
Responses must clearly state when information is unavailable in the source. Users must be informed when no relevant content is found in the book.

### Academic Integrity
Maintain textbook-level clarity and correctness. All responses should reflect the pedagogical quality of academic materials.

### Model & AI Standards
Use only approved technologies: Cohere for language models, Qdrant Cloud for vector storage, and ensure embedding consistency between indexing and querying.

## Response Rules

- Use only retrieved passages from the book
- Do not use external knowledge or general world facts
- Clearly reference relevant sections or excerpts when possible
- If answer is not found, reply with: "The selected text or book content does not contain enough information to answer this question."

## Technical Constraints

- Backend Framework: FastAPI
- Vector Database: Qdrant Cloud (Free Tier)
- Relational Database: Neon Serverless PostgreSQL
- Prompt Governance: Specifikit+
- CLI Orchestration & Testing: Qwen CLI
- API Provider: Cohere only

## Interaction Standards

- Responses must be concise, accurate, and reader-friendly
- Tone should match an academic textbook assistant
- Avoid speculation, assumptions, or extrapolation beyond the text

## Security & Privacy

- Do not store sensitive user data
- Store chat history only for session continuity and debugging
- Ensure API keys are never exposed to the client

## Success Criteria

- Zero hallucinations detected during evaluation
- All answers traceable to retrieved book content
- Correct handling of text-selection-based queries
- Clear refusal behavior when context is missing
- Smooth, interactive user experience within the book interface

## Evaluation Metrics

- Answer accuracy
- Context relevance
- Latency of retrieval
- User trust and interpretability
- Compliance with RAG constraints

## Governance

This constitution governs all development practices for the RAG Chatbot project. All implementation decisions must align with these principles. Amendments require documentation of the change rationale and approval from the project stakeholders. Any deviation from these principles must be justified and documented.

All PRs and reviews must verify compliance with these principles. Code that violates the core principles (e.g., introduces hallucinations, bypasses context fidelity) must be rejected. Use this constitution as the primary reference for decision-making during development.

**Version**: 1.1.0 | **Ratified**: 2025-12-18 | **Last Amended**: 2025-12-18
