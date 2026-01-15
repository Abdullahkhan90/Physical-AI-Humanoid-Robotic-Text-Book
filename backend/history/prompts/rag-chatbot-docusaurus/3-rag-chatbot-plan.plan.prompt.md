---
id: 3
title: "rag-chatbot-plan"
stage: "plan"
date_iso: "2025-12-18"
surface: "agent"
model: "default"
feature: "rag-chatbot-docusaurus"
branch: "001-rag-chatbot-docusaurus"
user: "user"
command: "/sp.plan"
labels: ["plan", "rag", "chatbot", "docusaurus"]
links:
  spec: "../spec.md"
  plan: "./plan.md"
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "plan.md"
  - "research.md"
  - "data-model.md"
  - "quickstart.md"
  - "contracts/api-contract.md"
tests_yaml: []
---

# Implementation Plan: RAG Chatbot for AI-Native Textbook

## PROMPT_TEXT

```
Integrated RAG Chatbot for AI-Native Textbook on Physical AI and Humanoid Robotics
Project Overview:
Develop a high-performance, embedded RAG chatbot inside a Docusaurus-based AI-native textbook for the GIAIC hackathon. The chatbot will exclusively use textbook content for retrieval, support both general queries and selected-text queries, and provide accurate, cited, educational responses. All development must use SpecifyKit Plus for scaffolding and Qwen CLI for workflows, with Cohere for generation, Qdrant Cloud for vector storage, and Neon Serverless Postgres for metadata.
Key Credentials (store securely in .env – never commit to repo):
COHERE_API_KEY=PHqQyG8aNDMWlBOswUIhHXmcBNpaqZ2wLW4DnSIr
QDRANT_CLUSTER_ID=044f23c3-5b09-4644-bc92-76606856da36
QDRANT_ENDPOINT=https://044f23c3-5b09-4644-bc92-76606856da36.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.INTSFmlRaK6Z_r4aTYvc7LUbGvwYreCllPlsCx6BAm0
NEON_DB_URL=postgresql://neondb_owner:npg_aIsd6tx2qXHc@ep-old-cherry-ah4a2zc0-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
Step-by-Step Execution Plan:

Project Initialization & Scaffolding (0-4 hours)
• Use SpecifyKit Plus to scaffold the full project structure.
• Create/extend Docusaurus site for the textbook.
• Set up monorepo folders: /ingestion (content processing), /backend (FastAPI), /frontend (Docusaurus customizations).
• Initialize Git repo and configure .env with all credentials.
• Set up Qwen CLI as primary development tool.
Content Ingestion Pipeline (4-12 hours)
• Write ingestion script (Python via Qwen CLI) to recursively read all Markdown files from Docusaurus docs/.
• Chunk text intelligently (500-800 tokens, preserve headings/section hierarchy).
• Add rich metadata: chapter, section, file_path, headings.
• Generate embeddings using Cohere embed-english-v3.0 or embed-multilingual-v3.0.
• Create Qdrant collection (if not exists) and upsert vectors + payloads.
• Store chunk-to-source mapping in Neon Postgres for fast citation lookup.
Backend Development – FastAPI RAG Server (12-24 hours)
• Build FastAPI app with two main endpoints:
POST /query: General query from user input
POST /query-selected: Query with user-selected text as additional context
• Retrieval logic: Vector search in Qdrant (top-k=8-12), optional hybrid with keyword.
• RAG prompt engineering: Force citation of sources, educational tone, no external knowledge.
• Generation using Cohere (command-r-plus preferred for quality and tool use).
• Response format: Include answer + list of cited sections (with links if possible).
• Add streaming response support and proper error handling.

Frontend Integration – Chatbot UI in Docusaurus (24-36 hours)
• Develop React chatbot component (floating button → sidebar/modal).
• Implement global text selection listener across all doc pages.
• On text selection: Show “Ask about selected text” floating action.
• Connect frontend to FastAPI backend (CORS enabled).
• Display responses with Markdown rendering and clickable source references that scroll to the section.
• Ensure fully mobile-responsive and accessible design.
Testing & Optimization (36-42 hours)
• Prepare 50+ test queries covering:
General topics
Selected-text scenarios
Edge cases (ambiguous, out-of-scope, very specific)
• Verify ≥95% accuracy, zero hallucinations, proper citations.
• Optimize token usage and response time (<5 seconds average).
• Stress test within free-tier limits.

Deployment & Documentation (42-48 hours)
• Deploy Docusaurus site to Vercel or GitHub Pages.
• Deploy FastAPI backend to Vercel (serverless) or Render free tier.
• Configure environment variables on deployment platform.
• Write comprehensive README:
Project overview
Setup & ingestion instructions
Running locally
Credentials guide
• Record demo video showcasing selected-text feature and accuracy.

Final Polish & Hackathon Submission (48-52 hours)
• Add loading indicators, error messages, and retry logic.
• Fine-tune prompts for maximum educational value.
• Prepare pitch: Highlight innovation, accuracy, and impact on learning Physical AI & Robotics.
• Submit repository link and live demo URL.

Milestones:
• End of Day 1: Ingestion pipeline complete + Qdrant populated
• End of Day 2: Backend fully functional + basic chatbot working
• End of Day 3: Selected-text feature + testing + live deployment
Deliverables:
• Fully deployed live textbook with embedded interactive RAG chatbot
• Clean, well-documented GitHub repository
• Demo video and hackathon submission assets
• Zero-cost, free-tier compliant implementation
```

## RESPONSE_TEXT

Successfully completed the implementation planning for the RAG Chatbot for AI-Native Textbook:

- Updated the implementation plan with detailed technical context
- Performed constitution check to ensure compliance with project principles
- Designed project structure with backend (FastAPI) and frontend (Docusaurus) components
- Completed Phase 0 research with findings on technology decisions and architecture patterns
- Designed data models for Query, Retrieved Context, Generated Response, and Chat Session entities
- Created API contracts for query endpoints and health checks
- Developed a comprehensive quickstart guide
- Updated agent context with project-specific information

The plan is comprehensive and aligned with the project's constitution, ensuring grounded generation, context fidelity, hallucination zero-tolerance, and other core principles.

## OUTCOME

A complete implementation plan has been created with all necessary design artifacts. The project is ready to proceed to the tasks creation phase where specific implementation tasks will be defined based on this plan.

## EVALUATION

- All required planning phases completed (Phase 0 research, Phase 1 design)
- All artifacts generated (plan.md, research.md, data-model.md, contracts/, quickstart.md)
- Constitution compliance verified
- Agent context updated with project-specific information
- Project structure designed appropriately for the requirements