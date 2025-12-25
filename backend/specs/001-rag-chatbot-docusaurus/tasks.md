---

description: "Task list for RAG Chatbot for AI-Native Textbook implementation"
---

# Tasks: RAG Chatbot for AI-Native Textbook

**Input**: Design documents from `/specs/001-rag-chatbot-docusaurus/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are included as part of the implementation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `docusaurus/src/`
- Paths shown below assume the web application structure from plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure with backend/ and docusaurus/ directories
- [X] T002 Initialize Python project with FastAPI dependencies in backend/
- [X] T003 Initialize Node.js project with Docusaurus dependencies in docusaurus/
- [X] T004 [P] Create requirements.txt with FastAPI, Cohere SDK, Qdrant client
- [X] T005 [P] Create .env.example with COHERE_API_KEY, QDRANT_CLUSTER_ID, QDRANT_ENDPOINT, QDRANT_API_KEY
- [X] T006 Create initial directory structure for backend/src/models/, backend/src/services/, backend/src/api/, backend/src/utils/
- [X] T007 Create initial directory structure for docusaurus/src/components/Chatbot/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T008 Setup database connection and models in backend/src/models/
- [X] T009 [P] Implement Qdrant vector store connection in backend/src/services/qdrant_service.py
- [X] T010 [P] Implement Qdrant metadata handling in backend/src/services/qdrant_service.py
- [X] T011 Create base models for Query, Retrieved Context, Generated Response, and Chat Session in backend/src/models/
- [X] T012 Setup FastAPI app structure with routing in backend/src/api/main.py
- [X] T013 [P] Configure CORS middleware for frontend-backend communication
- [X] T014 Setup environment variable loading in backend/src/config.py
- [X] T015 [P] Create base API endpoints structure in backend/src/api/routes/
- [X] T016 Setup Cohere API integration in backend/src/services/cohere_service.py
- [X] T017 Implement text splitting utilities in backend/src/utils/text_splitter.py
- [X] T018 Implement embedding generation utilities in backend/src/utils/embeddings.py
- [X] T019 Create ingestion pipeline structure in backend/ingestion/main.py
- [X] T020 Create basic chatbot component structure in docusaurus/src/components/Chatbot/Chatbot.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Chatbot Interaction (Priority: P1) 🎯 MVP

**Goal**: Enable GIAIC hackathon participants and students to ask questions about Physical AI and Humanoid Robotics and get immediate, accurate answers based on textbook content with citations.

**Independent Test**: Can be fully tested by asking questions about the textbook content and verifying responses are accurate and cite the correct sections.

### Tests for User Story 1 (OPTIONAL - included) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T021 [P] [US1] Contract test for POST /query endpoint in backend/tests/contract/test_query.py
- [X] T022 [P] [US1] Integration test for basic query functionality in backend/tests/integration/test_query_flow.py
- [X] T023 [P] [US1] Unit test for Cohere response generation in backend/tests/unit/test_cohere_service.py

### Implementation for User Story 1

- [X] T024 [P] [US1] Create Query model in backend/src/models/query.py
- [X] T025 [P] [US1] Create RetrievedContext model in backend/src/models/retrieved_context.py
- [X] T026 [P] [US1] Create GeneratedResponse model in backend/src/models/response.py
- [X] T027 [P] [US1] Create ChatSession model in backend/src/models/chat_session.py
- [X] T028 [US1] Implement RAG service in backend/src/services/rag_service.py
- [X] T029 [US1] Implement citation service in backend/src/services/citation_service.py
- [X] T030 [US1] Implement POST /query endpoint in backend/src/api/routes/query.py
- [X] T031 [US1] Add response validation and error handling in backend/src/api/routes/query.py
- [X] T032 [US1] Create chat window UI component in docusaurus/src/components/Chatbot/ChatWindow.tsx
- [X] T033 [US1] Create query input component in docusaurus/src/components/Chatbot/QueryInput.tsx
- [X] T034 [US1] Implement chatbot communication with backend API in docusaurus/src/components/Chatbot/Chatbot.tsx
- [X] T035 [US1] Add citation display in chat responses in docusaurus/src/components/Chatbot/ChatWindow.tsx
- [X] T036 [US1] Implement mobile responsive design for chat interface in docusaurus/src/components/Chatbot/

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Selected Text Querying (Priority: P2)

**Goal**: Enable students and judges to select specific text in the textbook and ask questions about just that content.

**Independent Test**: Can be tested by selecting specific text, asking questions about it, and verifying that responses are derived only from the selected text rather than the broader textbook.

### Tests for User Story 2 (OPTIONAL - included) ⚠️

- [X] T037 [P] [US2] Contract test for POST /query-selected endpoint in backend/tests/contract/test_query_selected.py
- [X] T038 [P] [US2] Integration test for selected text query functionality in backend/tests/integration/test_selected_query_flow.py

### Implementation for User Story 2

- [X] T039 [US2] Implement POST /query-selected endpoint in backend/src/api/routes/query_selected.py
- [X] T040 [US2] Enhance RAG service to handle selected text context in backend/src/services/rag_service.py
- [X] T041 [US2] Create text selection utility in docusaurus/src/utils/textSelection.ts
- [X] T042 [US2] Implement floating action button for "Ask about selected text" in docusaurus/src/components/Chatbot/Chatbot.tsx
- [ ] T043 [US2] Add selected text highlighting and context handling in docusaurus/src/components/Chatbot/
- [X] T044 [US2] Modify backend API client to handle selected text queries in docusaurus/src/components/Chatbot/Chatbot.tsx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Mobile-Responsive Interaction (Priority: P3)

**Goal**: Enable users to access the chatbot functionality on mobile devices while studying.

**Independent Test**: Can be tested by accessing and using the chatbot on various screen sizes and verifying the UI remains functional and readable.

### Tests for User Story 3 (OPTIONAL - included) ⚠️

- [ ] T045 [P] [US3] Responsive UI test for mobile devices in docusaurus/tests/integration/test_responsive_ui.js

### Implementation for User Story 3

- [ ] T046 [US3] Enhance mobile responsiveness of chat window in docusaurus/src/components/Chatbot/ChatWindow.tsx
- [ ] T047 [US3] Optimize touch interactions for chat components in docusaurus/src/components/Chatbot/
- [ ] T048 [US3] Implement mobile-friendly input methods in docusaurus/src/components/Chatbot/QueryInput.tsx
- [ ] T049 [US3] Add mobile-specific UI optimizations for text selection in docusaurus/src/components/Chatbot/

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Ingestion Pipeline Implementation

**Goal**: Implement the content ingestion pipeline to process textbook content into Qdrant vector store with metadata in payloads

- [ ] T050 Implement document loading utilities in backend/ingestion/loaders/
- [ ] T051 Implement text processing utilities in backend/ingestion/processors/
- [ ] T052 Complete ingestion script main logic in backend/ingestion/main.py
- [ ] T053 Add error handling and logging to ingestion pipeline in backend/ingestion/
- [ ] T054 Create ingestion test suite in backend/tests/integration/test_ingestion.py

---

## Phase 7: API Enhancement and Health Checks

**Goal**: Implement remaining API endpoints and ensure system reliability

- [ ] T055 Implement GET /health endpoint in backend/src/api/routes/health.py
- [ ] T056 Implement GET /stats endpoint in backend/src/api/routes/health.py
- [ ] T057 Add comprehensive error handling and logging across all endpoints
- [ ] T058 Implement response streaming for improved UX in backend/src/api/routes/query.py

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T059 [P] Documentation updates in docs/
- [ ] T060 Add comprehensive logging throughout backend services
- [ ] T061 Add performance monitoring and metrics collection
- [ ] T062 [P] Additional unit tests in backend/tests/unit/
- [ ] T063 Security hardening and input validation
- [ ] T064 Implement caching for embeddings and responses
- [ ] T065 Run quickstart.md validation and update documentation
- [ ] T066 Deploy to Vercel or GitHub Pages for frontend and backend

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Ingestion Pipeline (Phase 6)**: Can run in parallel with user stories after foundational
- **API Enhancement (Phase 7)**: Can run in parallel with user stories after foundational
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for POST /query endpoint in backend/tests/contract/test_query.py"
Task: "Integration test for basic query functionality in backend/tests/integration/test_query_flow.py"
Task: "Unit test for Cohere response generation in backend/tests/unit/test_cohere_service.py"

# Launch all models for User Story 1 together:
Task: "Create Query model in backend/src/models/query.py"
Task: "Create RetrievedContext model in backend/src/models/retrieved_context.py"
Task: "Create GeneratedResponse model in backend/src/models/response.py"
Task: "Create ChatSession model in backend/src/models/chat_session.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: Ingestion Pipeline (Phase 6)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence