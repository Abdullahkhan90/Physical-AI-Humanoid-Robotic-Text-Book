---
id: 1
title: Update Constitution
stage: constitution
date: 2025-12-12
surface: agent
model: Qwen
feature: none
branch: main
user: Lenovo
command: /sp.constitution
labels: ["constitution","textbook","ai","robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - No tests added
---

## Prompt

/sp.constitution # Project: AI Native Textbook on Physical AI & Humanoid Robotics

## Core principles:
- **Accuracy** through primary source verification, with a strong focus on real-world applications.
- **Clarity** for a technical audience with backgrounds in AI, robotics, and engineering.
- **Reproducibility** of all processes and experiments, with full traceability of claims.
- **Rigour** (peer-reviewed sources and credible publications for scientific integrity).

## Key standards:
- All factual claims must be traceable to reliable and verifiable sources.
- Citation format: **APA style** for all references.
- **Source types**: At least 50% of sources must be peer-reviewed journal articles, conference papers, or industry standards.
- **Plagiarism check**: Zero tolerance for plagiarism before submission.
- **Writing clarity**: Writing should aim for a **Flesch-Kincaid grade level of 12-14** for technical clarity and readability.

## Constraints:
- **Word count**: 15,000-20,000 words (to cover all course modules comprehensively).
- **Minimum 25 sources**, with diversity in source types (journals, books, online standards).
- **Format**: Published via Docusaurus and deployed to **GitHub Pages**; chapters available in Markdown format with embedded citations.
- **Deadline**: First draft to be submitted within **3 months**.

## Success criteria:
- All technical claims are verified and referenced from credible sources.
- No plagiarism detected, verified via plagiarism detection software.
- Textbook passes an internal review by domain experts in AI, robotics, and education.
- Successful integration of **RAG chatbot** to answer student queries based on book content.
- Published book is fully functional and accessible on **GitHub Pages**.
- Demonstrated ability for students to **personalize content** and **translate chapters** into Urdu for localized learning experiences.

## Response snapshot

The constitution for the "AI Native Textbook on Physical AI & Humanoid Robotics" project has been successfully created with version 1.0.0. The document includes six core principles (Accuracy, Clarity, Reproducibility, Rigor, Verification, and Plagiarism-Free Content), key standards, constraints, success criteria, and governance procedures tailored to the textbook project. The constitution file is now complete and stored at `.specify/memory/constitution.md`.

## Outcome

- ✅ Impact: Created initial project constitution document with all specified principles and standards
- 🧪 Tests: No tests added
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Could proceed with /sp.specify to create feature specifications
- 🧠 Reflection: The constitution successfully captures the academic rigor and standards required for the textbook project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): Standard constitution creation prompt
- Next experiment (smallest change to try): Create feature specifications based on this constitution