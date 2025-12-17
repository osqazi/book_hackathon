---
id: 0004
title: Chapter Personalization Implementation Planning
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-chapter-personalization
branch: 003-chapter-personalization
user: Claude Code
command: /sp.plan
labels: ["planning", "architecture", "database", "api-design", "frontend"]
links:
  spec: specs/003-chapter-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-chapter-personalization/plan.md
  - specs/003-chapter-personalization/research.md
  - specs/003-chapter-personalization/data-model.md
  - specs/003-chapter-personalization/quickstart.md
  - specs/003-chapter-personalization/contracts/openapi.yaml
  - CLAUDE.md
tests:
  - Constitution check passed (all 6 principles)
  - Performance targets validated (<200ms API latency achievable with indexes)
---

## Prompt

/sp.plan with user-provided technical specifications:

**Technical Specifications (Emphasizing Existing Backend Reuse)**:
- **Authentication & Session**: Use existing Better-Auth client/session (e.g., `authClient` or session hooks) to get current user ID. All API calls must be authenticated via existing Better-Auth mechanisms (e.g., cookies/sessions).
- **Database**: Add a new table (e.g., `personalized_chapters`) in the existing Neon Postgres DB: columns `id` (PK), `user_id` (FK to Better-Auth users table), `chapter_slug` (or path/ID), `created_at`. Use Better-Auth's schema extension or Prisma/Drizzle if already in use for migrations.
- **Backend/API (Extend Existing Better-Auth Server)**: Add custom protected routes/endpoints in the existing auth server (e.g., under `/api/personalization` or via Better-Auth plugins if supported).
  - Endpoints: GET `/personalized`, POST `/personalized`, DELETE `/personalized/:chapterSlug`
  - Secure with existing Better-Auth middleware (e.g., require session/user).
- **Frontend (Docusaurus)**: Swizzle/customize DocItem or DocPage components for button/indicator (client-side, conditional on auth status). Use React hooks (e.g., SWR, TanStack Query) for fetching/toggling personalization data from existing backend APIs.
- **UI/UX**: Modern, accessible (e.g., use shadcn/ui if available); loading/toast feedback on actions.

## Response snapshot

Generated comprehensive implementation plan with complete architecture design:

**Phase 0: Research** ✅ COMPLETED
- Decided on technology stack: TanStack Query, Drizzle ORM, Docusaurus swizzling
- Documented all technical decisions with rationale and alternatives considered
- Research covered: backend extension strategy, database schema, state management, component customization, performance optimization, authentication integration
- Created research.md with 8 technology decisions and architecture patterns

**Phase 1: Design & Contracts** ✅ COMPLETED
- **data-model.md**: Complete database schema with Drizzle ORM definitions
  - Table: personalized_chapters with UNIQUE(user_id, chapter_path)
  - Indexes for performance: user_id, chapter_path, composite unique
  - Query examples for CRUD operations (idempotent add/remove)
  - Validation rules with Zod schemas

- **contracts/openapi.yaml**: Full OpenAPI 3.0 specification
  - 3 endpoints: GET, POST, DELETE with complete request/response schemas
  - Authentication via Better-Auth session cookies
  - Error responses for all scenarios (401, 422, 429, 500)
  - Idempotency documented for POST and DELETE

- **quickstart.md**: Developer setup guide
  - Prerequisites, environment setup, database migration steps
  - File structure overview with NEW/MODIFIED markers
  - Step-by-step implementation guide (5 phases)
  - Troubleshooting common issues
  - Manual testing checklist

- **plan.md**: Complete implementation plan (this generated document)
  - Architecture overview with component diagrams and data flows
  - Constitution check: ALL 6 GATES PASSED
  - Performance targets: <200ms p95 API latency, <50KB bundle
  - Security considerations: Authentication, input validation, data privacy
  - Risk assessment with mitigation strategies
  - Deployment plan with rollback procedures

**Agent Context Updated**: CLAUDE.md updated with new technologies (TypeScript 5.x, Node.js 18+, Neon Postgres table)

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation; all design decisions documented with rationale; constitution compliance verified; performance and security requirements specified
- 🧪 Tests: Constitution check passed all 6 gates; performance targets validated as achievable with indexed queries
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, contracts/openapi.yaml; updated CLAUDE.md
- 🔁 Next prompts: /sp.tasks (generate tasks.md with detailed implementation checklist)
- 🧠 Reflection: Planning phase complete. Architecture leverages existing Better-Auth backend (no new services), uses proven patterns (TanStack Query for state, Drizzle for DB), and meets all constitutional requirements. Key decisions: Docusaurus swizzling for UI injection, idempotent operations via database constraints, 50-character excerpt from spec clarification.

## Evaluation notes (flywheel)

- Failure modes observed: None - all phases completed successfully
- Graders run and results (PASS/FAIL): PASS - Constitution check passed all 6 principles, performance targets validated, security requirements specified
- Prompt variant (if applicable): Standard /sp.plan workflow with user-provided technical specifications
- Next experiment (smallest change to try): Generate tasks.md with /sp.tasks command to create detailed implementation checklist
