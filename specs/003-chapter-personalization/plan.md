# Implementation Plan: Per-User Chapter Personalization

**Branch**: `003-chapter-personalization` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-chapter-personalization/spec.md`

## Summary

Implement a user-specific chapter personalization (bookmarking) system for the Humanoid Robotics Book documentation site. Authenticated users can mark chapters as personalized with a single click, view all personalized chapters on a dedicated `/personalization` page, and see visual indicators showing which chapters are personalized. The system extends the existing Better-Auth backend with custom API endpoints and integrates seamlessly with Docusaurus using React components and TanStack Query for state management.

**Key Technical Approach**:
- Extend existing Better-Auth server with REST API endpoints (`/api/personalization/*`)
- Add `personalized_chapters` table to existing Neon Postgres database using Drizzle ORM
- Use TanStack Query for frontend data fetching, caching, and optimistic updates
- Swizzle Docusaurus DocItem component to inject personalization button
- Create dedicated `/personalization` page with responsive card grid layout
- Implement idempotent API operations with database-level unique constraints
- Ensure <200ms p95 API latency via indexed database queries

## Technical Context

**Language/Version**: TypeScript 5.x, Node.js 18+
**Primary Dependencies**:
- **Backend**: Better-Auth (existing), Drizzle ORM 0.20+, Zod 3.22+ (validation), Express (existing)
- **Frontend**: React 18+, TanStack Query 5.0+, Docusaurus 3.x (existing)
**Storage**: Neon Postgres (existing from 002-better-auth-signup), new table `personalized_chapters`
**Testing**: Jest (unit), React Testing Library (components), Playwright/Cypress (E2E)
**Target Platform**: Web (Docusaurus static site + Node.js backend for API)
**Project Type**: Web application (separate frontend static build + backend API server)
**Performance Goals**: <200ms p95 API response time, <50KB additional frontend bundle (gzipped)
**Constraints**:
- Must reuse existing Better-Auth backend (no new servers)
- Must preserve all CORS/session/security configurations
- Must not impact Docusaurus static build process (client-side only)
- Must work without changes to existing authentication flow
**Scale/Scope**:
- Expected 10,000+ users, avg 20 personalized chapters per user
- 3 API endpoints, 4 new React components, 1 new database table
- Integration with existing Docusaurus theme (swizzle 1 component)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Authentication Implementation Principles

✅ **I. Strict Better-Auth Adherence**: PASS
- All authentication uses existing Better-Auth session validation
- No custom authentication logic implemented
- API endpoints use Better-Auth middleware for session checks
- User ID extracted from session token (not from request body)

✅ **II. Seamless Docusaurus Integration**: PASS
- Uses Docusaurus swizzling (official customization mechanism)
- No modifications to core Docusaurus functionality
- Client-side rendering only (no impact on static generation)
- Compatible with Docusaurus plugin ecosystem

✅ **III. User Privacy & Data Minimization**: PASS
- Only stores chapter URLs (public information) and user associations
- No PII collected beyond existing Better-Auth user table
- No tracking or analytics on personalization behavior
- Users can delete all personalizations (cascade on account deletion)

✅ **IV. Full TypeScript Type Safety**: PASS
- All database queries use Drizzle ORM with TypeScript types
- API contracts defined in OpenAPI spec with TypeScript interfaces
- React components fully typed (props, hooks, context)
- Zod validation provides runtime type safety

✅ **V. Secure Storage & Retrieval**: PASS
- Database access via Drizzle ORM (prevents SQL injection)
- All API endpoints require authentication
- User can only access/modify their own personalizations
- HTTPS enforced for all data transmission (existing infrastructure)

✅ **VI. Maintainable & Extensible Code**: PASS
- Modular architecture (separation of concerns)
- Reusable components (PersonalizationButton, PersonalizationCard)
- Extensible for future features (tags, categories, notes)
- Clear documentation in research.md, data-model.md, quickstart.md

### Constitution Compliance Summary

**Status**: ✅ ALL GATES PASSED

No violations detected. The implementation follows all constitutional principles:
- Strict adherence to Better-Auth patterns
- Seamless Docusaurus integration without disruption
- Privacy-first data collection (minimal, transparent)
- Full TypeScript type safety across stack
- Secure storage with Better-Auth session validation
- Clean, maintainable code architecture

**Re-evaluated After Phase 1 Design**: ✅ PASS (no changes needed)

## Project Structure

### Documentation (this feature)

```text
specs/003-chapter-personalization/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (technology decisions, rationale)
├── data-model.md        # Phase 1 output (database schema, queries)
├── quickstart.md        # Phase 1 output (developer setup guide)
├── contracts/           # Phase 1 output (API contracts)
│   └── openapi.yaml     # OpenAPI 3.0 specification
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

**Selected Structure**: Web Application (Option 2)

```text
humanoid-robotics-book/
├── src/
│   ├── lib/
│   │   ├── db/
│   │   │   ├── schema.ts                  # Drizzle schema (add PersonalizedChapter table)
│   │   │   ├── queries.ts                 # NEW: Database queries for personalization CRUD
│   │   │   └── client.ts                  # Drizzle client (existing)
│   │   └── auth/
│   │       └── server.ts                  # Better-Auth server (existing, extend with new routes)
│   ├── api/
│   │   └── personalization/
│   │       ├── routes.ts                  # NEW: Express router for personalization endpoints
│   │       ├── handlers.ts                # NEW: Request handlers (GET, POST, DELETE)
│   │       └── validation.ts              # NEW: Zod schemas for request validation
│   ├── components/
│   │   ├── PersonalizationButton.tsx      # NEW: Toggle button for doc pages
│   │   ├── PersonalizationCard.tsx        # NEW: Chapter card component for /personalization page
│   │   ├── PersonalizationIcon.tsx        # NEW: Star/bookmark icon (SVG)
│   │   └── PersonalizationEmptyState.tsx  # NEW: Empty state when no personalizations
│   ├── hooks/
│   │   ├── usePersonalization.ts          # NEW: TanStack Query hooks (queries + mutations)
│   │   └── useSession.ts                  # Existing Better-Auth session hook
│   ├── pages/
│   │   └── personalization.tsx            # NEW: Dedicated /personalization page
│   ├── theme/                             # Docusaurus theme customizations
│   │   └── DocItem/
│   │       └── Layout/
│   │           └── index.tsx              # MODIFIED: Swizzled component (inject button)
│   └── styles/
│       └── personalization.css            # NEW: Custom styles for personalization UI
├── drizzle/
│   └── migrations/
│       └── 0001_add_personalized_chapters.sql  # NEW: Database migration
├── tests/
│   ├── unit/
│   │   ├── queries.test.ts                # NEW: Database query tests
│   │   ├── handlers.test.ts               # NEW: API handler tests
│   │   └── hooks.test.ts                  # NEW: React hook tests
│   ├── integration/
│   │   └── personalization-flow.test.ts   # NEW: End-to-end personalization flow
│   └── e2e/
│       └── personalization.spec.ts        # NEW: Playwright/Cypress E2E tests
├── specs/003-chapter-personalization/     # This directory
├── docusaurus.config.js                   # MODIFIED: Register /personalization page
├── package.json                           # MODIFIED: Add TanStack Query dependency
└── .env.local                             # Existing (no changes needed)
```

**Structure Decision**:
Web application structure with frontend (Docusaurus static site) and backend (Better-Auth Node.js server). Frontend communicates with backend via REST API. This structure:
- Reuses existing Better-Auth backend (no new services)
- Keeps client-side code in `src/components`, `src/pages`, `src/hooks`
- Keeps server-side code in `src/api`, `src/lib/db`
- Follows existing project conventions from 002-better-auth-signup feature

## Complexity Tracking

*No constitutional violations detected. This section is not applicable.*

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A       | N/A        | N/A                                 |

## Architecture Overview

### System Components

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          User Browser                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │              Docusaurus Static Site (Client-Side)                  │ │
│  ├────────────────────────────────────────────────────────────────────┤ │
│  │                                                                      │ │
│  │  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐ │ │
│  │  │  Doc Page        │  │  /personalization │  │  React Query     │ │ │
│  │  │  (Swizzled)      │  │  Page            │  │  Client          │ │ │
│  │  ├──────────────────┤  ├──────────────────┤  ├──────────────────┤ │ │
│  │  │ Personalization  │  │ PersonalizationCa│  │ Cache            │ │ │
│  │  │ Button           │  │ rd Grid          │  │ Mutations        │ │ │
│  │  │ (Star Icon)      │  │ (Title, Excerpt) │  │ Queries          │ │ │
│  │  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘ │ │
│  │           │                     │                     │           │ │
│  │           └─────────────────────┴──────────── ────────┘           │ │
│  │                                 │                                  │ │
│  └─────────────────────────────────┼──────────────────────────────────┘ │
│                                    │                                    │
└────────────────────────────────────┼────────────────────────────────────┘
                                     │
                                     │ HTTP REST API
                                     │ (Authenticated via Better-Auth session)
                                     │
┌────────────────────────────────────▼────────────────────────────────────┐
│                   Better-Auth Backend Server (Node.js)                   │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌─────────────────────────────────────────────────────────────────────┐ │
│  │                    Express Application                              │ │
│  ├─────────────────────────────────────────────────────────────────────┤ │
│  │                                                                      │ │
│  │  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐ │ │
│  │  │  Better-Auth     │  │  Personalization │  │  Database        │ │ │
│  │  │  Middleware      │  │  Router          │  │  Queries         │ │ │
│  │  │  (Session        │  │  /api/           │  │  (Drizzle ORM)   │ │ │
│  │  │  Validation)     │  │  personalization │  │                  │ │ │
│  │  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘ │ │
│  │           │                     │                     │           │ │
│  │           └─────────────────────┴─────────────────────┘           │ │
│  │                                 │                                  │ │
│  └─────────────────────────────────┼──────────────────────────────────┘ │
│                                    │                                    │
└────────────────────────────────────┼────────────────────────────────────┘
                                     │
                                     │ SQL Queries
                                     │
┌────────────────────────────────────▼────────────────────────────────────┐
│                       Neon Postgres Database                             │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌──────────────────┐         ┌────────────────────────────────────┐   │
│  │  user            │         │  personalized_chapters             │   │
│  │  (Better-Auth)   │◄────────│  (New Table)                       │   │
│  ├──────────────────┤  1:N    ├────────────────────────────────────┤   │
│  │  id (PK)         │         │  id (PK)                           │   │
│  │  email           │         │  user_id (FK → user.id)            │   │
│  │  name            │         │  chapter_path (URL path)           │   │
│  │  ...             │         │  chapter_title                     │   │
│  └──────────────────┘         │  chapter_excerpt (50 chars)        │   │
│                                │  created_at                        │   │
│                                │  UNIQUE(user_id, chapter_path)     │   │
│                                └────────────────────────────────────┘   │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
```

### Data Flow

**User Personalizes a Chapter** (POST flow):

1. User clicks personalization button on doc page
2. React component calls `useAddPersonalization()` mutation hook
3. TanStack Query sends POST `/api/personalization/chapters` with `{ chapter_path, chapter_title, chapter_excerpt }`
4. Better-Auth middleware validates session, extracts `user_id`
5. Request handler validates input with Zod schema
6. Database query executes `INSERT ... ON CONFLICT DO NOTHING` (idempotent)
7. Response returns created/existing `PersonalizedChapter` object
8. TanStack Query updates cache and invalidates related queries
9. UI updates optimistically (button shows "personalized" state immediately)

**User Views Personalized Chapters** (GET flow):

1. User navigates to `/personalization` page
2. React component calls `usePersonalizedChapters()` query hook
3. TanStack Query checks cache (if fresh, return cached data)
4. If stale, sends GET `/api/personalization/chapters`
5. Better-Auth middleware validates session, extracts `user_id`
6. Database query fetches all chapters for user: `SELECT * FROM personalized_chapters WHERE user_id = ?`
7. Response returns array of `PersonalizedChapter` objects
8. TanStack Query caches response for 5 minutes
9. UI renders PersonalizationCard components in grid layout

**User Removes Personalized Chapter** (DELETE flow):

1. User clicks remove button on PersonalizationCard
2. React component calls `useRemovePersonalization()` mutation hook
3. TanStack Query sends DELETE `/api/personalization/chapters/:chapterPath` (URL-encoded)
4. Better-Auth middleware validates session, extracts `user_id`
5. Database query executes `DELETE WHERE user_id = ? AND chapter_path = ?`
6. Response returns 204 No Content (idempotent, success even if not found)
7. TanStack Query updates cache (removes item) and invalidates queries
8. UI updates optimistically (card removed from grid immediately)

### Error Handling Flow

**Session Expired**:
1. API returns 401 Unauthorized with `{ error: "SESSION_EXPIRED", message: "..." }`
2. TanStack Query mutation onError callback triggered
3. Error message displayed via toast notification
4. User redirected to login page (Better-Auth redirect)

**Network Error**:
1. API request fails (timeout, no connection)
2. TanStack Query automatically retries 3 times with exponential backoff
3. After 3 retries, mutation onError callback triggered
4. Error message displayed: "Network error. Please try again."

**Validation Error**:
1. API returns 422 Unprocessable Entity with `{ error: "VALIDATION_ERROR", validation_errors: [...] }`
2. TanStack Query mutation onError callback triggered
3. Specific validation errors displayed in toast notification

## Implementation Phases

### Phase 0: Research ✅ COMPLETED

**Output**: `research.md`

**Completed Decisions**:
- ✅ Backend extension strategy: Extend existing Better-Auth server with custom routes
- ✅ Database schema: Drizzle ORM with `personalized_chapters` table
- ✅ Frontend state management: TanStack Query for data fetching and mutations
- ✅ Docusaurus customization: Swizzle DocItem/Layout/Wrapper component
- ✅ UI component library: Custom components using Docusaurus/Infima CSS
- ✅ Performance optimization: URL path-based idempotent operations with DB constraints
- ✅ Preview/excerpt generation: Extract on frontend from Docusaurus page metadata
- ✅ Authentication integration: Use existing Better-Auth `useSession()` hook

### Phase 1: Design & Contracts ✅ COMPLETED

**Outputs**: `data-model.md`, `contracts/openapi.yaml`, `quickstart.md`

**Completed Artifacts**:
- ✅ **data-model.md**: Complete database schema, Drizzle ORM definitions, query examples
- ✅ **contracts/openapi.yaml**: Full OpenAPI 3.0 spec with all endpoints, schemas, examples
- ✅ **quickstart.md**: Developer setup guide with prerequisites, step-by-step instructions

**Database Schema**:
- Table: `personalized_chapters`
- Fields: `id`, `user_id`, `chapter_path`, `chapter_title`, `chapter_excerpt`, `created_at`
- Constraints: UNIQUE(user_id, chapter_path), FK to user(id) CASCADE DELETE
- Indexes: user_id, chapter_path, composite unique index

**API Endpoints**:
- GET `/api/personalization/chapters` → Fetch all personalized chapters for user
- POST `/api/personalization/chapters` → Add chapter (idempotent)
- DELETE `/api/personalization/chapters/:chapterPath` → Remove chapter (idempotent)

### Phase 2: Implementation Planning 🚀 NEXT STEP

**Output**: `tasks.md` (generated by `/sp.tasks` command)

**Tasks to Generate**:
1. **Database Layer**:
   - Add PersonalizedChapter table to Drizzle schema
   - Create database migration
   - Implement CRUD query functions
   - Write unit tests for database operations

2. **Backend API**:
   - Create Express router for `/api/personalization`
   - Implement request handlers (GET, POST, DELETE)
   - Add Zod validation schemas
   - Add Better-Auth middleware for authentication
   - Integrate with existing Better-Auth server
   - Write unit tests for API handlers

3. **Frontend Hooks**:
   - Setup TanStack Query client
   - Implement `usePersonalizedChapters()` query hook
   - Implement `useAddPersonalization()` mutation hook
   - Implement `useRemovePersonalization()` mutation hook
   - Implement `useIsChapterPersonalized()` hook
   - Configure cache invalidation and optimistic updates
   - Write unit tests for hooks

4. **UI Components**:
   - Create PersonalizationButton component
   - Create PersonalizationIcon component (SVG)
   - Create PersonalizationCard component
   - Create PersonalizationEmptyState component
   - Add CSS styling (responsive, dark mode)
   - Write component tests with React Testing Library

5. **Docusaurus Integration**:
   - Swizzle DocItem/Layout component
   - Inject PersonalizationButton in doc header
   - Create /personalization page
   - Register page in Docusaurus config
   - Ensure unauthenticated users don't see UI

6. **Testing & QA**:
   - Write integration tests (end-to-end flows)
   - Write E2E tests with Playwright/Cypress
   - Manual testing checklist
   - Performance testing (API latency)

7. **Deployment**:
   - Run database migration on staging
   - Deploy backend with new API routes
   - Deploy frontend with new components
   - Verify in production environment

## Risk Assessment

### Technical Risks

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Docusaurus swizzling breaks on upgrade | Medium | High | Use "wrap" mode where possible, maintain comprehensive tests, document customizations |
| Database performance degrades at scale | Low | Medium | Indexes on all query columns, limit personalizations per user (max 100), add pagination if needed |
| TanStack Query cache inconsistencies | Low | Medium | Strict cache invalidation rules, optimistic update rollback on error, comprehensive testing |
| CORS issues with API calls | Low | High | Reuse existing Better-Auth CORS config, test thoroughly in staging |
| Session expiration during actions | Medium | Low | Clear error messages, automatic redirect to login, no data loss (retry after re-auth) |

### Implementation Risks

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Tight coupling with Docusaurus internals | Medium | Medium | Abstract Docusaurus-specific logic, use official APIs where possible |
| Database migration fails in production | Low | High | Test migration thoroughly in staging, keep rollback script, deploy during low-traffic window |
| API latency exceeds 200ms requirement | Low | Medium | Load test in staging, optimize database queries, add caching if needed |
| Mobile responsiveness issues | Low | Low | Test on multiple devices, use Docusaurus responsive breakpoints |

## Security Considerations

### Authentication & Authorization

- All API endpoints require valid Better-Auth session (validated via middleware)
- User ID extracted from session token (never from request body)
- Users can only access/modify their own personalizations
- CSRF protection via Better-Auth's built-in middleware
- Rate limiting on API endpoints (100 requests/min per user)

### Input Validation

- All user input validated with Zod schemas on backend
- `chapter_path`: Must match URL path pattern `/[a-zA-Z0-9/_-]+`
- `chapter_title`: Max 500 characters, sanitized for XSS
- `chapter_excerpt`: Max 50 characters, sanitized for XSS
- SQL injection prevention via Drizzle ORM parameterized queries

### Data Privacy

- Personalization data private to each user (no sharing)
- No tracking or analytics on personalization behavior
- Users can delete all personalizations (cascade on account deletion)
- No PII collected beyond existing Better-Auth user table

## Performance Targets

### API Response Times

**Target**: <200ms p95 latency (per spec clarification FR-001)

**Expected Performance**:
- GET `/api/personalization/chapters`: ~50ms (indexed query on user_id)
- POST `/api/personalization/chapters`: ~80ms (upsert with conflict check)
- DELETE `/api/personalization/chapters/:path`: ~60ms (delete with index lookup)

**Optimization Strategies**:
1. Database indexes on `user_id`, `chapter_path`, and composite `(user_id, chapter_path)`
2. Connection pooling for database (reuse existing pool from Better-Auth)
3. Limit query results to 100 chapters per user
4. TanStack Query caching (reduce redundant API calls)
5. Deploy backend close to database (same region for low latency)

### Frontend Bundle Size

**Target**: <50KB additional JavaScript (gzipped)

**Estimated Bundle Impact**:
- TanStack Query: ~15KB (gzipped)
- Custom components: ~10KB (gzipped)
- Icons: ~5KB (gzipped)
- **Total**: ~30KB (well under 50KB target)

### Database Storage

**Per User**: ~200 bytes per personalized chapter

**Scaling Estimate**:
- 10,000 users × 20 chapters avg = 200,000 rows = 40MB
- 100,000 users × 20 chapters avg = 2,000,000 rows = 400MB
- 1,000,000 users × 20 chapters avg = 20,000,000 rows = 4GB

**Conclusion**: Storage is negligible even at 1M users. No partitioning or archiving needed.

## Testing Strategy

### Unit Tests (Jest)

**Coverage Target**: >80%

**Test Files**:
- `src/lib/db/queries.test.ts` - Database CRUD operations
- `src/api/personalization/handlers.test.ts` - API request handlers
- `src/hooks/usePersonalization.test.ts` - React Query hooks
- `src/components/PersonalizationButton.test.tsx` - Button component
- `src/components/PersonalizationCard.test.tsx` - Card component

### Integration Tests

**Scenarios**:
1. End-to-end personalization flow (add → view → remove)
2. Session expiration handling
3. Concurrent requests (race condition prevention)
4. Database constraint enforcement (duplicate prevention)
5. Cache invalidation on mutations

### E2E Tests (Playwright/Cypress)

**Test Cases**:
- User logs in, personalizes chapter, sees indicator
- User navigates to /personalization, sees chapter card
- User removes chapter from card, verifies removal
- User toggles personalization button (add/remove)
- Unauthenticated user doesn't see personalization UI
- Session expiration shows error message

### Manual Testing Checklist

See `quickstart.md` for complete manual testing checklist.

## Deployment Plan

### Prerequisites

- Database migration ready and tested in staging
- Backend code merged and built
- Frontend code merged and built
- Environment variables configured in production

### Deployment Steps

1. **Database Migration** (30 minutes before deploy):
   ```bash
   # Connect to production database
   npx drizzle-kit push:pg --schema=./src/lib/db/schema.ts

   # Verify table created
   npx drizzle-kit introspect:pg
   ```

2. **Backend Deployment** (continuous):
   - Deploy updated Better-Auth server with new API routes
   - Verify `/api/personalization/chapters` endpoint responds
   - Monitor logs for errors

3. **Frontend Deployment** (continuous):
   - Build Docusaurus static site: `npm run build`
   - Deploy to existing hosting (Vercel/Netlify/GitHub Pages)
   - Verify /personalization page accessible

4. **Smoke Testing** (15 minutes after deploy):
   - Log in with test account
   - Personalize a chapter
   - View /personalization page
   - Remove personalization
   - Verify no console errors

### Rollback Plan

**If Critical Issues Detected**:

1. **Revert Frontend**:
   - Redeploy previous build from Git history
   - Hide personalization UI via feature flag (if available)

2. **Revert Backend**:
   - Redeploy previous backend version
   - Remove `/api/personalization` routes (comment out router registration)

3. **Rollback Database** (LAST RESORT):
   ```sql
   -- Run rollback migration
   DROP INDEX IF EXISTS idx_personalized_chapters_path;
   DROP INDEX IF EXISTS idx_personalized_chapters_user_id;
   DROP INDEX IF EXISTS idx_personalized_chapters_user_chapter;
   DROP TABLE IF EXISTS personalized_chapters;
   ```
   **⚠️ WARNING**: This deletes all personalization data. Only use if table is causing database issues.

### Post-Deployment Monitoring

**Metrics to Watch** (first 24 hours):
- API endpoint latency (p50, p95, p99)
- API error rates (4xx, 5xx)
- Database query performance (slow query log)
- Frontend JavaScript errors (Sentry/similar)
- User adoption (personalizations created per day)

## Next Steps

1. ✅ **Planning Complete**: All design artifacts generated (research.md, data-model.md, contracts/, quickstart.md)
2. 🚀 **Run `/sp.tasks`**: Generate tasks.md with detailed implementation checklist
3. 💻 **Begin Implementation**: Follow tasks.md for step-by-step development
4. ✅ **Testing**: Write and run unit, integration, and E2E tests
5. 🚀 **Deploy**: Follow deployment plan above
6. 📊 **Monitor**: Track metrics and user adoption

## References

- **Feature Specification**: [spec.md](./spec.md)
- **Research & Decisions**: [research.md](./research.md)
- **Database Schema**: [data-model.md](./data-model.md)
- **API Contracts**: [contracts/openapi.yaml](./contracts/openapi.yaml)
- **Developer Setup**: [quickstart.md](./quickstart.md)
- **TanStack Query Docs**: https://tanstack.com/query/latest
- **Drizzle ORM Docs**: https://orm.drizzle.team/docs/overview
- **Docusaurus Swizzling**: https://docusaurus.io/docs/swizzling
- **Better-Auth Documentation**: https://www.better-auth.com/docs

---

**Plan Version**: 1.0.0
**Last Updated**: 2025-12-17
**Status**: Phase 1 Complete, Ready for `/sp.tasks`
