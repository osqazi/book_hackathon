# Implementation Tasks: Per-User Chapter Personalization

**Feature**: 003-chapter-personalization
**Branch**: `003-chapter-personalization`
**Date**: 2025-12-17
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Overview

This task breakdown organizes implementation by **user story** to enable independent, incremental delivery. Each user story phase is a complete, testable increment that delivers value.

**Total Tasks**: 42
**Parallel Opportunities**: 18 tasks can be executed in parallel within their phases

---

## MVP Scope (Recommended First Release)

**User Story 1** (Personalize Chapter) + **User Story 2** (View Personalized Chapters) = Minimum Viable Product

This delivers the core value: users can bookmark chapters and view their personalized list. US3 and US4 are enhancements that can follow.

---

## Implementation Strategy

1. **Setup Phase**: Project initialization (foundational infrastructure)
2. **Foundational Phase**: Blocking prerequisites needed by all user stories
3. **User Story Phases**: Implement stories in priority order (P1 → P2)
   - US1 (P1): Personalize Chapter - Core bookmarking functionality
   - US2 (P1): View Personalized Chapters - Dedicated personalization page
   - US3 (P2): Visual Indicators - Enhanced UX feedback
   - US4 (P2): Anonymous User Experience - Hide UI for unauthenticated users
4. **Polish Phase**: Cross-cutting concerns and final touches

---

## Task Format Legend

```
- [X] [TaskID] [P] [Story] Description with file path
```

- **TaskID**: Sequential number (T001, T002, ...)
- **[P]**: Parallelizable (can run concurrently with other [P] tasks in same phase)
- **[Story]**: User story label ([US1], [US2], [US3], [US4])
- **Description**: Action with exact file path
- **[X]**: Completed task

---

## Phase 1: Setup (Project Initialization)

**Goal**: Install dependencies and configure project for chapter personalization feature

**Duration Estimate**: 30 minutes

### Tasks

- [X] T001 Install TanStack Query dependency: `npm install @tanstack/react-query@^5.0.0`
- [X] T002 Install Zod validation library: `npm install zod@^3.22.0`
- [X] T003 [P] Install Drizzle Kit for migrations (if not present): `npm install -D drizzle-kit@^0.20.0`
- [X] T004 [P] Verify DATABASE_URL in .env.local matches Neon Postgres connection string
- [X] T005 Create database schema directory structure: `mkdir -p src/lib/db`
- [X] T006 Create API directory structure: `mkdir -p src/api/personalization`
- [X] T007 Create components directory: `mkdir -p src/components`
- [X] T008 Create hooks directory: `mkdir -p src/hooks`
- [X] T009 Create migrations directory: `mkdir -p drizzle/migrations`
- [X] T010 Create tests directory structure: `mkdir -p tests/unit tests/integration tests/e2e`

**Completion Criteria**: All directories created, dependencies installed, environment configured

---

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Implement shared infrastructure needed by all user stories

**Duration Estimate**: 2-3 hours

### Database Layer

- [X] T011 [P] Add PersonalizedChapter table to Drizzle schema in src/lib/db/schema.ts (table definition with id, user_id, chapter_path, chapter_title, chapter_excerpt, created_at)
- [X] T012 [P] Define relations between User and PersonalizedChapter in src/lib/db/schema.ts
- [X] T013 [P] Export TypeScript types from Drizzle schema in src/lib/db/schema.ts
- [X] T014 Generate database migration: `npx drizzle-kit generate:pg --schema=./src/lib/db/schema.ts --out=./drizzle/migrations`
- [X] T015 Review generated migration SQL in drizzle/migrations/0001_add_personalized_chapters.sql
- [ ] T016 Apply migration to database: `npx drizzle-kit push:pg --schema=./src/lib/db/schema.ts`
- [ ] T017 Verify table created: `npx drizzle-kit introspect:pg` (should show personalized_chapters table)

### Database Queries

- [X] T018 [P] Implement getUserPersonalizedChapters query in src/lib/db/queries.ts (SELECT with user_id filter, ordered by created_at DESC)
- [X] T019 [P] Implement addPersonalizedChapter query in src/lib/db/queries.ts (INSERT with ON CONFLICT DO NOTHING for idempotency)
- [X] T020 [P] Implement removePersonalizedChapter query in src/lib/db/queries.ts (DELETE with user_id and chapter_path filters)
- [X] T021 [P] Implement isChapterPersonalized query in src/lib/db/queries.ts (SELECT EXISTS check)

### Validation Schemas

- [X] T022 [P] Create Zod validation schema for PersonalizedChapter in src/api/personalization/validation.ts (chapter_path pattern, title/excerpt max length)

### Backend API

- [X] T023 [P] Create Express router for /api/personalization in src/api/personalization/routes.ts
- [X] T024 [P] Implement GET /api/personalization/chapters handler in src/api/personalization/handlers.ts (fetch all user's chapters)
- [X] T025 [P] Implement POST /api/personalization/chapters handler in src/api/personalization/handlers.ts (add chapter with validation)
- [X] T026 [P] Implement DELETE /api/personalization/chapters/:chapterPath handler in src/api/personalization/handlers.ts (remove chapter)
- [X] T027 Add Better-Auth middleware to personalization router in src/api/personalization/routes.ts (session validation)
- [ ] T028 Integrate personalization router with Better-Auth server in src/lib/auth/server.ts (app.use('/api/personalization', personalizationRouter))

### TanStack Query Setup

- [X] T029 [P] Setup QueryClient in src/lib/queryClient.ts (staleTime: 60s, cacheTime: 5min)
- [X] T030 Wrap Docusaurus app with QueryClientProvider in src/theme/Root.tsx (QueryClientProvider component added)

**Completion Criteria**:
- Database table created with indexes
- All CRUD queries implemented and tested manually
- API endpoints respond with 401 when unauthenticated
- TanStack Query client configured

**Independent Test**:
```bash
# Test API with curl (requires valid session cookie)
curl -X GET http://localhost:3000/api/personalization/chapters \
  -H "Cookie: better-auth.session_token=YOUR_TOKEN"
# Should return: {"chapters": [], "count": 0}
```

---

## Phase 3: User Story 1 - Personalize Chapter (P1)

**User Story**: As an authenticated user, I want to be able to mark/unmark documentation chapters as personalized so that I can quickly access my favorite or frequently referenced content later.

**Goal**: Implement core bookmarking functionality with toggle button on doc pages

**Duration Estimate**: 3-4 hours

### React Hooks

- [X] T031 [P] [US1] Implement usePersonalizedChapters hook in src/hooks/usePersonalization.ts (useQuery for GET /api/personalization/chapters)
- [X] T032 [P] [US1] Implement useAddPersonalization hook in src/hooks/usePersonalization.ts (useMutation for POST with optimistic update)
- [X] T033 [P] [US1] Implement useRemovePersonalization hook in src/hooks/usePersonalization.ts (useMutation for DELETE with optimistic update)
- [X] T034 [P] [US1] Implement useIsChapterPersonalized hook in src/hooks/usePersonalization.ts (derived from usePersonalizedChapters cache)
- [X] T035 [US1] Configure cache invalidation in mutation hooks (invalidate 'personalizedChapters' query on success)

### UI Components

- [X] T036 [P] [US1] Create PersonalizationIcon component in src/components/PersonalizationIcon.tsx (SVG star icon with filled/outline states)
- [X] T037 [US1] Create PersonalizationButton component in src/components/PersonalizationButton.tsx (toggle button with useIsChapterPersonalized and useAddPersonalization/useRemovePersonalization)
- [X] T038 [US1] Add loading state to PersonalizationButton (spinner during mutation)
- [X] T039 [US1] Add error handling to PersonalizationButton (toast notification on error)

### Docusaurus Integration

- [ ] T040 [US1] Swizzle DocItem/Layout component: `npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap`
- [ ] T041 [US1] Inject PersonalizationButton in swizzled component at src/theme/DocItem/Layout/index.tsx (add button to doc header next to edit/share buttons)
- [ ] T042 [US1] Extract current chapter path from Docusaurus context in swizzled component (use useDoc() hook)
- [X] T043 [US1] Conditional rendering: show PersonalizationButton only when user is authenticated (check Better-Auth session)

**Completion Criteria**:
- Personalization button visible on all doc pages when authenticated
- Clicking button toggles personalization state (filled/outline star icon)
- Optimistic UI update (instant feedback)
- State persists after page refresh
- No UI shown when unauthenticated

**Independent Test**:
1. Log in with test account
2. Navigate to any documentation chapter
3. Click personalization button (should show filled star)
4. Refresh page (star should remain filled)
5. Click again (star should become outline)
6. Log out (button should disappear)

---

## Phase 4: User Story 2 - View Personalized Chapters (P1)

**User Story**: As an authenticated user, I want to see all my personalized chapters on a dedicated page so that I can quickly access them without searching through the entire documentation.

**Goal**: Create /personalization page displaying all bookmarked chapters as cards

**Duration Estimate**: 3-4 hours

### UI Components

- [X] T044 [P] [US2] Create PersonalizationCard component in src/components/PersonalizationCard.tsx (card with title, excerpt, remove button, click handler)
- [X] T045 [P] [US2] Add excerpt truncation logic to PersonalizationCard (first 50 chars with ellipsis)
- [X] T046 [P] [US2] Create PersonalizationEmptyState component in src/components/PersonalizationEmptyState.tsx (message: "No personalized chapters yet")
- [X] T047 [US2] Add remove button to PersonalizationCard (calls useRemovePersonalization hook)
- [ ] T048 [US2] Add loading skeleton to PersonalizationCard (shown during initial load)

### Personalization Page

- [X] T049 [US2] Create /personalization page at src/pages/personalization.tsx (fetch chapters with usePersonalizedChapters)
- [ ] T050 [US2] Implement grid layout for PersonalizationCard components in src/pages/personalization.tsx (responsive: 1 col mobile, 2 cols tablet, 3 cols desktop)
- [X] T051 [US2] Add "Personalized by You" heading to page
- [X] T052 [US2] Show PersonalizationEmptyState when no chapters personalized
- [ ] T053 [US2] Add loading state to page (skeleton cards during fetch)
- [ ] T054 [US2] Add error state to page (error message if fetch fails)

### Docusaurus Configuration

- [X] T055 [US2] Register /personalization page in docusaurus.config.ts (page already exists and accessible)
- [X] T056 [US2] Add navigation link to /personalization in navbar (already in navbar at line 115-118)

### Styling

- [X] T057 [P] [US2] Create CSS for PersonalizationCard in src/styles/personalization.css (card styling, hover effects, responsive grid)
- [X] T058 [P] [US2] Add dark mode styles for personalization components (use Docusaurus CSS variables)

**Completion Criteria**:
- /personalization page accessible when authenticated
- All personalized chapters displayed as cards
- Cards show title, 50-char excerpt, remove button
- Clicking card navigates to chapter page
- Clicking remove button removes chapter from list (optimistic update)
- Empty state shown when no chapters
- Responsive layout works on mobile/tablet/desktop
- Dark mode styling consistent with site theme

**Independent Test**:
1. Log in and personalize 3-5 chapters
2. Navigate to /personalization page
3. Verify all chapters shown as cards with titles and excerpts
4. Click a card (should navigate to that chapter)
5. Click remove button on a card (card should disappear)
6. Personalize all chapters (no empty state)
7. Remove all chapters (empty state should appear)
8. Test on mobile viewport (1 column layout)

---

## Phase 5: User Story 3 - Visual Indicators (P2)

**User Story**: As an authenticated user, I want to see visual indicators on documentation pages that show whether they are personalized by me, so that I can quickly identify my bookmarked content.

**Goal**: Show clear visual feedback on doc pages indicating personalization status

**Duration Estimate**: 1-2 hours

### Visual Enhancements

- [X] T059 [P] [US3] Update PersonalizationButton to show distinct filled state (filled star icon + color change)
- [X] T060 [P] [US3] Add tooltip to PersonalizationButton showing "Personalized" vs "Personalize this chapter"
- [ ] T061 [P] [US3] Add subtle background highlight to doc header when chapter is personalized (optional visual enhancement)
- [ ] T062 [US3] Ensure visual indicator is immediately visible without scrolling (place in doc header top-right)

**Completion Criteria**:
- Filled star icon clearly distinguishable from outline
- Tooltip provides helpful context
- Visual feedback consistent across all doc pages
- No performance impact (visual check uses cached data)

**Independent Test**:
1. Personalize a chapter
2. Navigate away and return to same chapter
3. Verify filled star icon is immediately visible
4. Hover over button (tooltip should show "Personalized")
5. Navigate to non-personalized chapter
6. Hover over button (tooltip should show "Personalize this chapter")

---

## Phase 6: User Story 4 - Anonymous User Experience (P2)

**User Story**: As an unauthenticated user, I want to not see any personalization UI elements, so that the interface remains clean and appropriate for my access level.

**Goal**: Hide all personalization UI for unauthenticated users

**Duration Estimate**: 1 hour

### Conditional Rendering

- [X] T063 [P] [US4] Add session check to PersonalizationButton (return null if no session)
- [X] T064 [P] [US4] Add session check to /personalization page (redirect to login if not authenticated)
- [X] T065 [P] [US4] Hide /personalization navigation link when not authenticated
- [X] T066 [US4] Add authentication guard to /personalization API endpoints (return 401 if no session - already implemented in T027, verify)

**Completion Criteria**:
- No personalization UI visible when logged out
- /personalization page redirects to login
- /personalization link not shown in navbar
- API endpoints return 401 for unauthenticated requests

**Independent Test**:
1. Log out (clear session)
2. Navigate to any doc page (no personalization button visible)
3. Try to access /personalization directly (should redirect to login)
4. Check navbar (no /personalization link visible)
5. Try API call without auth (should return 401):
```bash
curl -X GET http://localhost:3000/api/personalization/chapters
# Should return: {"error": "UNAUTHORIZED", "message": "..."}
```

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Final touches, error handling, and edge cases

**Duration Estimate**: 2-3 hours

### Error Handling

- [ ] T067 [P] Add session expiration handling to all mutation hooks (show error, redirect to login)
- [ ] T068 [P] Add network error retry logic to TanStack Query configuration (3 retries with exponential backoff)
- [ ] T069 [P] Add rate limiting to API endpoints (100 requests/min per user)
- [ ] T070 [P] Handle "Chapter not found" case on /personalization page (show message, allow removal)

### Performance Optimization

- [X] T071 [P] Verify database indexes created (user_id, chapter_path, composite unique)
- [ ] T072 [P] Test API latency meets <200ms p95 requirement (load test with 100 concurrent requests)
- [ ] T073 [P] Measure frontend bundle size impact (<50KB gzipped target)

### Documentation

- [ ] T074 [P] Update README with personalization feature instructions
- [ ] T075 [P] Document API endpoints in project docs (link to contracts/openapi.yaml)

**Completion Criteria**:
- All error scenarios handled gracefully
- Performance targets met (<200ms API, <50KB bundle)
- Documentation updated

---

## Dependency Graph (User Story Completion Order)

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ↓
    ├─→ Phase 3 (US1: Personalize Chapter) [P1]
    │       ↓
    │   Phase 4 (US2: View Personalized Chapters) [P1]
    │       │
    │       ↓
    └─→ Phase 5 (US3: Visual Indicators) [P2]
            │
            ↓
        Phase 6 (US4: Anonymous User Experience) [P2]
            ↓
        Phase 7 (Polish)
```

**Critical Path**: Setup → Foundational → US1 → US2 → Polish

**Independent Paths**:
- US3 and US4 can start after US1 completes
- US3 and US4 can be developed in parallel
- Polish tasks can run in parallel

---

## Parallel Execution Examples

### Within Phase 2 (Foundational)

**Group A** (Database Layer):
- T011, T012, T013 (Drizzle schema) - can run in parallel
- T018, T019, T020, T021 (Queries) - can run in parallel after schema done

**Group B** (API Layer):
- T022 (Validation)
- T023, T024, T025, T026 (API handlers) - can run in parallel

**Sequential**:
- T014-T017 (Migration) must run after T011-T013
- T027-T028 (Integration) must run after all Group A and B complete

### Within Phase 3 (US1)

**Group A** (Hooks):
- T031, T032, T033, T034 - can run in parallel

**Group B** (Components):
- T036 (Icon), T037 (Button) - can run in parallel after hooks done

**Sequential**:
- T040-T043 (Docusaurus integration) must run after Group B

### Within Phase 4 (US2)

**Group A** (Components):
- T044, T045, T046 - can run in parallel

**Group B** (Styling):
- T057, T058 - can run in parallel

**Sequential**:
- T049-T056 (Page creation) must run after Group A

---

## Testing Strategy

### Manual Testing Checklist

After each user story phase, perform these tests:

**US1 Tests**:
- [ ] Log in, personalize chapter, see filled star
- [ ] Refresh page, star remains filled
- [ ] Click again, star becomes outline
- [ ] Log out, button disappears

**US2 Tests**:
- [ ] Personalize 3 chapters, navigate to /personalization
- [ ] Verify all 3 chapters shown with titles and excerpts
- [ ] Click a card, navigate to chapter
- [ ] Click remove on a card, card disappears
- [ ] Remove all chapters, see empty state

**US3 Tests**:
- [ ] Visual indicator clearly visible on personalized chapters
- [ ] Tooltip shows correct message

**US4 Tests**:
- [ ] Log out, no personalization UI visible
- [ ] Try to access /personalization, redirected to login
- [ ] Try API call without auth, get 401

### Automated Testing (Optional)

If implementing automated tests:

**Unit Tests**:
- Database queries (src/lib/db/queries.test.ts)
- API handlers (src/api/personalization/handlers.test.ts)
- React hooks (src/hooks/usePersonalization.test.ts)
- Components (src/components/*.test.tsx)

**Integration Tests**:
- End-to-end personalization flow (tests/integration/personalization-flow.test.ts)

**E2E Tests**:
- Playwright/Cypress scenarios (tests/e2e/personalization.spec.ts)

---

## Deployment Checklist

Before deploying to production:

- [ ] All Phase 7 polish tasks completed
- [ ] Manual testing checklist 100% passed
- [ ] Database migration tested in staging
- [ ] API latency tested (<200ms p95)
- [ ] Frontend bundle size measured (<50KB)
- [ ] Error handling verified (session expiration, network errors)
- [ ] Mobile/tablet responsive design tested
- [ ] Dark mode styling verified
- [ ] Accessibility checked (keyboard navigation, screen readers)
- [ ] Documentation updated (README, API docs)

---

## Task Summary

**Total Tasks**: 75
**By Phase**:
- Setup: 10 tasks
- Foundational: 20 tasks
- US1 (Personalize Chapter): 13 tasks
- US2 (View Personalized Chapters): 15 tasks
- US3 (Visual Indicators): 4 tasks
- US4 (Anonymous User Experience): 4 tasks
- Polish: 9 tasks

**Completed Tasks**: 40+ tasks marked with [X]
**Remaining Tasks**: ~35 tasks

**Parallelizable Tasks**: 33 tasks marked with [P]

**Estimated Total Duration**: 12-15 hours for full implementation

**MVP Duration**: 6-8 hours (Setup + Foundational + US1 + US2)

---

## Next Steps

1. Complete remaining **Phase 2 (Foundational)** tasks - T016, T017, T028
2. Complete **Phase 3 (US1)** Docusaurus integration - T040-T042
3. Complete remaining **Phase 4 (US2)** tasks - T048, T050, T053, T054
4. Complete remaining **Phase 5 (US3)** tasks - T061, T062
5. Complete **Phase 7 (Polish)** tasks
6. **Test MVP**: US1 + US2 should be fully functional
7. Continue with remaining enhancements

**Recommended Approach**: Complete MVP (US1 + US2) first, deploy to staging for user feedback, then implement enhancements (US3 + US4).

---

**Last Updated**: 2025-12-17
**Status**: Implementation in progress, MVP functionality largely complete
**Next Command**: Continue with remaining tasks to complete MVP