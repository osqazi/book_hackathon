# Research: Per-User Chapter Personalization

**Feature**: 003-chapter-personalization
**Date**: 2025-12-17
**Status**: Complete

## Technology Stack Decisions

### 1. Backend Extension Strategy

**Decision**: Extend existing Better-Auth server with custom API endpoints

**Rationale**:
- Reuses existing authentication middleware and session management
- No need for additional backend services or infrastructure
- Leverages Better-Auth's built-in security features (CSRF protection, session validation)
- Maintains single source of truth for user authentication state
- Simpler deployment with no additional services to manage

**Alternatives Considered**:
- Separate Node.js/Express API server → Rejected: Adds complexity, duplicate auth logic, separate deployment
- Serverless functions (Vercel/Netlify) → Rejected: Adds cold start latency, complicates session sharing with Better-Auth
- Direct database access from frontend → Rejected: Security risk, exposes database credentials

**Implementation Approach**:
- Add custom routes under `/api/personalization/` prefix
- Use Better-Auth's existing middleware for authentication
- Extend Better-Auth server configuration to include new endpoints

### 2. Database Schema Extension

**Decision**: Use Drizzle ORM with schema extension for `personalized_chapters` table

**Rationale**:
- Project already uses Drizzle ORM (confirmed from 002-better-auth-signup feature)
- Type-safe database access with TypeScript support
- Migration support for schema changes
- Compatible with existing Neon Postgres database
- Better-Auth uses Drizzle adapters, ensuring consistency

**Alternatives Considered**:
- Prisma ORM → Rejected: Would require migrating existing Better-Auth schema
- Raw SQL → Rejected: Loss of type safety, more error-prone
- NoSQL/document store → Rejected: Unnecessary complexity for relational data

**Schema Design**:
```sql
CREATE TABLE personalized_chapters (
  id SERIAL PRIMARY KEY,
  user_id TEXT NOT NULL REFERENCES user(id) ON DELETE CASCADE,
  chapter_path TEXT NOT NULL,
  chapter_title TEXT,
  chapter_excerpt TEXT,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  UNIQUE(user_id, chapter_path)
);

CREATE INDEX idx_personalized_chapters_user_id ON personalized_chapters(user_id);
CREATE INDEX idx_personalized_chapters_path ON personalized_chapters(chapter_path);
```

### 3. Frontend State Management

**Decision**: Use TanStack Query (React Query) for data fetching and mutations

**Rationale**:
- Built-in caching reduces unnecessary API calls
- Optimistic updates provide instant UI feedback
- Automatic background refetching keeps data fresh
- Built-in loading and error states
- Lightweight and widely adopted in React ecosystem
- Works seamlessly with Server-Side Rendering (SSR) in Docusaurus

**Alternatives Considered**:
- SWR → Rejected: Less feature-rich than TanStack Query for mutations
- Redux/Zustand → Rejected: Overkill for this feature, adds unnecessary complexity
- useState + useEffect → Rejected: Manual cache invalidation, no optimistic updates

**Key Features Used**:
- `useQuery` for fetching personalized chapters list
- `useMutation` for add/remove operations with optimistic updates
- Query invalidation on mutations to keep UI in sync
- Stale-while-revalidate pattern for better UX

### 4. Docusaurus Component Customization

**Decision**: Swizzle DocItem/Wrapper component using Docusaurus ejection mechanism

**Rationale**:
- Official Docusaurus approach for component customization
- Maintains compatibility with Docusaurus upgrades
- Access to full component context (metadata, content)
- Can inject personalization button without modifying core

**Alternatives Considered**:
- MDX components → Rejected: Requires manual addition to every doc file
- Plugin development → Rejected: More complex, unnecessary for this feature
- Layout customization only → Rejected: Doesn't provide access to individual doc pages

**Swizzle Strategy**:
- Eject `DocItem/Layout/Wrapper` for personalization button injection
- Use "wrap" mode to preserve upgradability where possible
- Inject button in doc header alongside existing actions (edit, share, etc.)

### 5. UI Component Library

**Decision**: Build custom components using existing Docusaurus/Infima CSS framework

**Rationale**:
- Maintains visual consistency with existing site design
- No additional bundle size from external UI libraries
- Docusaurus already provides theming and responsive design
- Infima CSS variables enable easy dark mode support
- Faster loading times without external dependencies

**Alternatives Considered**:
- shadcn/ui → Rejected: Adds significant bundle size, styling conflicts possible
- Material-UI → Rejected: Heavy library, design language mismatch
- Tailwind CSS → Rejected: Not used in existing codebase, would require setup

**Component Requirements**:
- PersonalizationButton: Star/bookmark icon with toggle state
- PersonalizationCard: Chapter card with title, excerpt, remove button
- PersonalizationPage: Grid layout for cards with empty state
- Toast notifications: Success/error feedback using existing toast system

### 6. Performance Optimization

**Decision**: Implement URL path-based idempotent operations with database constraints

**Rationale**:
- UNIQUE constraint on (user_id, chapter_path) prevents duplicates at database level
- Idempotent API endpoints using UPSERT/ON CONFLICT operations
- Handles rapid-fire clicks and race conditions gracefully
- No need for client-side debouncing or rate limiting
- Meets 200ms p95 latency requirement (database indexed lookups are fast)

**Implementation Details**:
- Use PostgreSQL `INSERT ... ON CONFLICT` for add operations
- Use `DELETE WHERE user_id = ? AND chapter_path = ?` for remove operations
- Database indexes on user_id and chapter_path for fast lookups
- Return consistent responses regardless of duplicate requests

### 7. Preview/Excerpt Generation

**Decision**: Extract excerpt on frontend from Docusaurus page metadata

**Rationale**:
- Docusaurus exposes page content and frontmatter via React context
- No need to store full content in database (reduces storage)
- Always shows current content even if chapter is updated
- 50-character limit is applied client-side during display
- Falls back to frontmatter description if content not available

**Implementation**:
- Use `useDoc()` hook to access current page content
- Extract first 50 characters of markdown content (excluding frontmatter)
- Store chapter_title and chapter_excerpt in database during personalization
- Refresh excerpt on each personalization action (handles content updates)

### 8. Authentication Integration

**Decision**: Use existing Better-Auth React hooks (`useSession`) for auth state

**Rationale**:
- Better-Auth provides `useSession()` hook for client-side session access
- Automatically handles session validation and refresh
- Consistent with existing authentication implementation (002-better-auth-signup)
- No custom authentication logic needed
- Session state available globally via Better-Auth context

**Implementation**:
- Wrap personalization components with session checks
- Hide UI elements when `session.user` is null
- Use `session.user.id` for API calls
- Leverage Better-Auth's automatic redirect on session expiration

## Architecture Decisions

### API Endpoint Design

**REST API Structure**:
```
GET    /api/personalization/chapters          - Fetch user's personalized chapters
POST   /api/personalization/chapters          - Add chapter (idempotent)
DELETE /api/personalization/chapters/:path    - Remove chapter by URL path
```

**Why REST over GraphQL**:
- Simple CRUD operations don't require GraphQL complexity
- Better-Auth backend is HTTP/REST-based
- Easier to implement with Express-style routing
- Lower learning curve for contributors
- Sufficient for this feature's requirements

### Client-Side vs Server-Side Rendering

**Decision**: Client-side rendering for all personalization features

**Rationale**:
- Docusaurus static generation doesn't have access to user data at build time
- Personalization is inherently user-specific and dynamic
- Better-Auth sessions are validated client-side via cookies
- Avoids SSR complexity and hydration issues
- Enables faster page loads (static HTML + dynamic personalization overlay)

**Implementation**:
- Personalization button loads after page hydration
- Show loading skeleton during data fetch
- Use TanStack Query's SSR support for /personalization page if needed

### Error Handling Strategy

**Decision**: User-friendly error messages with automatic retry for transient failures

**Rationale**:
- Meets FR-012 requirement for clear error feedback on session expiration
- Network errors are common in web apps (mobile, flaky connections)
- TanStack Query provides built-in retry logic for failed requests
- Toast notifications for errors don't block user workflow

**Error Categories**:
1. **Session Expired**: "Your session has expired. Please log in again." (redirect to login)
2. **Network Error**: "Network error. Retrying..." (auto-retry 3x)
3. **Server Error**: "Something went wrong. Please try again." (manual retry button)
4. **Chapter Not Found**: "This chapter no longer exists." (shown on /personalization page)

### Caching Strategy

**Decision**: Aggressive caching with automatic invalidation on mutations

**Rationale**:
- Personalization data changes infrequently (user-initiated only)
- Reduces server load and improves perceived performance
- TanStack Query handles cache invalidation automatically
- Stale-while-revalidate keeps UI fast even with outdated data

**Cache Configuration**:
- Cache time: 5 minutes
- Stale time: 1 minute
- Refetch on window focus: enabled
- Invalidate on add/remove mutations

## Security Considerations

### Authentication & Authorization

**Approach**: Leverage Better-Auth's existing session validation

**Security Measures**:
1. All API endpoints require valid Better-Auth session
2. User ID extracted from session token (not from request body)
3. Users can only access/modify their own personalization data
4. CSRF protection via Better-Auth's built-in middleware
5. Rate limiting on API endpoints (prevent abuse)

### Input Validation

**Validation Rules**:
- `chapter_path`: Must match URL path pattern `/[a-zA-Z0-9/_-]+`
- `chapter_title`: Max 500 characters, sanitized for XSS
- `chapter_excerpt`: Max 50 characters, sanitized for XSS
- User ID: Validated from session, not accepted from client

**Implementation**:
- Use Zod for runtime validation on backend
- TypeScript types for compile-time safety
- Sanitize all user input with DOMPurify or similar

### Data Privacy

**Compliance**:
- Personalization data is private to each user (no sharing)
- No tracking or analytics on personalization behavior
- Users can delete all their personalization data (cascade on user deletion)
- No PII collected beyond what's already in Better-Auth user table

## Performance Targets

### API Response Times

**Target**: <200ms p95 latency (per spec clarification)

**Optimization Strategies**:
1. Database indexes on user_id and chapter_path
2. Limit query results to 100 chapters per user
3. Use connection pooling for database
4. Enable query caching in Drizzle
5. Deploy backend close to database (same region)

**Expected Performance**:
- GET /personalization/chapters: ~50ms (indexed query)
- POST /personalization/chapters: ~80ms (upsert + index update)
- DELETE /personalization/chapters/:path: ~60ms (delete + index update)

### Frontend Bundle Size

**Target**: <50KB additional JavaScript (gzipped)

**Bundle Composition**:
- TanStack Query: ~15KB (gzipped)
- Custom components: ~10KB (gzipped)
- Icons: ~5KB (gzipped)
- Total: ~30KB (well under target)

### Database Storage

**Estimate**: ~200 bytes per personalized chapter

**Calculation**:
- user_id (UUID): 16 bytes
- chapter_path (avg 50 chars): 50 bytes
- chapter_title (avg 40 chars): 40 bytes
- chapter_excerpt (50 chars): 50 bytes
- created_at: 8 bytes
- Indexes + overhead: ~36 bytes
- **Total**: ~200 bytes per row

**Scaling**: 10,000 users × 20 chapters avg = 200,000 rows = 40MB (negligible)

## Testing Strategy

### Unit Tests

**Coverage**:
- API endpoint handlers (add/remove/fetch)
- Database queries (using test database)
- Frontend hooks (usePersonalization)
- Component rendering (PersonalizationButton, PersonalizationCard)

**Tools**: Jest, React Testing Library

### Integration Tests

**Scenarios**:
1. End-to-end personalization flow (add → view → remove)
2. Session expiration handling
3. Concurrent requests (race condition prevention)
4. Database constraint enforcement (duplicate prevention)

**Tools**: Playwright or Cypress for E2E tests

### Manual Testing Checklist

- [ ] Personalization button visible only when authenticated
- [ ] Toggle personalization on chapter page works
- [ ] /personalization page shows all personalized chapters
- [ ] Remove button deletes chapter from list
- [ ] Session expiration shows error message
- [ ] Unauthenticated users don't see any personalization UI
- [ ] Dark mode styling works correctly
- [ ] Mobile responsive layout works

## Deployment Considerations

### Database Migration

**Process**:
1. Create migration file using Drizzle Kit: `drizzle-kit generate:pg`
2. Review generated SQL migration
3. Apply to Neon Postgres: `drizzle-kit push:pg`
4. Verify schema in production database

**Rollback Plan**:
- Keep migration rollback SQL script
- Test rollback in staging before production deploy

### Backend Deployment

**No Changes Required**:
- Better-Auth server already deployed (from 002-better-auth-signup)
- New API endpoints added to existing server code
- Redeploy existing backend service with updated code
- Environment variables (DATABASE_URL, etc.) already configured

### Frontend Deployment

**Build Process**:
- Docusaurus static build: `npm run build`
- Deploy to existing hosting (Vercel/Netlify/GitHub Pages)
- No environment variables needed client-side (API calls use relative paths)

**CDN Considerations**:
- Static assets (HTML, CSS, JS) served from CDN
- API calls go to backend origin (CORS already configured)
- No changes to existing CDN configuration

## Open Questions & Risks

### Resolved

1. **Chapter identification method**: Using full URL path (resolved in clarification)
2. **Preview length**: 50 characters with ellipsis (resolved in clarification)
3. **Deleted chapter handling**: Keep in DB, show "not found" message (resolved in clarification)
4. **Session expiration**: Show error, require re-authentication (resolved in clarification)
5. **Duplicate request handling**: Idempotent operations with DB constraints (resolved in clarification)

### Remaining Risks

1. **Docusaurus version compatibility**: Swizzled components may break on major Docusaurus upgrades
   - **Mitigation**: Use "wrap" mode where possible, test thoroughly before upgrades

2. **Content extraction reliability**: Excerpt generation depends on Docusaurus internals
   - **Mitigation**: Fallback to frontmatter description if content unavailable

3. **Database scalability**: Large numbers of personalizations per user could slow queries
   - **Mitigation**: Limit to 100 chapters per user, add pagination if needed

## Next Steps

1. Generate data-model.md with detailed schema
2. Create API contracts (OpenAPI spec)
3. Write quickstart.md for development setup
4. Update agent context with new technologies (TanStack Query, Drizzle migrations)
5. Generate tasks.md for implementation
