# Quick Start Guide: Chapter Personalization Feature

**Feature**: 003-chapter-personalization
**Last Updated**: 2025-12-17

This guide provides step-by-step instructions for developers to set up, develop, and test the chapter personalization feature.

## Prerequisites

Before starting, ensure you have:

- **Node.js**: v18.0.0 or higher
- **npm**: v9.0.0 or higher
- **PostgreSQL**: Access to Neon Postgres database (or local Postgres for development)
- **Better-Auth**: Already configured and working (from 002-better-auth-signup feature)
- **Git**: For version control and branch management
- **Code Editor**: VS Code (recommended) or similar

**Required Knowledge**:
- TypeScript/JavaScript
- React and React Hooks
- Docusaurus basics
- REST APIs
- SQL/PostgreSQL basics

## Setup Instructions

### 1. Clone and Branch

```bash
# Ensure you're in the project root
cd humanoid-robotics-book

# Checkout the feature branch
git checkout 003-chapter-personalization

# Pull latest changes
git pull origin 003-chapter-personalization

# Install dependencies (if not already done)
npm install
```

### 2. Install New Dependencies

```bash
# Install TanStack Query for state management
npm install @tanstack/react-query@^5.0.0

# Install Drizzle Kit for database migrations (if not already installed)
npm install -D drizzle-kit@^0.20.0

# Install validation library
npm install zod@^3.22.0
```

### 3. Environment Configuration

**File**: `.env.local` (create if it doesn't exist)

```env
# Database (should already be configured from Better-Auth setup)
DATABASE_URL="postgresql://user:password@your-neon-db.neon.tech/dbname?sslmode=require"

# Better-Auth (should already be configured)
BETTER_AUTH_SECRET="your-secret-key-here"
BETTER_AUTH_URL="http://localhost:3000"

# Optional: Rate limiting configuration
RATE_LIMIT_MAX_REQUESTS=100
RATE_LIMIT_WINDOW_MS=60000
```

**Verification**:
```bash
# Test database connection
npx drizzle-kit introspect:pg
```

### 4. Database Migration

**Step 1**: Generate migration from schema

```bash
# Navigate to database schema location
cd src/lib/db  # Adjust path based on your project structure

# Generate migration SQL
npx drizzle-kit generate:pg --schema=./schema.ts --out=../../drizzle/migrations

# Review the generated SQL file in drizzle/migrations/
```

**Step 2**: Apply migration to database

```bash
# Apply migration
npx drizzle-kit push:pg --schema=./src/lib/db/schema.ts

# Verify table was created
npx drizzle-kit introspect:pg
# Should show personalized_chapters table
```

**Manual Verification** (optional):

```sql
-- Connect to your database and run:
SELECT * FROM personalized_chapters LIMIT 1;
-- Should return empty result (no error)
```

### 5. Start Development Server

**Terminal 1** (Backend - Better-Auth server):
```bash
# Start the backend server
npm run dev:backend
# Or if using a different command:
# node src/auth/server.js
```

**Terminal 2** (Frontend - Docusaurus):
```bash
# Start Docusaurus development server
npm run start

# Access at http://localhost:3000
```

**Verification**:
- Navigate to `http://localhost:3000`
- Log in with test user credentials
- Check browser console for no errors

## Development Workflow

### File Structure

After implementation, your project should have this structure:

```
humanoid-robotics-book/
├── src/
│   ├── lib/
│   │   ├── db/
│   │   │   ├── schema.ts                    # Database schema with PersonalizedChapter
│   │   │   ├── queries.ts                   # Database queries (CRUD operations)
│   │   │   └── client.ts                    # Drizzle DB client
│   │   └── auth/
│   │       └── server.ts                    # Better-Auth server (existing)
│   ├── api/
│   │   └── personalization/
│   │       └── routes.ts                    # API endpoint handlers
│   ├── components/
│   │   ├── PersonalizationButton.tsx        # Toggle button for doc pages
│   │   ├── PersonalizationCard.tsx          # Chapter card component
│   │   └── PersonalizationIcon.tsx          # Star/bookmark icon
│   ├── hooks/
│   │   └── usePersonalization.ts            # TanStack Query hooks
│   ├── pages/
│   │   └── personalization.tsx              # /personalization page
│   └── theme/
│       └── DocItem/
│           └── Layout/
│               └── index.tsx                # Swizzled Docusaurus component
├── drizzle/
│   └── migrations/
│       └── 0001_add_personalized_chapters.sql
├── specs/
│   └── 003-chapter-personalization/
│       ├── spec.md
│       ├── plan.md
│       ├── research.md
│       ├── data-model.md
│       ├── quickstart.md
│       └── contracts/
│           └── openapi.yaml
└── package.json
```

### Step-by-Step Implementation

#### Phase 1: Database Layer

**File**: `src/lib/db/schema.ts`

1. Add PersonalizedChapter table definition (see data-model.md)
2. Define relations between User and PersonalizedChapter
3. Export TypeScript types

**File**: `src/lib/db/queries.ts`

1. Implement `getUserPersonalizedChapters(userId)`
2. Implement `addPersonalizedChapter(userId, chapterPath, title, excerpt)`
3. Implement `removePersonalizedChapter(userId, chapterPath)`
4. Implement `isChapterPersonalized(userId, chapterPath)`

**Test**:
```bash
# Create a test file: src/lib/db/queries.test.ts
npm run test -- queries.test.ts
```

#### Phase 2: Backend API

**File**: `src/api/personalization/routes.ts`

1. Create Express router (or extend existing Better-Auth server)
2. Add Better-Auth middleware for authentication
3. Implement GET `/api/personalization/chapters`
4. Implement POST `/api/personalization/chapters`
5. Implement DELETE `/api/personalization/chapters/:chapterPath`
6. Add input validation with Zod
7. Add error handling

**Integration Point**: `src/lib/auth/server.ts`

```typescript
import { personalizationRouter } from '@/api/personalization/routes';

// In Better-Auth server setup:
app.use('/api/personalization', personalizationRouter);
```

**Test**:
```bash
# Use curl or Postman to test endpoints
curl -X GET http://localhost:3000/api/personalization/chapters \
  -H "Cookie: better-auth.session_token=YOUR_SESSION_TOKEN"
```

#### Phase 3: Frontend Hooks

**File**: `src/hooks/usePersonalization.ts`

1. Setup TanStack Query client
2. Implement `usePersonalizedChapters()` hook (GET query)
3. Implement `useAddPersonalization()` hook (POST mutation)
4. Implement `useRemovePersonalization()` hook (DELETE mutation)
5. Implement `useIsChapterPersonalized(chapterPath)` hook
6. Configure optimistic updates and cache invalidation

**Setup QueryClient**: `src/pages/_app.tsx` or `docusaurus.config.js`

```typescript
import { QueryClient, QueryClientProvider } from '@tanstack/react-query';

const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      staleTime: 60 * 1000, // 1 minute
      cacheTime: 5 * 60 * 1000, // 5 minutes
    },
  },
});

// Wrap your app with QueryClientProvider
```

#### Phase 4: UI Components

**File**: `src/components/PersonalizationButton.tsx`

1. Create button component with star/bookmark icon
2. Use `useIsChapterPersonalized()` to check state
3. Use `useAddPersonalization()` and `useRemovePersonalization()` for toggle
4. Add loading and error states
5. Show only when user is authenticated

**File**: `src/components/PersonalizationCard.tsx`

1. Create card component with title, excerpt, remove button
2. Use `useRemovePersonalization()` for remove action
3. Add click handler to navigate to chapter
4. Handle "Chapter not found" case

**File**: `src/pages/personalization.tsx`

1. Create dedicated page at `/personalization` route
2. Use `usePersonalizedChapters()` to fetch data
3. Render grid of PersonalizationCard components
4. Add empty state when no chapters personalized
5. Add loading skeleton

#### Phase 5: Docusaurus Integration

**Swizzle DocItem Component**:

```bash
# Eject DocItem/Layout component
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

**File**: `src/theme/DocItem/Layout/index.tsx`

1. Import PersonalizationButton component
2. Get current chapter path from Docusaurus context
3. Inject button in doc header (near edit/share buttons)
4. Ensure button is hidden for unauthenticated users

**Register Personalization Page**: `docusaurus.config.js`

```javascript
module.exports = {
  // ... other config
  presets: [
    [
      'classic',
      {
        docs: {
          // ... docs config
        },
      },
    ],
  ],
  plugins: [
    // Add personalization page
    [
      '@docusaurus/plugin-content-pages',
      {
        path: 'src/pages',
        // personalization.tsx will be served at /personalization
      },
    ],
  ],
};
```

### Testing Checklist

**Manual Testing**:

- [ ] **Login**: Log in with test user account
- [ ] **Personalize Chapter**: Click personalization button on a doc page
- [ ] **Visual Indicator**: Verify button shows "personalized" state (filled star)
- [ ] **View List**: Navigate to `/personalization`, see personalized chapter
- [ ] **Card Content**: Verify card shows title, excerpt (50 chars), remove button
- [ ] **Remove**: Click remove button, verify chapter is removed from list
- [ ] **Toggle Off**: Click personalization button again on doc page, verify unpersonalized
- [ ] **Session Expiration**: Let session expire, verify error message on action
- [ ] **Unauthenticated**: Log out, verify no personalization UI visible
- [ ] **Mobile**: Test on mobile viewport (responsive design)
- [ ] **Dark Mode**: Test in dark mode (styling consistent)

**Automated Testing**:

```bash
# Run unit tests
npm run test

# Run integration tests
npm run test:integration

# Run E2E tests (if configured)
npm run test:e2e
```

## Common Issues & Troubleshooting

### Issue: Database connection error

**Symptom**: `Error: connection to database failed`

**Solution**:
1. Check `DATABASE_URL` in `.env.local` is correct
2. Verify Neon Postgres database is accessible
3. Test connection: `npx drizzle-kit introspect:pg`
4. Check firewall/network restrictions

### Issue: Session expired errors

**Symptom**: `401 Unauthorized` or "Session expired" message

**Solution**:
1. Log out and log back in
2. Check Better-Auth server is running
3. Verify `BETTER_AUTH_SECRET` matches between frontend and backend
4. Clear browser cookies and retry

### Issue: Personalization button not showing

**Symptom**: Button not visible on doc pages

**Solution**:
1. Check user is authenticated (`useSession()` returns user)
2. Verify swizzled component is correct path
3. Check CSS z-index (button might be hidden behind other elements)
4. Inspect browser console for React errors

### Issue: API calls failing with CORS errors

**Symptom**: `CORS policy: No 'Access-Control-Allow-Origin' header`

**Solution**:
1. Verify Better-Auth server has CORS configured for frontend origin
2. Check `BETTER_AUTH_URL` matches frontend URL
3. Ensure credentials are included in fetch requests (`credentials: 'include'`)

### Issue: Migrations not applying

**Symptom**: `personalized_chapters` table doesn't exist

**Solution**:
```bash
# Regenerate migration
npx drizzle-kit generate:pg --schema=./src/lib/db/schema.ts

# Force push to database
npx drizzle-kit push:pg --schema=./src/lib/db/schema.ts --force

# Manually apply SQL if needed (from drizzle/migrations/)
```

### Issue: TanStack Query not working

**Symptom**: Data not fetching, mutations not triggering

**Solution**:
1. Verify `QueryClientProvider` wraps app
2. Check React DevTools for Query status
3. Enable Query DevTools: `import { ReactQueryDevtools } from '@tanstack/react-query-devtools'`
4. Check browser Network tab for API calls

## Development Tips

### Hot Reload

- Docusaurus hot reloads on file changes
- Backend server may need restart for API changes
- Clear browser cache if seeing stale data

### Debugging

**Frontend**:
```typescript
// Enable TanStack Query DevTools
import { ReactQueryDevtools } from '@tanstack/react-query-devtools';

// In your app:
<QueryClientProvider client={queryClient}>
  <App />
  <ReactQueryDevtools initialIsOpen={false} />
</QueryClientProvider>
```

**Backend**:
```typescript
// Add logging to API routes
console.log('POST /api/personalization/chapters', { userId, chapterPath });

// Use debugger in VS Code (set breakpoints)
```

**Database**:
```bash
# Monitor database queries
npx drizzle-kit studio
# Opens GUI at http://localhost:4983
```

### Performance Optimization

- Use React DevTools Profiler to find slow renders
- Check Network tab for unnecessary API calls
- Use TanStack Query DevTools to inspect cache

## Next Steps

After completing the setup:

1. **Read Implementation Docs**: Review `research.md`, `data-model.md`, and `contracts/openapi.yaml`
2. **Run Tests**: Ensure all tests pass before making changes
3. **Follow Tasks**: Implement features according to `tasks.md` (generated by `/sp.tasks`)
4. **Code Review**: Submit PR when feature is complete

## Resources

- [TanStack Query Docs](https://tanstack.com/query/latest)
- [Drizzle ORM Docs](https://orm.drizzle.team/docs/overview)
- [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)
- [Better-Auth Documentation](https://www.better-auth.com/docs)
- [OpenAPI Spec](./contracts/openapi.yaml)

## Getting Help

- **Project Issues**: Check GitHub Issues for similar problems
- **Slack/Discord**: Ask in #development channel
- **Documentation**: Refer to `research.md` and `data-model.md` for architecture details
- **Code Examples**: See `contracts/openapi.yaml` for API examples

---

**Last Updated**: 2025-12-17
**Maintainer**: [Your Team Name]
**Version**: 1.0.0
