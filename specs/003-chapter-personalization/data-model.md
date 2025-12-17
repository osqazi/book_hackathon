# Data Model: Per-User Chapter Personalization

**Feature**: 003-chapter-personalization
**Date**: 2025-12-17
**Database**: Neon Postgres
**ORM**: Drizzle ORM

## Entity Relationship Diagram

```
┌─────────────────┐         ┌──────────────────────────┐
│     User        │         │  PersonalizedChapter     │
│  (Better-Auth)  │         │                          │
├─────────────────┤         ├──────────────────────────┤
│ id (PK)         │◄────────│ id (PK)                  │
│ email           │ 1     * │ user_id (FK)             │
│ name            │         │ chapter_path             │
│ ...             │         │ chapter_title            │
└─────────────────┘         │ chapter_excerpt          │
                            │ created_at               │
                            │                          │
                            │ UNIQUE(user_id,          │
                            │        chapter_path)     │
                            └──────────────────────────┘
```

## Entities

### PersonalizedChapter

**Purpose**: Stores the mapping between users and their personalized (bookmarked) documentation chapters.

**Table Name**: `personalized_chapters`

#### Fields

| Field Name        | Type                      | Constraints                          | Description                                                                 |
|-------------------|---------------------------|--------------------------------------|-----------------------------------------------------------------------------|
| `id`              | SERIAL (INT)              | PRIMARY KEY                          | Auto-incrementing unique identifier for each personalization record         |
| `user_id`         | TEXT                      | NOT NULL, FOREIGN KEY → user(id)     | Reference to Better-Auth user table; identifies which user personalized     |
| `chapter_path`    | TEXT                      | NOT NULL                             | Full URL path of the chapter (e.g., "/docs/intro/getting-started")         |
| `chapter_title`   | TEXT                      | NULL                                 | Title of the chapter at time of personalization (for display in cards)     |
| `chapter_excerpt` | TEXT                      | NULL                                 | First 50 characters of chapter content (for preview in cards)               |
| `created_at`      | TIMESTAMP WITH TIME ZONE  | DEFAULT NOW()                        | Timestamp when the chapter was personalized by the user                     |

#### Constraints

1. **Primary Key**: `id` - Unique identifier for each record
2. **Foreign Key**: `user_id` REFERENCES `user(id)` ON DELETE CASCADE
   - Ensures referential integrity with Better-Auth user table
   - Cascade delete: When user is deleted, all their personalizations are removed
3. **Unique Constraint**: `UNIQUE(user_id, chapter_path)`
   - Prevents duplicate personalizations of the same chapter by the same user
   - Enables idempotent operations (multiple add requests for same chapter)
4. **Not Null**: `user_id`, `chapter_path` - Required fields for valid personalization

#### Indexes

```sql
-- Primary index (automatic on PRIMARY KEY)
CREATE INDEX personalized_chapters_pkey ON personalized_chapters(id);

-- Foreign key index for user lookups (fetch all chapters for a user)
CREATE INDEX idx_personalized_chapters_user_id ON personalized_chapters(user_id);

-- Index for chapter path lookups (check if specific chapter is personalized)
CREATE INDEX idx_personalized_chapters_path ON personalized_chapters(chapter_path);

-- Composite index for unique constraint (automatic on UNIQUE)
CREATE UNIQUE INDEX idx_personalized_chapters_user_chapter ON personalized_chapters(user_id, chapter_path);
```

**Index Rationale**:
- `user_id` index: Optimizes `GET /api/personalization/chapters` (fetch all for user)
- `chapter_path` index: Optimizes checking if current page is personalized
- Composite unique index: Prevents duplicates, optimizes add/remove operations

#### Validation Rules

**Server-Side Validation (Zod Schema)**:

```typescript
import { z } from 'zod';

export const personalizedChapterSchema = z.object({
  chapter_path: z.string()
    .min(1, 'Chapter path is required')
    .max(500, 'Chapter path too long')
    .regex(/^\/[a-zA-Z0-9/_-]+$/, 'Invalid chapter path format'),

  chapter_title: z.string()
    .max(500, 'Chapter title too long')
    .optional()
    .nullable(),

  chapter_excerpt: z.string()
    .max(50, 'Chapter excerpt must be 50 characters or less')
    .optional()
    .nullable(),
});

export type PersonalizedChapterInput = z.infer<typeof personalizedChapterSchema>;
```

**Validation Rules**:
1. `chapter_path`: Must start with `/`, contain only alphanumeric, `/`, `_`, `-` characters
2. `chapter_title`: Optional, max 500 characters (Docusaurus title length limit)
3. `chapter_excerpt`: Optional, max 50 characters (as specified in clarifications)
4. All text fields: Sanitized for XSS before storage

#### State Transitions

**Lifecycle**: A personalized chapter has two states:
1. **Exists**: Chapter is personalized (record exists in database)
2. **Deleted**: Chapter is not personalized (record does not exist)

**Transitions**:
- `null` → `Exists`: User clicks personalization button on unpersonalized chapter
- `Exists` → `Exists`: Duplicate add request (idempotent, no change)
- `Exists` → `Deleted`: User clicks personalization button again (toggle off) OR removes from /personalization page
- `Deleted` → `Deleted`: Duplicate remove request (idempotent, no error)

**No Soft Deletes**: Records are permanently deleted, not marked as inactive.

### User (Existing Entity)

**Purpose**: Stores user authentication data (managed by Better-Auth).

**Table Name**: `user` (Better-Auth default)

**Relevant Fields for This Feature**:
- `id` (TEXT, PRIMARY KEY): User unique identifier
- `email` (TEXT): User email (for display in admin/debugging)
- `name` (TEXT): User display name (optional)

**Note**: We do NOT modify this table. It's managed entirely by Better-Auth.

## Drizzle ORM Schema

**File**: `src/lib/db/schema.ts` (or similar location in project)

```typescript
import { pgTable, serial, text, timestamp, uniqueIndex, index } from 'drizzle-orm/pg-core';
import { relations } from 'drizzle-orm';

// Reference to existing Better-Auth user table
export const user = pgTable('user', {
  id: text('id').primaryKey(),
  email: text('email').notNull(),
  name: text('name'),
  // ... other Better-Auth fields
});

// New table for personalized chapters
export const personalizedChapter = pgTable(
  'personalized_chapters',
  {
    id: serial('id').primaryKey(),
    userId: text('user_id')
      .notNull()
      .references(() => user.id, { onDelete: 'cascade' }),
    chapterPath: text('chapter_path').notNull(),
    chapterTitle: text('chapter_title'),
    chapterExcerpt: text('chapter_excerpt'),
    createdAt: timestamp('created_at', { withTimezone: true }).defaultNow().notNull(),
  },
  (table) => ({
    // Unique constraint: one user can't personalize same chapter twice
    userChapterIdx: uniqueIndex('idx_personalized_chapters_user_chapter').on(
      table.userId,
      table.chapterPath
    ),
    // Index for fetching all chapters for a user
    userIdIdx: index('idx_personalized_chapters_user_id').on(table.userId),
    // Index for checking if specific chapter is personalized
    chapterPathIdx: index('idx_personalized_chapters_path').on(table.chapterPath),
  })
);

// Relations (for Drizzle's relational query API)
export const userRelations = relations(user, ({ many }) => ({
  personalizedChapters: many(personalizedChapter),
}));

export const personalizedChapterRelations = relations(personalizedChapter, ({ one }) => ({
  user: one(user, {
    fields: [personalizedChapter.userId],
    references: [user.id],
  }),
}));

// TypeScript types
export type PersonalizedChapter = typeof personalizedChapter.$inferSelect;
export type NewPersonalizedChapter = typeof personalizedChapter.$inferInsert;
```

## Database Migration

**Migration File**: `drizzle/migrations/0001_add_personalized_chapters.sql`

```sql
-- Migration: Add personalized_chapters table
-- Generated by: drizzle-kit generate:pg
-- Date: 2025-12-17

CREATE TABLE IF NOT EXISTS "personalized_chapters" (
  "id" SERIAL PRIMARY KEY,
  "user_id" TEXT NOT NULL,
  "chapter_path" TEXT NOT NULL,
  "chapter_title" TEXT,
  "chapter_excerpt" TEXT,
  "created_at" TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,
  CONSTRAINT "fk_personalized_chapters_user"
    FOREIGN KEY ("user_id")
    REFERENCES "user"("id")
    ON DELETE CASCADE
);

-- Unique constraint: prevent duplicate personalizations
CREATE UNIQUE INDEX IF NOT EXISTS "idx_personalized_chapters_user_chapter"
  ON "personalized_chapters" ("user_id", "chapter_path");

-- Index for user lookups
CREATE INDEX IF NOT EXISTS "idx_personalized_chapters_user_id"
  ON "personalized_chapters" ("user_id");

-- Index for chapter path lookups
CREATE INDEX IF NOT EXISTS "idx_personalized_chapters_path"
  ON "personalized_chapters" ("chapter_path");
```

**Rollback Migration**:

```sql
-- Rollback: Remove personalized_chapters table
DROP INDEX IF EXISTS "idx_personalized_chapters_path";
DROP INDEX IF EXISTS "idx_personalized_chapters_user_id";
DROP INDEX IF EXISTS "idx_personalized_chapters_user_chapter";
DROP TABLE IF EXISTS "personalized_chapters";
```

## Query Examples

### Fetch All Personalized Chapters for User

```typescript
import { db } from '@/lib/db';
import { personalizedChapter } from '@/lib/db/schema';
import { eq } from 'drizzle-orm';

async function getUserPersonalizedChapters(userId: string) {
  return await db
    .select()
    .from(personalizedChapter)
    .where(eq(personalizedChapter.userId, userId))
    .orderBy(personalizedChapter.createdAt.desc());
}
```

### Add Personalized Chapter (Idempotent)

```typescript
import { db } from '@/lib/db';
import { personalizedChapter } from '@/lib/db/schema';

async function addPersonalizedChapter(
  userId: string,
  chapterPath: string,
  chapterTitle?: string,
  chapterExcerpt?: string
) {
  // Using INSERT ... ON CONFLICT for idempotency
  return await db
    .insert(personalizedChapter)
    .values({
      userId,
      chapterPath,
      chapterTitle,
      chapterExcerpt,
    })
    .onConflictDoNothing({
      target: [personalizedChapter.userId, personalizedChapter.chapterPath],
    })
    .returning();
}
```

### Remove Personalized Chapter

```typescript
import { db } from '@/lib/db';
import { personalizedChapter } from '@/lib/db/schema';
import { and, eq } from 'drizzle-orm';

async function removePersonalizedChapter(userId: string, chapterPath: string) {
  return await db
    .delete(personalizedChapter)
    .where(
      and(
        eq(personalizedChapter.userId, userId),
        eq(personalizedChapter.chapterPath, chapterPath)
      )
    )
    .returning();
}
```

### Check if Chapter is Personalized

```typescript
import { db } from '@/lib/db';
import { personalizedChapter } from '@/lib/db/schema';
import { and, eq } from 'drizzle-orm';

async function isChapterPersonalized(userId: string, chapterPath: string): Promise<boolean> {
  const result = await db
    .select({ id: personalizedChapter.id })
    .from(personalizedChapter)
    .where(
      and(
        eq(personalizedChapter.userId, userId),
        eq(personalizedChapter.chapterPath, chapterPath)
      )
    )
    .limit(1);

  return result.length > 0;
}
```

## Data Access Patterns

### Read Operations (Queries)

1. **Get All User's Personalized Chapters**
   - **Frequency**: Every page load of `/personalization` (once per session with caching)
   - **Performance**: O(1) with user_id index, ~50ms for 100 chapters
   - **Caching**: TanStack Query caches for 5 minutes

2. **Check if Current Chapter is Personalized**
   - **Frequency**: Every doc page load (multiple per session)
   - **Performance**: O(1) with composite index, ~20ms
   - **Caching**: TanStack Query caches for 1 minute, invalidated on mutation

### Write Operations (Mutations)

1. **Add Personalized Chapter**
   - **Frequency**: User-initiated, ~1-5 times per session
   - **Performance**: O(1) with unique index, ~80ms
   - **Idempotency**: ON CONFLICT DO NOTHING prevents duplicates

2. **Remove Personalized Chapter**
   - **Frequency**: User-initiated, ~1-2 times per session
   - **Performance**: O(1) with composite index, ~60ms
   - **Idempotency**: DELETE WHERE (no error if already deleted)

## Scalability Considerations

### Storage Growth

**Per User**: Average 20 personalized chapters × 200 bytes = 4KB per user

**System Scale**:
- 10,000 users: 40MB
- 100,000 users: 400MB
- 1,000,000 users: 4GB

**Conclusion**: Storage is negligible, no partitioning needed.

### Query Performance

**Current Design** (No Optimization Needed):
- Indexes keep all queries under 100ms
- Database can handle 1000s of requests/second
- Connection pooling prevents connection exhaustion

**Future Optimization** (If Needed):
- Limit personalizations per user (e.g., max 100 chapters)
- Add pagination to GET endpoint if users exceed 100 chapters
- Consider materialized view if complex aggregations needed

### Concurrency

**Race Condition Prevention**:
- UNIQUE constraint prevents duplicate inserts from concurrent requests
- PostgreSQL serializable transactions ensure consistency
- Idempotent operations mean duplicate requests are safe

**Example Concurrent Scenario**:
- User double-clicks personalization button (2 simultaneous requests)
- Both requests try to INSERT with same (user_id, chapter_path)
- First succeeds, second hits unique constraint and returns (no error)
- Result: Only one record created (correct behavior)

## Security & Privacy

### Authorization

**Rule**: Users can only access/modify their own personalized chapters.

**Enforcement**:
1. User ID extracted from Better-Auth session (server-side)
2. User ID NOT accepted from client request body
3. All queries filtered by `userId = session.user.id`
4. No endpoints to access other users' personalizations

### Data Retention

**Policy**: Personalized chapters are deleted when:
1. User manually removes them (via UI)
2. User account is deleted (cascade delete via FK constraint)

**No Automatic Cleanup**: We keep personalizations even if chapters are deleted/renamed (per spec requirement FR-011).

### Personally Identifiable Information (PII)

**Data Classification**:
- `user_id`: Linked to Better-Auth user table (PII by association)
- `chapter_path`: Public information (doc URLs are not sensitive)
- `chapter_title`, `chapter_excerpt`: Public information (doc content is public)

**Conclusion**: Personalization data itself is low-risk, but table contains user association.

## Testing Data

### Seed Data (Development/Testing)

```typescript
// seed-personalization.ts
import { db } from '@/lib/db';
import { personalizedChapter } from '@/lib/db/schema';

const testUserId = 'test-user-123'; // Replace with actual test user ID

const seedChapters = [
  {
    userId: testUserId,
    chapterPath: '/docs/intro/getting-started',
    chapterTitle: 'Getting Started with Humanoid Robotics',
    chapterExcerpt: 'Learn the basics of humanoid robotics and how...',
  },
  {
    userId: testUserId,
    chapterPath: '/docs/basics/ros-fundamentals',
    chapterTitle: 'ROS 2 Fundamentals',
    chapterExcerpt: 'Understand the core concepts of ROS 2 for robo...',
  },
  {
    userId: testUserId,
    chapterPath: '/docs/advanced/motion-planning',
    chapterTitle: 'Advanced Motion Planning',
    chapterExcerpt: 'Explore sophisticated motion planning algorithm...',
  },
];

async function seed() {
  for (const chapter of seedChapters) {
    await db.insert(personalizedChapter).values(chapter).onConflictDoNothing();
  }
  console.log('✅ Seed data inserted');
}

seed();
```

## Change Log

| Date       | Change Description                                  | Migration Version |
|------------|-----------------------------------------------------|-------------------|
| 2025-12-17 | Initial schema design for personalized_chapters    | 0001              |

## Next Steps

1. ✅ Define database schema and migrations
2. Generate API contracts (OpenAPI spec)
3. Implement Drizzle schema in codebase
4. Create and run database migration
5. Implement data access layer (queries/mutations)
6. Write unit tests for database operations
