# Chapter Personalization Feature

## Overview

The chapter personalization feature allows authenticated users to bookmark documentation chapters for quick access later. Users can personalize chapters by clicking the star icon on any documentation page, and view all their personalized chapters on the `/personalization` page.

## Architecture

### Frontend
- **TanStack Query**: Client-side state management and caching
- **React Hooks**: Custom hooks for API interactions (`usePersonalization.ts`)
- **Docusaurus Integration**: Remark plugin injects personalization buttons into documentation pages
- **UI Components**: PersonalizationButton, PersonalizationCard, PersonalizationEmptyState

### Backend
- **Better-Auth**: Session management and authentication
- **PostgreSQL**: Storage for personalized chapters with Drizzle ORM
- **Express API**: `/api/personalization/*` endpoints
- **Database Schema**: `personalized_chapters` table with indexes

## API Endpoints

### GET `/api/personalization/chapters`
Fetch all personalized chapters for the authenticated user.

**Response**:
```json
{
  "chapters": [
    {
      "id": 1,
      "user_id": "user_abc123",
      "chapter_path": "/docs/intro/getting-started",
      "chapter_title": "Getting Started with Humanoid Robotics",
      "chapter_excerpt": "Learn the basics of humanoid robotics...",
      "created_at": "2025-12-17T10:30:00Z"
    }
  ],
  "count": 1
}
```

### POST `/api/personalization/chapters`
Add a chapter to user's personalizations (idempotent operation).

**Request Body**:
```json
{
  "chapter_path": "/docs/intro/getting-started",
  "chapter_title": "Getting Started with Humanoid Robotics",
  "chapter_excerpt": "Learn the basics of humanoid robotics..."
}
```

**Response**: Returns the personalized chapter object (201 if created, 200 if already existed).

### DELETE `/api/personalization/chapters/{chapterPath}`
Remove a chapter from user's personalizations (idempotent operation).

**Response**: 204 No Content

## Database Schema

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

## Implementation Details

### Frontend Components

1. **PersonalizationButton** (`src/components/PersonalizationButton.tsx`): Toggle button that appears on documentation pages
2. **PersonalizationCard** (`src/components/PersonalizationCard.tsx`): Display component for personalized chapters on the personalization page
3. **PersonalizationEmptyState** (`src/components/PersonalizationEmptyState.tsx`): Message shown when user has no personalized chapters

### React Hooks

1. **usePersonalizedChapters**: Fetch all personalized chapters
2. **useAddPersonalization**: Add a chapter to personalizations
3. **useRemovePersonalization**: Remove a chapter from personalizations
4. **useIsChapterPersonalized**: Check if a specific chapter is personalized

### Docusaurus Integration

The feature integrates with Docusaurus using:
1. A remark plugin that injects the personalization button into documentation pages
2. A Root component wrapper for TanStack Query provider
3. A dedicated page at `/personalization` for viewing all personalized chapters

## Security

- All API endpoints require valid Better-Auth session
- Users can only access/modify their own personalization data
- Input validation using Zod schemas
- CSRF protection via Better-Auth middleware
- Rate limiting on API endpoints

## Performance

- API response times target <200ms p95
- Database indexes on user_id and chapter_path
- TanStack Query caching with automatic invalidation
- Optimistic updates for better UX

## Error Handling

- Session expiration shows error and redirects to login
- Network errors have retry logic (3 attempts)
- Clear error messages for different failure scenarios
- Graceful degradation when API is unavailable

## Styling

- CSS in `src/styles/personalization.css`
- Responsive design for mobile/tablet/desktop
- Dark mode support using Docusaurus CSS variables
- Consistent styling with existing Docusaurus theme

## Testing

The feature includes:
- Manual testing checklist in tasks.md
- API endpoint validation
- Session authentication verification
- Database operation testing
- UI component interaction testing