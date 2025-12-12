# Data Model: Better-Auth Signup and Signin

## User Profile Entity

**UserProfile**
- id: string (primary key, auto-generated)
- email: string (unique, required, valid email format)
- password: string (hashed, required, minimum 8 characters)
- createdAt: datetime (auto-generated on creation)
- updatedAt: datetime (auto-generated on update)
- lastSignInAt: datetime (updated on each sign in)
- softwareBackground: JSON object (programming languages, frameworks, experience level)
- hardwareBackground: JSON object (robotics kits, sensors, actuators, experience level)
- isBackgroundComplete: boolean (indicates if background profiling is complete)

**Validation Rules**:
- Email must follow valid email format
- Password must be minimum 8 characters
- Email must be unique across all users
- SoftwareBackground and HardwareBackground must follow predefined schema for multiple choice options

## Background Question Entity

**BackgroundQuestion** (Reference data, not stored per user)
- id: string (primary key)
- category: string (enum: "software", "hardware")
- questionText: string (required)
- options: array of strings (for multiple choice format)
- required: boolean (whether this question is required)

## Session Entity

**Session** (Managed by Better-Auth)
- id: string (primary key)
- userId: string (foreign key to UserProfile)
- expiresAt: datetime (7 days from creation as specified in feature spec)
- createdAt: datetime
- updatedAt: datetime

## Database Schema Extensions

### Users Table Extensions (via Better-Auth custom fields)
```sql
-- Extending Better-Auth's user table with custom fields
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "software_background" JSONB;
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "hardware_background" JSONB;
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "background_complete" BOOLEAN DEFAULT FALSE;
```

### Background Questions Reference Table (for UI)
```sql
CREATE TABLE IF NOT EXISTS "background_questions" (
  "id" TEXT PRIMARY KEY,
  "category" TEXT NOT NULL,
  "question_text" TEXT NOT NULL,
  "options" TEXT[] NOT NULL,
  "required" BOOLEAN DEFAULT FALSE
);
```

## Relationships
- UserProfile has many Sessions (one-to-many)
- BackgroundQuestions are reference data used for form generation
- Session belongs to UserProfile (many-to-one)

## State Transitions
- UserProfile:
  - Created during signup (background_complete = false)
  - Updated when background profiling is completed (background_complete = true)
  - Updated on each sign in (lastSignInAt)

## Indexes
- Index on email field for fast lookup
- Index on session expiresAt for cleanup
- Index on background_complete for personalization queries