-- schema/auth.sql
-- Database schema extensions for Better-Auth with background profiling fields

-- The main user table is managed by Better-Auth, but we extend it with custom fields
-- These fields are handled by Better-Auth's custom fields functionality
-- However, if we need to manually extend, here's the SQL:

-- Extend user table with background profiling fields
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "software_background" JSON;
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "hardware_background" JSON;
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "background_complete" BOOLEAN DEFAULT FALSE;

-- Create background questions reference table (for UI)
CREATE TABLE IF NOT EXISTS "background_questions" (
  "id" TEXT PRIMARY KEY,
  "category" TEXT NOT NULL CHECK (category IN ('software', 'hardware')),
  "question_text" TEXT NOT NULL,
  "options" TEXT[] NOT NULL,
  "required" BOOLEAN DEFAULT FALSE
);

-- Create indexes for performance
CREATE INDEX IF NOT EXISTS "idx_user_background_complete" ON "user" ("background_complete");
CREATE INDEX IF NOT EXISTS "idx_user_email" ON "user" ("email");