// auth-server/db/queries.js
import { db } from './client.js';
import { personalizedChapter } from './schema.js';
import { eq, and, desc } from 'drizzle-orm';

/**
 * Fetch all personalized chapters for a user
 */
export async function getUserPersonalizedChapters(userId) {
  return await db
    .select()
    .from(personalizedChapter)
    .where(eq(personalizedChapter.userId, userId))
    .orderBy(desc(personalizedChapter.createdAt));
}

/**
 * Add a personalized chapter (idempotent operation)
 */
export async function addPersonalizedChapter(
  userId,
  chapterPath,
  chapterTitle,
  chapterExcerpt
) {
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

/**
 * Remove a personalized chapter
 */
export async function removePersonalizedChapter(userId, chapterPath) {
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

/**
 * Check if a chapter is personalized by a user
 */
export async function isChapterPersonalized(userId, chapterPath) {
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
