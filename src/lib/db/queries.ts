import { db } from './client';
import { personalizedChapter } from './schema';
import { eq, and } from 'drizzle-orm';
import type { NewPersonalizedChapter } from './schema';

/**
 * Fetch all personalized chapters for a user
 */
export async function getUserPersonalizedChapters(userId: string) {
  return await db
    .select()
    .from(personalizedChapter)
    .where(eq(personalizedChapter.userId, userId))
    .orderBy(personalizedChapter.createdAt.desc());
}

/**
 * Add a personalized chapter (idempotent operation)
 */
export async function addPersonalizedChapter(
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

/**
 * Remove a personalized chapter
 */
export async function removePersonalizedChapter(userId: string, chapterPath: string) {
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
export async function isChapterPersonalized(userId: string, chapterPath: string): Promise<boolean> {
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