// auth-server/handlers/personalization.js
import {
  getUserPersonalizedChapters,
  addPersonalizedChapter,
  removePersonalizedChapter,
  isChapterPersonalized
} from '../db/queries.js';

/**
 * Transform database row to API response format
 */
const transformChapter = (chapter) => {
  console.log('[Handler] Raw chapter from DB:', chapter);

  const transformed = {
    id: chapter.id,
    user_id: chapter.userId || chapter.user_id,
    chapter_path: chapter.chapterPath || chapter.chapter_path,
    chapter_title: chapter.chapterTitle || chapter.chapter_title || '',
    chapter_excerpt: chapter.chapterExcerpt || chapter.chapter_excerpt || '',
    created_at: chapter.createdAt || chapter.created_at || new Date().toISOString(),
  };

  console.log('[Handler] Transformed chapter:', transformed);
  return transformed;
};

/**
 * GET /api/personalization/chapters
 * Fetch all personalized chapters for the authenticated user
 */
export const getPersonalizedChapters = async (req, res) => {
  try {
    const user = req.user;
    if (!user) {
      return res.status(401).json({
        error: 'UNAUTHORIZED',
        message: 'You must be logged in to access this resource'
      });
    }

    console.log('[Handler] Fetching chapters for user:', user.id);
    const chapters = await getUserPersonalizedChapters(user.id);
    console.log('[Handler] Fetched chapters count:', chapters.length);

    // Transform chapters to ensure consistent format
    const transformedChapters = chapters.map(transformChapter);

    res.status(200).json({
      chapters: transformedChapters,
      count: transformedChapters.length
    });
  } catch (error) {
    console.error('Error fetching personalized chapters:', error);
    res.status(500).json({
      error: 'INTERNAL_SERVER_ERROR',
      message: 'An unexpected error occurred. Please try again later.'
    });
  }
};

/**
 * POST /api/personalization/chapters
 * Add a chapter to user's personalizations
 */
export const addPersonalized = async (req, res) => {
  try {
    const user = req.user;
    if (!user) {
      return res.status(401).json({
        error: 'UNAUTHORIZED',
        message: 'You must be logged in to access this resource'
      });
    }

    const { chapter_path, chapter_title, chapter_excerpt } = req.body;

    console.log('[Handler] Adding chapter:', {
      chapter_path,
      chapter_title,
      chapter_excerpt,
      user_id: user.id
    });

    // Basic validation
    if (!chapter_path || typeof chapter_path !== 'string') {
      return res.status(400).json({
        error: 'BAD_REQUEST',
        message: 'chapter_path is required'
      });
    }

    // Add the personalized chapter
    const result = await addPersonalizedChapter(
      user.id,
      chapter_path,
      chapter_title || '',
      chapter_excerpt || ''
    );

    console.log('[Handler] Add result:', result);

    if (result.length > 0) {
      // Chapter was added
      const transformed = transformChapter(result[0]);
      res.status(201).json(transformed);
    } else {
      // Chapter already existed (idempotent operation)
      const chapters = await getUserPersonalizedChapters(user.id);
      const chapter = chapters.find(c =>
        (c.chapterPath || c.chapter_path) === chapter_path
      );
      if (chapter) {
        const transformed = transformChapter(chapter);
        res.status(200).json(transformed);
      } else {
        res.status(500).json({
          error: 'INTERNAL_SERVER_ERROR',
          message: 'Chapter was not found after insertion'
        });
      }
    }
  } catch (error) {
    console.error('Error adding personalized chapter:', error);
    res.status(500).json({
      error: 'INTERNAL_SERVER_ERROR',
      message: 'An unexpected error occurred. Please try again later.'
    });
  }
};

/**
 * DELETE /api/personalization/chapters/:chapterPath
 * Remove a chapter from user's personalizations
 */
export const removePersonalized = async (req, res) => {
  try {
    const user = req.user;
    if (!user) {
      return res.status(401).json({
        error: 'UNAUTHORIZED',
        message: 'You must be logged in to access this resource'
      });
    }

    const { chapterPath } = req.params;

    console.log('[Handler] Removing chapter:', {
      chapterPath,
      user_id: user.id
    });

    // URL decode the chapter path
    const decodedChapterPath = decodeURIComponent(chapterPath);

    // Remove the personalized chapter
    await removePersonalizedChapter(user.id, decodedChapterPath);

    console.log('[Handler] Chapter removed successfully');

    // Idempotent operation - return 204 regardless
    res.status(204).send();
  } catch (error) {
    console.error('Error removing personalized chapter:', error);
    res.status(500).json({
      error: 'INTERNAL_SERVER_ERROR',
      message: 'An unexpected error occurred. Please try again later.'
    });
  }
};
