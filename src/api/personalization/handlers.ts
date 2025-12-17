import type { Request, Response } from 'express';
import { getUserPersonalizedChapters, addPersonalizedChapter, removePersonalizedChapter, isChapterPersonalized } from '../../lib/db/queries';
import { personalizedChapterSchema } from './validation';
import { withAuth } from 'better-auth/integrations/express';

/**
 * GET /api/personalization/chapters
 * Fetch all personalized chapters for the authenticated user
 */
export const getPersonalizedChapters = async (req: Request, res: Response) => {
  try {
    const { user } = req.auth;
    if (!user) {
      return res.status(401).json({
        error: 'UNAUTHORIZED',
        message: 'You must be logged in to access this resource'
      });
    }

    const chapters = await getUserPersonalizedChapters(user.id);

    res.status(200).json({
      chapters,
      count: chapters.length
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
export const addPersonalizedChapter = async (req: Request, res: Response) => {
  try {
    const { user } = req.auth;
    if (!user) {
      return res.status(401).json({
        error: 'UNAUTHORIZED',
        message: 'You must be logged in to access this resource'
      });
    }

    // Validate request body
    const validationResult = personalizedChapterSchema.safeParse(req.body);
    if (!validationResult.success) {
      return res.status(422).json({
        error: 'VALIDATION_ERROR',
        message: 'Request validation failed',
        validation_errors: validationResult.error.errors.map(err => ({
          field: err.path.join('.'),
          message: err.message
        }))
      });
    }

    const { chapter_path, chapter_title, chapter_excerpt } = validationResult.data;

    // Add the personalized chapter
    const result = await addPersonalizedChapter(
      user.id,
      chapter_path,
      chapter_title,
      chapter_excerpt
    );

    if (result.length > 0) {
      // Chapter was added
      res.status(201).json(result[0]);
    } else {
      // Chapter already existed (idempotent operation)
      const existingChapter = await isChapterPersonalized(user.id, chapter_path);
      if (existingChapter) {
        // Fetch the existing record to return
        const chapters = await getUserPersonalizedChapters(user.id);
        const chapter = chapters.find(c => c.chapterPath === chapter_path);
        if (chapter) {
          res.status(200).json(chapter);
        } else {
          res.status(500).json({
            error: 'INTERNAL_SERVER_ERROR',
            message: 'Chapter was not found after insertion'
          });
        }
      } else {
        res.status(500).json({
          error: 'INTERNAL_SERVER_ERROR',
          message: 'Chapter was not added to the database'
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
export const removePersonalizedChapter = async (req: Request, res: Response) => {
  try {
    const { user } = req.auth;
    if (!user) {
      return res.status(401).json({
        error: 'UNAUTHORIZED',
        message: 'You must be logged in to access this resource'
      });
    }

    const { chapterPath } = req.params;

    // Validate chapter path format
    if (!chapterPath || !/^\/[a-zA-Z0-9/_-]+$/.test(decodeURIComponent(chapterPath))) {
      return res.status(400).json({
        error: 'BAD_REQUEST',
        message: 'Invalid chapter path format'
      });
    }

    // URL decode the chapter path since it might be encoded
    const decodedChapterPath = decodeURIComponent(chapterPath);

    // Remove the personalized chapter
    const result = await removePersonalizedChapter(user.id, decodedChapterPath);

    // Idempotent operation - return 204 regardless of whether the chapter existed
    res.status(204).send();
  } catch (error) {
    console.error('Error removing personalized chapter:', error);
    res.status(500).json({
      error: 'INTERNAL_SERVER_ERROR',
      message: 'An unexpected error occurred. Please try again later.'
    });
  }
};