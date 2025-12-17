import express from 'express';
import { getPersonalizedChapters, addPersonalizedChapter, removePersonalizedChapter } from './handlers';
import { withAuth } from 'better-auth/integrations/express';

const router = express.Router();

// GET /api/personalization/chapters - Fetch all personalized chapters for user
router.get('/chapters', withAuth(), getPersonalizedChapters);

// POST /api/personalization/chapters - Add a chapter to personalizations
router.post('/chapters', withAuth(), addPersonalizedChapter);

// DELETE /api/personalization/chapters/:chapterPath - Remove a chapter from personalizations
router.delete('/chapters/:chapterPath', withAuth(), removePersonalizedChapter);

export default router;