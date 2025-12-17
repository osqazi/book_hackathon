// auth-server/routes/personalization.js
import express from 'express';
import { auth } from '../auth.js';
import {
  getPersonalizedChapters,
  addPersonalized,
  removePersonalized
} from '../handlers/personalization.js';

const router = express.Router();

// Middleware to get user from Better-Auth session
const authMiddleware = async (req, res, next) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers
    });

    if (session?.user) {
      req.user = session.user;
      next();
    } else {
      res.status(401).json({
        error: 'UNAUTHORIZED',
        message: 'You must be logged in to access this resource'
      });
    }
  } catch (error) {
    console.error('Auth middleware error:', error);
    res.status(401).json({
      error: 'UNAUTHORIZED',
      message: 'Invalid session'
    });
  }
};

// Apply auth middleware to all routes
router.use(authMiddleware);

// Routes
router.get('/chapters', getPersonalizedChapters);
router.post('/chapters', addPersonalized);
router.delete('/chapters/:chapterPath', removePersonalized);

export default router;
