import express from 'express';
import { auth } from './better-auth/auth';
import { server as betterAuthExpress } from 'better-auth/express';
import personalizationRouter from '../api/personalization/routes';
import cors from 'cors';

const app = express();

// Enable CORS for all routes
app.use(cors({
  origin: process.env.NODE_ENV === 'production'
    ? ['https://osqazi.github.io', 'https://book-hackathon-alpha.vercel.app']
    : ['http://localhost:3000', 'http://localhost:3001'],
  credentials: true,
}));

// Parse JSON bodies
app.use(express.json());

// Use Better-Auth middleware
app.use(
  '/api/auth',
  betterAuthExpress(auth)
);

// Use personalization routes
app.use('/api/personalization', personalizationRouter);

// Health check endpoint
app.get('/health', (req, res) => {
  res.status(200).json({ status: 'OK', timestamp: new Date().toISOString() });
});

// Error handling middleware
app.use((err: any, req: express.Request, res: express.Response, next: express.NextFunction) => {
  console.error('Unhandled error:', err);
  res.status(500).json({
    error: 'INTERNAL_SERVER_ERROR',
    message: 'An unexpected error occurred'
  });
});

// 404 handler
app.use('*', (req, res) => {
  res.status(404).json({
    error: 'NOT_FOUND',
    message: 'Endpoint not found'
  });
});

const PORT = process.env.PORT || 3001;

app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});

export default app;