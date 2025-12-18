// // auth-server/server.js
// import express from 'express';
// import cors from 'cors';
// import { toNodeHandler } from 'better-auth/node';
// import { auth } from './auth.js';

// const app = express();
// const PORT = process.env.PORT || 3001;

// // Configure CORS middleware
// app.use(
//   cors({
//     origin: ["http://localhost:3000", "http://localhost:3001"], // Allow Docusaurus origin
//     methods: ["GET", "POST", "PUT", "DELETE"],
//     credentials: true,
//     allowedHeaders: ["Content-Type", "Authorization", "X-Requested-With"],
//     exposedHeaders: ["Set-Cookie"]
//   })
// );

// // Mount Better-Auth routes using the correct handler
// app.all("/api/auth/*", toNodeHandler(auth));

// // Mount express json middleware after Better Auth handler
// app.use(express.json());

// // Health check endpoint
// app.get('/health', (req, res) => {
//   res.json({ status: 'ok', service: 'Auth Server' });
// });

// app.listen(PORT, () => {
//   console.log(`Auth server running on port ${PORT}`);
// });

// auth-server/server.js
import express from 'express';
import cors from 'cors';
import { toNodeHandler } from 'better-auth/node';
import { auth } from './auth.js';

const app = express();

// Use PORT from environment or default to 3001 for local development
const PORT = process.env.PORT || 3001;
const HOST = "0.0.0.0";

// Configure CORS middleware - whitelist all required origins
app.use(
  cors({
    origin: [
      // Development URLs
      "http://localhost:3000",
      "http://localhost:3001",
      // Production URLs
      "https://book-hackathon-alpha.vercel.app",
      "https://osqazi.github.io"
    ],
    methods: ["GET", "POST", "PUT", "DELETE"],
    credentials: true,
    allowedHeaders: ["Content-Type", "Authorization", "X-Requested-With"],
    exposedHeaders: ["Set-Cookie"]
  })
);

// Better Auth routes with error handling
app.all("/api/auth/*", async (req, res, next) => {
  try {
    console.log('[Server] Auth request:', {
      method: req.method,
      url: req.url,
      path: req.path,
      origin: req.get('origin'),
      contentType: req.get('content-type')
    });
    await toNodeHandler(auth)(req, res, next);
  } catch (error) {
    console.error('[Server] Auth handler error:', error);
    console.error('[Server] Error stack:', error.stack);
    res.status(500).json({
      error: 'Internal server error',
      message: error.message,
      stack: process.env.NODE_ENV === 'development' ? error.stack : undefined
    });
  }
});

// Express JSON AFTER auth
app.use(express.json());

// Import personalization routes
import personalizationRouter from './routes/personalization.js';

// Personalization routes
app.use('/api/personalization', personalizationRouter);

// Health check
app.get("/health", (req, res) => {
  res.json({ status: "ok", service: "Auth Server" });
});

// Start server for Hugging Face
app.listen(PORT, HOST, () => {
  console.log(`Auth server running on http://${HOST}:${PORT}`);
});

