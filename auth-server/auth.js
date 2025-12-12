// auth-server/auth.js
import { betterAuth } from 'better-auth';
import { Pool } from 'pg';
import dotenv from 'dotenv';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

// Get the directory name of the current module
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Load environment variables from parent directory
dotenv.config({ path: join(__dirname, '..', '.env') });

// Configure base URL and trusted origins based on environment
const baseURL = process.env.AUTH_BASE_URL || 'http://localhost:3001';
const frontendURL = process.env.FRONTEND_URL || 'http://localhost:3000';

// Determine if we're in production - check both NODE_ENV and baseURL
// This ensures cross-origin cookies work correctly even if NODE_ENV isn't set properly
const isProduction = process.env.NODE_ENV === 'production' ||
                     baseURL.includes('vercel.app') ||
                     baseURL.startsWith('https://');

console.log('[Auth Config] Environment check:', {
  NODE_ENV: process.env.NODE_ENV,
  baseURL: baseURL,
  isProduction: isProduction,
  cookieSameSite: isProduction ? 'none' : 'lax'
});

// Define all trusted origins (both dev and production)
const trustedOrigins = [
  // Development URLs
  'http://localhost:3000',
  'http://localhost:3001',
  // Production URLs
  'https://book-hackathon-alpha.vercel.app',
  'https://osqazi.github.io',
];

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.NEON_DATABASE_URL,
  }),
  // Base URL for the auth server (dynamic based on environment)
  baseURL: baseURL,
  // Trusted origins (includes both dev and production)
  trustedOrigins: trustedOrigins,
  // Custom fields for background profiling
  user: {
    additionalFields: {
      softwareBackground: {
        type: 'json',
        required: false,
      },
      hardwareBackground: {
        type: 'json',
        required: false,
      },
      backgroundComplete: {
        type: 'boolean',
        required: false,
        defaultValue: false,
      },
    },
  },
  // Session configuration (7 days as specified)
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days in seconds
  },
  // Advanced security configuration (conditional based on environment)
  advanced: {
    useSecureCookies: isProduction, // Enable secure cookies in production
    disableOriginCheck: !isProduction, // Disable origin check only in development
    crossSubDomainCookies: {
      enabled: false,
    },
    cookieOptions: {
      // Required for cross-origin authentication (GitHub Pages -> Vercel)
      sameSite: isProduction ? 'none' : 'lax',
      secure: isProduction, // Must be true when sameSite is 'none'
    },
  },
  // Other configurations
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // For now, can be enabled later
    password: {
      // Minimum 8 characters as specified
      minLength: 8,
    },
  },
});
