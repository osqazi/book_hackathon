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

// CRITICAL: Force production mode for HTTPS to ensure SameSite=None
const isHTTPS = baseURL.startsWith('https://');
const isLocalhost = baseURL.includes('localhost');

console.log('[Auth Config] Environment detection:', {
  baseURL,
  isHTTPS,
  isLocalhost,
  forcedSameSite: isHTTPS ? 'none' : 'lax'
});

// Define all trusted origins (both dev and production)
const trustedOrigins = [
  'http://localhost:3000',
  'http://localhost:3001',
  'https://book-hackathon-alpha.vercel.app',
  'https://osqazi.github.io',
];

// Cookie configuration - CRITICAL for cross-origin auth
const cookieConfig = {
  name: 'better-auth.session_token',
  options: {
    httpOnly: true,
    secure: isHTTPS,
    sameSite: isHTTPS ? 'none' : 'lax',
    path: '/',
    maxAge: 7 * 24 * 60 * 60, // 7 days
  }
};

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.NEON_DATABASE_URL,
  }),
  baseURL: baseURL,
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
  // Session configuration with cookie options
  session: {
    expiresIn: 7 * 24 * 60 * 60,
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // 5 minutes
    },
  },
  // Advanced security configuration
  advanced: {
    useSecureCookies: isHTTPS,
    crossSubDomainCookies: {
      enabled: false,
    },
    cookiePrefix: isHTTPS ? '__Secure-' : '',
  },
  // Email/password auth
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    password: {
      minLength: 8,
    },
  },
  // CRITICAL: Set cookie options at root level
  cookies: {
    session_token: {
      name: isHTTPS ? '__Secure-better-auth.session_token' : 'better-auth.session_token',
      attributes: {
        sameSite: isHTTPS ? 'none' : 'lax',
        secure: isHTTPS,
        httpOnly: true,
        path: '/',
      },
    },
  },
});

console.log('[Auth Config] Cookie configuration:', {
  sameSite: isHTTPS ? 'none' : 'lax',
  secure: isHTTPS,
  prefix: isHTTPS ? '__Secure-' : ''
});
