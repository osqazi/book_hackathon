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

// CRITICAL: Detect production based on HTTPS
const isHTTPS = baseURL.startsWith('https://');

console.log('[Auth Config] Environment:', {
  baseURL,
  isHTTPS,
  sameSite: isHTTPS ? 'none' : 'lax',
  secure: isHTTPS
});

// Define all trusted origins (both dev and production)
const trustedOrigins = [
  'http://localhost:3000',
  'http://localhost:3001',
  'https://book-hackathon-alpha.vercel.app',
  'https://osqazi.github.io',
];

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.NEON_DATABASE_URL,
  }),
  baseURL: baseURL,
  trustedOrigins: trustedOrigins,
  // CRITICAL: Explicitly set the secret from environment
  secret: process.env.BETTER_AUTH_SECRET,
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
  // Session configuration
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
    // CRITICAL: Update session tokens on every request to keep them fresh
    updateAge: 24 * 60 * 60, // Update session every 24 hours
  },
  // Advanced configuration
  advanced: {
    // CRITICAL: Set default cookie attributes for ALL cookies
    // This is the correct API for Better Auth
    defaultCookieAttributes: {
      sameSite: isHTTPS ? 'none' : 'lax',
      secure: isHTTPS,
      httpOnly: true,
      path: '/',
      // partitioned: true, // New browser standards for cross-site cookies
    },
    // Force secure cookies for HTTPS
    useSecureCookies: isHTTPS,
    // Cross-subdomain cookies disabled (we're cross-origin, not subdomain)
    crossSubDomainCookies: {
      enabled: false,
    },
  },
  // Email/password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    password: {
      minLength: 8,
    },
  },
});

console.log('[Auth Config] Cookies will use:', {
  sameSite: isHTTPS ? 'none' : 'lax',
  secure: isHTTPS,
  httpOnly: true,
  partitioned: true
});
