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

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.NEON_DATABASE_URL,
  }),
  // Base URL for the auth server
  baseURL: 'http://localhost:3001',
  // Trusted origins
  trustedOrigins: ['http://localhost:3000', 'http://localhost:3001'],
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
  // Advanced security configuration
  advanced: {
    useSecureCookies: false, // Disable for localhost development
    disableOriginCheck: true, // Disable origin validation for development
    crossSubDomainCookies: {
      enabled: false,
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
