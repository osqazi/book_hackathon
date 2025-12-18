// src/lib/better-auth/client.ts
import { createAuthClient } from 'better-auth/react';

// Determine auth server URL based on environment
// For production (GitHub Pages), use the deployed Vercel URL
// For development (localhost), use local auth server
const getAuthServerURL = () => {
  // Check if we're in production (GitHub Pages)
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    console.log('[AuthClient] Current hostname:', hostname);
    if (hostname === 'osqazi.github.io') {
      const authURL = 'https://book-hackathon-alpha.vercel.app/api/auth';
      console.log('[AuthClient] Using production auth URL:', authURL);
      return authURL;
    }
  }
  // Default to localhost for development
  const authURL = 'http://localhost:3001/api/auth';
  console.log('[AuthClient] Using development auth URL:', authURL);
  return authURL;
};

export const authClient = createAuthClient({
  baseURL: getAuthServerURL(),
  fetchOptions: {
    credentials: 'include',
    headers: {
      'Content-Type': 'application/json',
    },
    mode: 'cors',
  },
  // CRITICAL: Ensure cookies are persisted
  plugins: [],
});