// src/lib/better-auth/client.ts
import { createAuthClient } from 'better-auth/react';

// Determine auth server URL based on environment
// For production (GitHub Pages), use the deployed Vercel URL
// For development (localhost), use local auth server
const getAuthServerURL = () => {
  // Check if we're in production (GitHub Pages)
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    if (hostname === 'osqazi.github.io') {
      return 'https://book-hackathon-alpha.vercel.app';
    }
  }
  // Default to localhost for development
  return 'http://localhost:3001';
};

export const authClient = createAuthClient({
  baseURL: getAuthServerURL(),
  fetchOptions: {
    credentials: 'include',
  },
});