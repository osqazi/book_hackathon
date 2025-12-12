// src/lib/better-auth/client.ts
import { createAuthClient } from 'better-auth/react';

export const authClient = createAuthClient({
  baseURL: 'http://localhost:3001',
  fetchOptions: {
    credentials: 'include',
  },
});