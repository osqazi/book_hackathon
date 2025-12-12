# Quickstart Guide: Better-Auth Signup and Signin

## Overview
This guide provides a quick overview of how to set up and use the Better-Auth authentication system with background profiling for the Humanoid Robotics Book.

## Prerequisites
- Node.js 18+ installed
- Docusaurus project set up
- PostgreSQL (production) or SQLite (development) database

## Setup Steps

### 1. Install Dependencies
```bash
npm install @better-auth/node @better-auth/node/client
npm install drizzle-orm drizzle-kit
npm install postgres # or better-sqlite3 for SQLite
```

### 2. Configure Better-Auth
Create the Better-Auth configuration with custom fields for background profiling:

```typescript
// src/lib/better-auth/auth.ts
import { betterAuth } from "@better-auth/node";
import { drizzleAdapter } from "@better-auth/node/drizzle";

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: "postgresql", // or "sqlite"
  }),
  // Custom fields for background profiling
  user: {
    fields: {
      softwareBackground: "json",
      hardwareBackground: "json",
      backgroundComplete: "boolean",
    },
  },
  // Session configuration (7 days as specified)
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days in seconds
  },
});
```

### 3. Set up Database Schema
Extend Better-Auth's schema with custom fields for background data:

```sql
-- Extend user table with background profiling fields
ALTER TABLE "user" ADD COLUMN software_background JSONB;
ALTER TABLE "user" ADD COLUMN hardware_background JSONB;
ALTER TABLE "user" ADD COLUMN background_complete BOOLEAN DEFAULT FALSE;
```

### 4. Create Authentication Components
Implement signup and signin forms with background profiling:

```tsx
// src/auth/components/SignupForm.tsx
import { useState } from 'react';
import { authClient } from '@/lib/better-auth/client';

export default function SignupForm() {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    softwareBackground: { programmingLanguages: [], frameworks: [], experienceLevel: '' },
    hardwareBackground: { roboticsKits: [], sensors: [], actuators: [], experienceLevel: '' }
  });

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      await authClient.register({
        email: formData.email,
        password: formData.password,
        ...formData.softwareBackground,
        ...formData.hardwareBackground
      });
    } catch (error) {
      console.error('Signup failed:', error);
    }
  };

  return (
    // Form JSX with email, password, and background profiling fields
  );
}
```

### 5. Implement Session Provider
Create a React context to manage authentication state:

```tsx
// src/auth/context/AuthProvider.tsx
import { AuthProvider as BetterAuthProvider } from '@better-auth/node/client/react';

export default function AuthProvider({ children }: { children: React.ReactNode }) {
  return (
    <BetterAuthProvider>
      {children}
    </BetterAuthProvider>
  );
}
```

### 6. Access User Data for Personalization
Use the authenticated user's background data to personalize content:

```tsx
// src/components/personalization/ContentAdapter.tsx
import { useSession } from '@better-auth/node/client/react';

export default function ContentAdapter({ children }: { children: React.ReactNode }) {
  const { session } = useSession();

  if (!session?.user) {
    // Show generic content for unauthenticated users
    return <>{children}</>;
  }

  // Adapt content based on user's background
  const userBackground = session.user.softwareBackground || session.user.hardwareBackground;

  // Return personalized content based on user's expertise
  return <>{/* personalized content */}</>;
}
```

## Testing the Implementation

### 1. Unit Tests
```typescript
// Test authentication flows
describe('Authentication', () => {
  test('should register user with background data', async () => {
    // Test implementation
  });

  test('should sign in existing user', async () => {
    // Test implementation
  });
});
```

### 2. Integration Tests
```typescript
// Test API endpoints
describe('Auth API', () => {
  test('POST /auth/signup should create user with background', async () => {
    // Test implementation
  });

  test('POST /auth/signin should authenticate user', async () => {
    // Test implementation
  });
});
```

## Deployment Notes
- Ensure database connection is configured for production
- Set up HTTPS for secure authentication
- Configure proper session storage for production environment
- Set up monitoring for authentication-related metrics