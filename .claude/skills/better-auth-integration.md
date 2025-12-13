# Skill: Better-Auth Integration for Docusaurus

**Created**: 2025-12-13
**Context**: This skill encapsulates the knowledge and best practices for integrating Better-Auth authentication into a Docusaurus-based application, based on the successful implementation in the Humanoid Robotics Book project.

## Overview

Better-Auth is a TypeScript-first authentication framework that provides comprehensive authentication features. This skill covers the complete integration pattern for Docusaurus projects, including:
- Separate authentication server architecture
- Custom user profile fields for personalization
- Cross-origin authentication configuration
- Session management and cookie handling
- Production deployment considerations

## Architecture Pattern

### Dual-Server Architecture
The integration uses a **separate authentication server** pattern:

1. **Auth Server** (Node.js/Express on port 3001/7860)
   - Handles all authentication operations
   - Manages database connections
   - Serves Better-Auth API endpoints at `/api/auth/*`
   - Runs independently from the Docusaurus site

2. **Docusaurus Client** (React/Docusaurus on port 3000)
   - Consumes authentication via Better-Auth React client
   - Manages client-side session state
   - Renders authentication UI components
   - Personalizes content based on user data

### Database Configuration
- **Development**: SQLite (better-sqlite3) for local testing
- **Production**: PostgreSQL (via Neon/pg Pool) for scalability
- **ORM**: Drizzle adapter for Better-Auth schema management

## File Structure

```
project-root/
├── auth-server/
│   ├── server.js          # Express server with CORS and Better-Auth handler
│   ├── auth.js            # Better-Auth configuration with custom fields
│   ├── package.json       # Server dependencies
│   └── .env               # Server environment variables (don't commit)
├── src/
│   ├── lib/
│   │   └── better-auth/
│   │       ├── auth.ts    # Better-Auth config (SQLite for dev)
│   │       └── client.ts  # Better-Auth React client with env detection
│   ├── auth/
│   │   ├── components/
│   │   │   ├── SignupForm.tsx        # Registration form
│   │   │   ├── SigninForm.tsx        # Login form
│   │   │   └── BackgroundQuestions.tsx  # Custom profile fields UI
│   │   ├── hooks/
│   │   │   └── useAuth.ts            # Authentication state hook
│   │   ├── context/
│   │   │   └── AuthProvider.tsx      # Global auth context
│   │   ├── services/
│   │   │   └── auth-service.ts       # API service layer
│   │   └── utils/
│   │       └── session-utils.ts      # Session helpers
│   ├── components/
│   │   └── AuthNavbar.tsx            # Authenticated user navbar
│   ├── pages/
│   │   ├── signin.tsx                # Sign-in page
│   │   └── signup.tsx                # Sign-up page
│   └── css/
│       └── auth-pages.css            # Auth UI styling
├── plugins/
│   └── docusaurus-auth/
│       └── index.js                  # Docusaurus plugin integration
└── .env                              # Root environment variables
```

## Core Implementation

### 1. Auth Server Configuration (auth-server/auth.js)

```javascript
import { betterAuth } from 'better-auth';
import { Pool } from 'pg';

const isHTTPS = baseURL.startsWith('https://');

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.NEON_DATABASE_URL,
  }),
  baseURL: process.env.AUTH_BASE_URL || 'http://localhost:3001',
  trustedOrigins: [
    'http://localhost:3000',
    'http://localhost:3001',
    'https://your-production-domain.com',
    'https://your-github-pages.io',
  ],
  user: {
    additionalFields: {
      softwareBackground: { type: 'json', required: false },
      hardwareBackground: { type: 'json', required: false },
      backgroundComplete: { type: 'boolean', required: false, defaultValue: false },
    },
  },
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days in seconds
  },
  advanced: {
    // CRITICAL: Configure cookies for cross-origin authentication
    defaultCookieAttributes: {
      sameSite: isHTTPS ? 'none' : 'lax',
      secure: isHTTPS,
      httpOnly: true,
      partitioned: true, // New standard for cross-site cookies
    },
    useSecureCookies: isHTTPS,
    crossSubDomainCookies: {
      enabled: false, // Use cross-origin, not subdomain
    },
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Can be enabled later
    password: {
      minLength: 8,
    },
  },
});
```

**Key Points**:
- Use `additionalFields` (not `fields`) for custom user data
- Set `trustedOrigins` to all allowed frontend URLs
- Configure cookies based on HTTPS detection
- Use `defaultCookieAttributes` in `advanced` config (correct Better-Auth API)
- Enable `partitioned` cookies for modern browser standards

### 2. Express Server Setup (auth-server/server.js)

```javascript
import express from 'express';
import cors from 'cors';
import { toNodeHandler } from 'better-auth/node';
import { auth } from './auth.js';

const app = express();
const PORT = process.env.PORT || 7860; // Hugging Face/Vercel default
const HOST = "0.0.0.0";

// CORS configuration - whitelist all required origins
app.use(
  cors({
    origin: [
      "http://localhost:3000",
      "http://localhost:3001",
      "https://your-production-url.com",
      "https://your-github-pages.io"
    ],
    methods: ["GET", "POST", "PUT", "DELETE"],
    credentials: true,
    allowedHeaders: ["Content-Type", "Authorization", "X-Requested-With"],
    exposedHeaders: ["Set-Cookie"]
  })
);

// Better-Auth routes MUST come before other middleware
app.all("/api/auth/*", toNodeHandler(auth));

// Other middleware AFTER auth handler
app.use(express.json());

// Health check endpoint
app.get("/health", (req, res) => {
  res.json({ status: "ok", service: "Auth Server" });
});

app.listen(PORT, HOST, () => {
  console.log(`Auth server running on http://${HOST}:${PORT}`);
});
```

**Key Points**:
- CORS origins should **NOT** include path segments (no `/book_hackathon`)
- Origins must be protocol + domain + port only
- Better-Auth handler MUST be mounted before `express.json()`
- Use `credentials: true` for cookie-based auth
- Bind to `0.0.0.0` for cloud deployments (Vercel, Hugging Face)

### 3. Client Configuration (src/lib/better-auth/client.ts)

```typescript
import { createAuthClient } from 'better-auth/react';

const getAuthServerURL = () => {
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    if (hostname === 'your-production-domain.com' || hostname === 'your-github-pages.io') {
      return 'https://your-auth-server.vercel.app/api/auth';
    }
  }
  return 'http://localhost:3001/api/auth';
};

export const authClient = createAuthClient({
  baseURL: getAuthServerURL(),
  fetchOptions: {
    credentials: 'include', // CRITICAL: Include cookies in requests
  },
});
```

**Key Points**:
- Environment detection based on `window.location.hostname`
- Always include `credentials: 'include'` for cookie-based sessions
- Do NOT set manual `Origin` header (let the browser handle it)
- Use `/api/auth` path consistently

### 4. Authentication Hook (src/auth/hooks/useAuth.ts)

```typescript
import { authClient } from '@/lib/better-auth/client';
import { useState, useEffect } from 'react';

export function useAuth() {
  const [user, setUser] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    authClient.session.get()
      .then(({ data }) => {
        setUser(data?.user || null);
      })
      .catch(() => setUser(null))
      .finally(() => setIsLoading(false));
  }, []);

  const signIn = async (email: string, password: string) => {
    const result = await authClient.signIn.email({ email, password });
    if (result.data?.user) {
      setUser(result.data.user);
    }
    return result;
  };

  const signOut = async () => {
    await authClient.signOut();
    setUser(null);
  };

  return {
    user,
    isAuthenticated: !!user,
    isLoading,
    signIn,
    signOut,
  };
}
```

## Environment Variables

### Root .env (for Docusaurus)
```env
# Better-Auth secret (32+ bytes, base64-encoded)
BETTER_AUTH_SECRET=your-generated-secret-here

# Production auth server URL
AUTH_SERVER_URL=https://your-auth-server.vercel.app
```

### auth-server/.env (for Auth Server)
```env
# Database connection
NEON_DATABASE_URL=postgresql://user:password@host/database

# Auth server configuration
AUTH_BASE_URL=https://your-auth-server.vercel.app
FRONTEND_URL=https://your-frontend-domain.com

# Better-Auth secret (must match root .env)
BETTER_AUTH_SECRET=your-generated-secret-here

# Port configuration (optional)
PORT=3001
```

**Generate Secret**:
```bash
node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"
```

## Common Issues and Solutions

### 1. INVALID_ORIGIN Error (403)

**Symptom**: `{"code":"INVALID_ORIGIN","message":"Invalid origin"}` on authentication requests

**Root Cause**: Better-Auth's origin validation is rejecting the request

**Solutions**:
1. Add the frontend origin to `trustedOrigins` array in auth config
2. Ensure CORS origins in Express don't include path segments
3. For development, can use `advanced.disableOriginCheck: true` (NOT recommended for production)
4. Verify origin header is being set correctly by browser

**Example Fix**:
```javascript
// In auth.js
trustedOrigins: [
  'http://localhost:3000',  // ✅ Correct
  'https://your-domain.com', // ✅ Correct
  // NOT 'http://localhost:3000/book_hackathon' ❌ Wrong
],
```

### 2. BETTER_AUTH_SECRET Error on Deployment

**Symptom**: "You are using the default secret" error on Vercel/production

**Root Cause**: Environment variable not set in deployment platform

**Solutions**:
1. Generate a cryptographically secure secret
2. Add to deployment platform environment variables:
   - Vercel: Project Settings → Environment Variables
   - Add `BETTER_AUTH_SECRET` for all environments (Production, Preview, Development)
3. Update auth config to use the secret:
   ```javascript
   export const auth = betterAuth({
     secret: process.env.BETTER_AUTH_SECRET || "",
     // ... rest of config
   });
   ```
4. Redeploy to apply changes

### 3. auth-client.js 404 Error

**Symptom**: `GET /auth-client.js 404 (Not Found)` in browser console

**Root Cause**: Docusaurus plugin trying to inject a non-existent script

**Solution**: Remove unnecessary script injection from Docusaurus plugin
```javascript
// In plugins/docusaurus-auth/index.js
// REMOVE the injectHtmlTags() function - Better-Auth React client is already imported via React
module.exports = function (context, options) {
  return {
    name: 'docusaurus-auth',
    // Don't inject scripts - use React imports instead
  };
};
```

### 4. Cross-Origin Cookie Issues

**Symptom**: Sessions not persisting across requests in production

**Root Cause**: Incorrect cookie attributes for cross-origin scenarios

**Solutions**:
1. Set `sameSite: 'none'` and `secure: true` for HTTPS deployments
2. Enable `partitioned: true` for modern browser standards
3. Use the correct Better-Auth API: `defaultCookieAttributes` in `advanced` config
4. Ensure `credentials: 'include'` in client fetch options

**Example**:
```javascript
// In auth.js
advanced: {
  defaultCookieAttributes: {
    sameSite: isHTTPS ? 'none' : 'lax',
    secure: isHTTPS,
    httpOnly: true,
    partitioned: true,
  },
  useSecureCookies: isHTTPS,
},
```

### 5. Database Migration Issues

**Symptom**: Schema errors or missing tables

**Solution**: Run Better-Auth migrations
```bash
cd auth-server
npx @better-auth/cli migrate
```

## Development Workflow

### 1. Initial Setup

```bash
# Install dependencies
npm install better-auth better-sqlite3 drizzle-orm
cd auth-server && npm install

# Generate secret
node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"

# Add to .env files (root and auth-server)
echo "BETTER_AUTH_SECRET=<generated-secret>" >> .env
echo "BETTER_AUTH_SECRET=<generated-secret>" >> auth-server/.env

# Run migrations
cd auth-server
npx @better-auth/cli migrate
```

### 2. Development Servers

**Terminal 1 - Auth Server**:
```bash
cd auth-server
npm run dev  # or: node server.js
```

**Terminal 2 - Docusaurus**:
```bash
npm start
```

### 3. Testing Checklist

- [ ] Health check returns 200: `curl http://localhost:3001/health`
- [ ] Signup form accessible at `/signup`
- [ ] User can register with email/password
- [ ] Background questions are collected and stored
- [ ] User can sign in with credentials
- [ ] Session persists across page refreshes
- [ ] User profile data accessible via `useAuth()` hook
- [ ] Sign out clears session
- [ ] Navbar shows correct authentication state

## Production Deployment

### 1. Auth Server (Vercel/Hugging Face)

**Vercel**:
1. Create new project from `auth-server` directory
2. Set environment variables:
   - `NEON_DATABASE_URL`: PostgreSQL connection string
   - `BETTER_AUTH_SECRET`: Generated secret
   - `AUTH_BASE_URL`: Vercel deployment URL
   - `FRONTEND_URL`: Docusaurus production URL
3. Deploy and note the URL

**Hugging Face Spaces**:
1. Create Space with Node.js SDK
2. Copy `auth-server` contents
3. Set secrets in Space settings
4. Set `PORT=7860` and `HOST=0.0.0.0`

### 2. Docusaurus (GitHub Pages/Vercel)

1. Update `src/lib/better-auth/client.ts` with production auth server URL
2. Build: `npm run build`
3. Deploy to hosting platform
4. Set environment variables in platform:
   - `BETTER_AUTH_SECRET`: Same secret as auth server

### 3. Post-Deployment Validation

```bash
# Test health endpoint
curl https://your-auth-server.vercel.app/health

# Test signup endpoint
curl -X POST https://your-auth-server.vercel.app/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123","name":"Test User"}'

# Check frontend can connect
# Open browser devtools → Network tab → Perform signup/signin
# Verify requests to auth server succeed with cookies
```

## Best Practices

### Security
1. **Always** use environment variables for secrets, never hardcode
2. **Always** validate and sanitize user inputs in custom endpoints
3. **Always** use HTTPS in production (required for `sameSite: 'none'`)
4. **Consider** enabling email verification for production
5. **Implement** rate limiting for auth endpoints
6. **Use** strong password requirements (minimum 8 characters, complexity rules)

### Performance
1. **Cache** session data client-side to reduce API calls
2. **Use** connection pooling for database (pg Pool)
3. **Implement** proper error boundaries in React components
4. **Monitor** auth server response times and database queries

### Code Quality
1. **Use** TypeScript for type safety across client and server
2. **Separate** concerns: auth logic, UI components, API services
3. **Test** critical paths: signup, signin, session persistence, signout
4. **Document** custom fields and their purpose
5. **Version** your auth schema migrations

### User Experience
1. **Provide** clear error messages (specific for validation, generic for security)
2. **Show** loading states during authentication operations
3. **Handle** offline scenarios gracefully
4. **Implement** auto-redirect after successful authentication
5. **Persist** session across browser restarts (7-day expiry)

## Extension Points

### Adding Social Providers
```javascript
// In auth.js
import { google, github } from 'better-auth/social';

export const auth = betterAuth({
  // ... existing config
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID,
      clientSecret: process.env.GITHUB_CLIENT_SECRET,
    },
  },
});
```

### Custom Endpoints for Profile Updates
```javascript
// In auth.js - add custom plugin
import { createAuthEndpoint } from 'better-auth';

const updateProfilePlugin = {
  id: 'update-profile',
  endpoints: {
    updateBackground: createAuthEndpoint(
      '/update-background',
      {
        method: 'POST',
        body: z.object({
          softwareBackground: z.any(),
          hardwareBackground: z.any(),
        }),
      },
      async (ctx) => {
        // Update user profile in database
        // Return updated user
      }
    ),
  },
};

export const auth = betterAuth({
  plugins: [updateProfilePlugin],
  // ... rest of config
});
```

### Content Personalization
```typescript
// In a Docusaurus component
import { useAuth } from '@/auth/hooks/useAuth';

export function PersonalizedContent() {
  const { user } = useAuth();

  // Adapt content based on user background
  const showAdvancedContent =
    user?.softwareBackground?.experienceLevel === 'advanced';

  const preferredLanguage =
    user?.softwareBackground?.languages?.[0] || 'Python';

  return (
    <div>
      {showAdvancedContent ? (
        <AdvancedTutorial language={preferredLanguage} />
      ) : (
        <BeginnerTutorial language={preferredLanguage} />
      )}
    </div>
  );
}
```

## References

- Feature Spec: `specs/002-better-auth-signup/spec.md`
- Implementation Plan: `specs/002-better-auth-signup/plan.md`
- ADR: `history/adr/0001-better-auth-architecture-decision.md`
- Better-Auth Docs: https://www.better-auth.com/docs
- Drizzle ORM: https://orm.drizzle.team/

## Changelog

- **2025-12-13**: Initial skill creation based on successful Docusaurus integration
- Fixed INVALID_ORIGIN error by configuring trustedOrigins
- Fixed BETTER_AUTH_SECRET error for deployment
- Fixed auth-client.js 404 by removing unnecessary script injection
- Fixed cross-origin cookies with defaultCookieAttributes API
