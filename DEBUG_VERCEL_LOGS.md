# How to Check Vercel Logs and Debug 500 Error

## Step 1: Wait for Deployment (2-3 minutes)
Vercel is now deploying the debug version. Wait for the deployment to complete.

## Step 2: Check Vercel Logs

### Option A: Vercel Dashboard (Recommended)
1. Go to https://vercel.com/dashboard
2. Click on your project: `book-hackathon-alpha`
3. Click on the latest deployment
4. Click on "Runtime Logs" tab
5. Look for these log messages:
   - `[Auth] Environment check:` - Shows which env vars are set
   - `[Auth Config] Environment:` - Shows HTTPS detection
   - `[Server] Auth request:` - Shows incoming sign-in requests
   - `[Server] Auth handler error:` - **THIS IS THE CRITICAL ERROR MESSAGE**

### Option B: Vercel CLI
```bash
# Install Vercel CLI if not already installed
npm i -g vercel

# Login
vercel login

# View logs
vercel logs book-hackathon-alpha --follow
```

## Step 3: Try Sign In Again

1. Go to https://osqazi.github.io/book_hackathon
2. Open DevTools (F12) > Console tab
3. Try to sign in
4. Immediately check Vercel logs for the error

## Step 4: What to Look For

### Expected Logs on Server Start:
```
[Auth] Environment check: {
  hasDatabase: true,
  hasSecret: true,
  authBaseURL: 'https://book-hackathon-alpha.vercel.app',
  frontendURL: 'https://osqazi.github.io/book_hackathon',
  nodeEnv: 'production'
}
```

### If hasDatabase or hasSecret is false:
**Problem**: Environment variables not set in Vercel
**Solution**: Add them in Vercel Dashboard > Settings > Environment Variables

### Expected Logs on Sign-In Request:
```
[Server] Auth request: {
  method: 'POST',
  url: '/api/auth/sign-in/email',
  path: '/api/auth/sign-in/email',
  origin: 'https://osqazi.github.io',
  contentType: 'application/json'
}
```

### If You See Error Logs:
```
[Server] Auth handler error: Error: ...
[Server] Error stack: ...
```

**Copy the entire error message and stack trace** - this will tell us exactly what's wrong.

## Common Issues and Solutions

### Issue 1: Missing BETTER_AUTH_SECRET
**Error**: `secret must be provided`
**Solution**:
1. Go to Vercel Dashboard > Project Settings > Environment Variables
2. Add: `BETTER_AUTH_SECRET=S12aLXBp4w5ndFK8hACHYbCa7RKLkH13SyA36Kum4pM=`
3. Redeploy

### Issue 2: Database Connection Error
**Error**: `connect ECONNREFUSED` or `SSL error`
**Solution**:
1. Verify NEON_DATABASE_URL in Vercel environment variables
2. Check if Neon database is accessible
3. Verify connection string format

### Issue 3: Invalid Configuration
**Error**: `Invalid configuration` or `Unknown option`
**Solution**: Check Better Auth version compatibility

### Issue 4: Cookie/CORS Issues
**Error**: `Origin not allowed` or `Cookie blocked`
**Solution**: Already handled in our CORS config

## Step 5: Share the Logs

After checking the logs, share:
1. The `[Auth] Environment check:` output
2. The `[Server] Auth handler error:` message
3. The full error stack trace

This will help identify the exact issue.

## Quick Test Commands

### Test Health Endpoint:
```bash
curl https://book-hackathon-alpha.vercel.app/health
```
Expected: `{"status":"ok","service":"Auth Server"}`

### Test with Verbose Logging:
```bash
curl -v -X POST https://book-hackathon-alpha.vercel.app/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -H "Origin: https://osqazi.github.io" \
  -d '{"email":"test@example.com","password":"testpass123"}'
```

This will show:
- Response headers (including Set-Cookie if successful)
- Response body (error message if failed)
- HTTP status code

## Environment Variables Checklist

Verify these are set in Vercel Dashboard:
- [ ] `NEON_DATABASE_URL` - PostgreSQL connection string
- [ ] `BETTER_AUTH_SECRET` - Auth encryption key
- [ ] `AUTH_BASE_URL` - https://book-hackathon-alpha.vercel.app
- [ ] `FRONTEND_URL` - https://osqazi.github.io/book_hackathon
- [ ] `NODE_ENV` - production

**Note**: After adding/changing environment variables, you MUST redeploy (Vercel auto-redeploys on new deployment, or manually trigger).
