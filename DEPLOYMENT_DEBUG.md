# Session Issue Fix and Deployment Guide

## Issue Summary
Session data works on localhost but fails on production due to cross-origin cookie restrictions between `osqazi.github.io` (GitHub Pages) and `book-hackathon-alpha.vercel.app` (Vercel).

## Changes Made

### 1. Auth Server Configuration (`auth-server/auth.js`)
- ✅ Added explicit `secret` from environment variable
- ✅ Added `path: '/'` to cookie attributes
- ✅ Added `updateAge` to session configuration
- ✅ Verified `sameSite: 'none'` and `secure: true` for production

### 2. Server CORS Configuration (`auth-server/server.js`)
- ✅ Added `OPTIONS` method to CORS
- ✅ Added `Cookie` to allowed headers
- ✅ Added debug logging middleware to track cookies

### 3. Client Configuration (`src/lib/better-auth/client.ts`)
- ✅ Added explicit `mode: 'cors'` to fetch options
- ✅ Added explicit headers configuration

## Deployment Steps

### Step 1: Verify Vercel Environment Variables
Make sure these variables are set in your Vercel project:
```
NEON_DATABASE_URL=postgresql://neondb_owner:npg_koREpi02cZmN@ep-curly-snow-a4j5syts-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
AUTH_BASE_URL=https://book-hackathon-alpha.vercel.app
FRONTEND_URL=https://osqazi.github.io/book_hackathon
BETTER_AUTH_SECRET=S12aLXBp4w5ndFK8hACHYbCa7RKLkH13SyA36Kum4pM=
NODE_ENV=production
```

### Step 2: Deploy to Vercel
```bash
cd auth-server
git add .
git commit -m "Fix: Cross-origin session handling with explicit cookie configuration"
git push
```

Vercel will automatically deploy the changes.

### Step 3: Test the Deployment

1. **Test Health Endpoint**
   ```bash
   curl https://book-hackathon-alpha.vercel.app/health
   ```
   Expected: `{"status":"ok","service":"Auth Server"}`

2. **Test Sign In with Cookie Inspection**
   - Open https://osqazi.github.io/book_hackathon in Chrome/Firefox
   - Open DevTools (F12)
   - Go to Network tab
   - Sign in with your credentials
   - Check the response headers for `Set-Cookie`
   - Check if cookies are stored in Application > Cookies

3. **Check Vercel Logs**
   ```bash
   vercel logs book-hackathon-alpha
   ```
   Look for the debug logs:
   - `[Auth Config] Environment:` - Should show HTTPS and sameSite: none
   - `[Server] Request:` - Should show incoming requests and cookie headers

## Debugging Checklist

### If Session Still Doesn't Work:

1. **Browser Cookie Settings**
   - [ ] Chrome: Settings > Privacy > "Allow all cookies"
   - [ ] Firefox: Settings > Privacy > "Standard" tracking protection
   - [ ] Safari: Settings > Privacy > Disable "Prevent cross-site tracking"

2. **Verify Cookies Are Set**
   - [ ] Open DevTools > Network
   - [ ] Sign in
   - [ ] Find the `/api/auth/sign-in/email` request
   - [ ] Check Response Headers for `Set-Cookie: better-auth.session_token=...`
   - [ ] Verify cookie attributes: `SameSite=None; Secure; HttpOnly; Path=/`

3. **Verify Cookies Are Sent**
   - [ ] After signing in, navigate to any page
   - [ ] Open DevTools > Network
   - [ ] Find any request to `book-hackathon-alpha.vercel.app`
   - [ ] Check Request Headers for `Cookie: better-auth.session_token=...`

4. **Check Browser Console**
   - [ ] Look for `[AuthProvider]` logs
   - [ ] Check for CORS errors
   - [ ] Check for cookie warnings

## Alternative Solution: Custom Domain

If third-party cookies continue to be blocked, consider these alternatives:

### Option 1: Use a Reverse Proxy
Deploy the auth server as a subdirectory of GitHub Pages:
- Frontend: `osqazi.github.io/book_hackathon`
- Auth Server: `osqazi.github.io/book_hackathon/api/auth` (proxied to Vercel)

### Option 2: Custom Domain with Subdomain
Use a custom domain:
- Frontend: `robotics.yourdomain.com`
- Auth Server: `api.robotics.yourdomain.com`

This makes them same-site, avoiding third-party cookie restrictions.

### Option 3: Token-based Auth (Last Resort)
Switch from cookie-based to token-based authentication:
- Store tokens in localStorage
- Send tokens in Authorization header
- Trade-off: Less secure against XSS attacks

## Testing Commands

### Test locally first:
```bash
# Terminal 1: Start auth server
cd auth-server
npm run dev

# Terminal 2: Start frontend
npm run dev

# Test sign in
# Open http://localhost:3000 and sign in
# Session should persist
```

### Test production:
```bash
# Check Vercel deployment
curl -i https://book-hackathon-alpha.vercel.app/health

# Check if auth endpoint is accessible
curl -i https://book-hackathon-alpha.vercel.app/api/auth/get-session \
  -H "Origin: https://osqazi.github.io" \
  -H "Cookie: better-auth.session_token=YOUR_TOKEN_HERE"
```

## Next Steps

1. Deploy the changes to Vercel
2. Clear all cookies in your browser
3. Test sign in on production
4. Check browser DevTools for cookie issues
5. Review Vercel logs for server-side errors

If issues persist, check:
- Browser's third-party cookie blocking status
- Vercel logs for any error messages
- Network tab for failed requests or CORS errors
