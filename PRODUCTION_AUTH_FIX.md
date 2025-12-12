# Production Authentication Fix Guide

## Issues Identified

After investigating the authentication flow between GitHub Pages (https://osqazi.github.io/book_hackathon) and Vercel (https://book-hackathon-alpha.vercel.app), the following issues were found:

### 1. **Missing BETTER_AUTH_SECRET in Production Environment**
The `.env.production` file is missing the `BETTER_AUTH_SECRET` variable, which is **critical** for Better-Auth to encrypt and sign session tokens.

### 2. **Cross-Origin Cookie Configuration**
While the code is correctly configured for cross-origin cookies (`sameSite: 'none'`, `secure: true`), the environment variables must be properly set on Vercel.

### 3. **Potential Cookie Domain Issues**
The cookies might not be persisting across the cross-origin request from GitHub Pages to Vercel.

---

## Solution Steps

### Step 1: Add Missing Environment Variables to Vercel

You need to add the following environment variables to your Vercel project (https://book-hackathon-alpha.vercel.app):

1. Go to: https://vercel.com/dashboard → Select your project → Settings → Environment Variables

2. Add these variables:

```bash
# Required - Better Auth Secret (use the same value from your .env file)
BETTER_AUTH_SECRET=S12aLXBp4w5ndFK8hACHYbCa7RKLkH13SyA36Kum4pM=

# Required - Database URL
NEON_DATABASE_URL=postgresql://neondb_owner:npg_koREpi02cZmN@ep-curly-snow-a4j5syts-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

# Required - Environment
NODE_ENV=production

# Required - Auth Server Base URL
AUTH_BASE_URL=https://book-hackathon-alpha.vercel.app

# Required - Frontend URL
FRONTEND_URL=https://osqazi.github.io/book_hackathon
```

**IMPORTANT:** After adding these, you **must redeploy** your Vercel project for the changes to take effect.

### Step 2: Verify Auth Server Configuration

The auth server (`auth-server/auth.js`) should already have the correct configuration:
- ✅ `sameSite: 'none'` for cross-origin cookies
- ✅ `secure: true` for HTTPS
- ✅ `credentials: 'include'` in client configuration
- ✅ Correct CORS origins

### Step 3: Test the Authentication Flow

After redeploying Vercel:

1. **Clear browser cookies and cache** for both:
   - https://osqazi.github.io
   - https://book-hackathon-alpha.vercel.app

2. **Open browser DevTools** → Network tab → Enable "Preserve log"

3. **Sign in** via https://osqazi.github.io/book_hackathon/signin

4. **Check for:**
   - ✅ Successful POST to `https://book-hackathon-alpha.vercel.app/api/auth/sign-in/email`
   - ✅ Response includes `Set-Cookie` headers with `SameSite=None; Secure`
   - ✅ Subsequent GET to `https://book-hackathon-alpha.vercel.app/api/auth/get-session` includes cookies
   - ✅ Session response returns user data

### Step 4: Debug with Browser Console

Open the browser console on https://osqazi.github.io/book_hackathon and check for:

```javascript
[AuthClient] Current hostname: osqazi.github.io
[AuthClient] Using production auth URL: https://book-hackathon-alpha.vercel.app/api/auth
[AuthProvider] Fetching session from auth server...
[AuthProvider] Session response: {...}
```

---

## Expected Behavior After Fix

### ✅ Sign In Flow:
1. User enters credentials on GitHub Pages
2. POST request to Vercel auth server
3. Server sets session cookie with `SameSite=None; Secure`
4. Cookie is saved in browser
5. Redirect to personalization page

### ✅ Personalization Page:
1. AuthProvider fetches session from Vercel
2. Cookie is sent with request
3. User data is retrieved
4. Page shows personalized content

### ✅ Navbar:
1. UserProfileNavbarItem receives user from AuthProvider
2. Shows "Hello, [username]!"
3. Shows "Sign Out" button

---

## Troubleshooting

### If authentication still doesn't work after fix:

#### Issue: Cookies not being set
**Check:**
- Browser DevTools → Application → Cookies
- Are cookies being set for `book-hackathon-alpha.vercel.app`?
- Do they have `SameSite=None` and `Secure` flags?

**Solution:**
- Ensure HTTPS is used (not HTTP)
- Check browser privacy settings (some browsers block 3rd-party cookies)

#### Issue: Session returns null
**Check:**
- Browser Console logs from `[AuthProvider]`
- Network tab → Check `/api/auth/get-session` response

**Solution:**
- Verify `BETTER_AUTH_SECRET` is set on Vercel
- Check database connection (NEON_DATABASE_URL)
- Verify user exists in database

#### Issue: CORS errors
**Check:**
- Browser Console for CORS errors
- Network tab → Check response headers

**Solution:**
- Verify `FRONTEND_URL` environment variable on Vercel
- Check `trustedOrigins` array in `auth-server/auth.js`

---

## Additional Notes

### Browser Compatibility
Some browsers (Safari, Firefox in private mode) may block 3rd-party cookies even with `SameSite=None`. This is expected behavior for privacy protection.

**Alternative Solution:**
If cookies are blocked, consider:
1. Moving both frontend and auth server to the same domain
2. Using a subdomain setup (e.g., `app.yourdomain.com` and `auth.yourdomain.com`)
3. Implementing token-based authentication stored in localStorage (less secure)

### Security Considerations
- ✅ `BETTER_AUTH_SECRET` should be kept secret and not committed to git
- ✅ Always use HTTPS in production
- ✅ Session expiry is set to 7 days (configurable in `auth-server/auth.js`)
- ✅ Cookies are httpOnly and secure

---

## Quick Verification Checklist

Before testing:
- [ ] `BETTER_AUTH_SECRET` added to Vercel
- [ ] All environment variables added to Vercel
- [ ] Vercel project redeployed
- [ ] Browser cookies cleared
- [ ] Browser console open for debugging

During testing:
- [ ] Sign in successful (no errors in console)
- [ ] Cookie set in browser (check DevTools → Application)
- [ ] Session retrieved after sign in
- [ ] Navbar shows username and sign out button
- [ ] Personalization page shows content
- [ ] Sign out works correctly

---

## Files Modified/Checked

### ✅ Already Correct:
- `src/lib/better-auth/client.ts` - Correctly detects production URL
- `auth-server/auth.js` - Correct cross-origin cookie config
- `auth-server/server.js` - Correct CORS configuration
- `src/auth/context/AuthProvider.tsx` - Using authClient correctly

### ⚠️ Needs Attention:
- `.env.production` - Missing `BETTER_AUTH_SECRET` (add to Vercel)
- Vercel Environment Variables - Need to be set

---

## Contact & Support

If issues persist after following this guide:
1. Check browser console for specific error messages
2. Check Vercel deployment logs for server-side errors
3. Test with different browsers to rule out browser-specific issues
