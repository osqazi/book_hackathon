# Production Authentication Debugging Guide

## Current Status
- ✅ Auth client detecting correct production URL
- ✅ Session fetch succeeding (no error)
- ❌ Session returning null data

## Step-by-Step Debugging

### Step 1: Verify Vercel Environment Variables ⚠️ CRITICAL

**Action Required:**
1. Go to: https://vercel.com/dashboard
2. Select project: `book-hackathon-alpha`
3. Go to: Settings → Environment Variables
4. **VERIFY** these variables exist:
   - `BETTER_AUTH_SECRET`
   - `NEON_DATABASE_URL`
   - `NODE_ENV`
   - `AUTH_BASE_URL`
   - `FRONTEND_URL`

**If any are missing:**
- Add them using the values from `PRODUCTION_AUTH_FIX.md`
- **MUST REDEPLOY** after adding (Deployments tab → Redeploy)

**To verify they're loaded:**
- Check Vercel deployment logs
- Look for: `[dotenv] injecting env` messages

---

### Step 2: Test Sign-In Flow with Browser DevTools

**Open DevTools Before Signing In:**

1. **Open Browser DevTools** (F12)
2. Go to **Network** tab
3. Check **Preserve log**
4. Filter by: `auth`

**Now Sign In:**

Go to: https://osqazi.github.io/book_hackathon/signin

**Check Network Tab - Sign In Request:**

Look for: `POST https://book-hackathon-alpha.vercel.app/api/auth/sign-in/email`

**Expected Response Headers:**
```
Set-Cookie: better-auth.session_token=...; SameSite=None; Secure; HttpOnly; Path=/
```

**If you DON'T see Set-Cookie headers:**
- ❌ Sign-in failed on the server side
- Check Console tab for errors
- BETTER_AUTH_SECRET is missing on Vercel

**If you DO see Set-Cookie headers:**
- ✅ Cookies are being set
- Continue to Step 3

---

### Step 3: Verify Cookies Are Stored

**Check Application Tab:**

1. Open DevTools → **Application** tab
2. Expand **Cookies** in left sidebar
3. Look for: `https://book-hackathon-alpha.vercel.app`

**Expected Cookies:**
```
better-auth.session_token    (value should be a long string)
Domain: book-hackathon-alpha.vercel.app
SameSite: None
Secure: ✓
HttpOnly: ✓
```

**If cookies are NOT there:**
- ❌ Browser is blocking third-party cookies
- Try different browser (Chrome/Edge recommended)
- Check browser privacy settings

**If cookies ARE there:**
- ✅ Cookies stored successfully
- Continue to Step 4

---

### Step 4: Verify Cookies Are Sent with Session Request

**Check Network Tab - Session Request:**

Look for: `GET https://book-hackathon-alpha.vercel.app/api/auth/get-session`

**Click on the request → Headers tab**

**Expected Request Headers:**
```
Cookie: better-auth.session_token=...
```

**If Cookie header is MISSING:**
- ❌ Browser not sending cookies (privacy settings)
- ❌ Cookies expired or invalid

**If Cookie header is PRESENT but response is still null:**
- ❌ Server can't decrypt/validate the session
- BETTER_AUTH_SECRET mismatch or missing on Vercel

---

### Step 5: Check Console Logs During Sign-In

**After clicking "Sign In" button, check Console:**

**Expected logs:**
```javascript
[AuthProvider] Signing in...
[AuthProvider] Sign in result: {data: {...}, error: null}
[AuthProvider] Sign in successful, fetching fresh session...
[AuthProvider] Fresh session: {data: {user: {...}, session: {...}}, error: null}
```

**If you see errors:**
- Post the error message for further debugging

**If sign in result has error:**
- Check the error message
- May indicate database, network, or auth config issues

---

### Step 6: Test Auth Server Directly

**Open a new tab and test:**

```bash
# Test 1: Health check
https://book-hackathon-alpha.vercel.app/health
Expected: {"status":"ok","service":"Auth Server"}

# Test 2: Get session (should return null if not logged in)
https://book-hackathon-alpha.vercel.app/api/auth/get-session
Expected: {"data":null,"error":null}
```

**If health check fails:**
- ❌ Auth server not deployed correctly
- Check Vercel deployment status

---

## Common Issues & Solutions

### Issue 1: BETTER_AUTH_SECRET Not Set on Vercel

**Symptoms:**
- Sign-in request succeeds but doesn't set cookies
- Session always returns null
- No Set-Cookie headers in response

**Solution:**
1. Add `BETTER_AUTH_SECRET` to Vercel environment variables
2. Value: `S12aLXBp4w5ndFK8hACHYbCa7RKLkH13SyA36Kum4pM=`
3. **MUST REDEPLOY** after adding

**How to verify it's fixed:**
- Check Vercel deployment logs
- Look for successful session creation in logs
- Set-Cookie headers should appear in sign-in response

---

### Issue 2: Browser Blocking Third-Party Cookies

**Symptoms:**
- Set-Cookie headers appear in response
- But cookies don't show in Application → Cookies
- Session returns null

**Solution:**
- Use Chrome or Edge (best compatibility)
- Check browser settings: `chrome://settings/cookies`
- Ensure "Block third-party cookies" is OFF
- Try incognito/private mode
- Clear all cookies and cache

**Alternative (if browser blocks persist):**
- Deploy frontend to same domain as auth server
- Use subdomain: `app.yourdomain.com` + `auth.yourdomain.com`

---

### Issue 3: CORS Errors

**Symptoms:**
- Console shows CORS errors
- Requests fail with network errors

**Solution:**
- Verify `FRONTEND_URL` env var on Vercel is correct
- Should be: `https://osqazi.github.io/book_hackathon`
- Check `trustedOrigins` in `auth-server/auth.js`

---

### Issue 4: Database Connection Issues

**Symptoms:**
- Sign-in succeeds but session still null
- Vercel logs show database errors

**Solution:**
- Verify `NEON_DATABASE_URL` is correct on Vercel
- Test database connection
- Check Neon dashboard for connection limits

---

## Quick Checklist

Before testing again:
- [ ] All environment variables added to Vercel
- [ ] Vercel project redeployed after adding env vars
- [ ] Browser cookies and cache cleared
- [ ] Using Chrome or Edge browser
- [ ] DevTools Network tab open with "Preserve log" enabled
- [ ] DevTools Console tab open to see logs

During sign-in test:
- [ ] Sign-in request succeeds (200 status)
- [ ] Set-Cookie headers present in response
- [ ] Cookies stored in browser (Application tab)
- [ ] Cookie sent with get-session request
- [ ] Session returns user data (not null)

---

## What to Check Next

Please perform these checks and report back:

1. **Vercel Environment Variables:**
   - Screenshot of Vercel env vars (hide sensitive values)
   - Confirm you redeployed after adding them

2. **Network Tab (Sign-In Request):**
   - Status code
   - Response headers (especially Set-Cookie)
   - Response body

3. **Application → Cookies:**
   - Screenshot showing if cookies exist for book-hackathon-alpha.vercel.app
   - Cookie properties (SameSite, Secure, HttpOnly)

4. **Console Logs (Full Sign-In Flow):**
   - All logs from clicking "Sign In" to redirect
   - Any errors or warnings

This information will help identify the exact issue!
