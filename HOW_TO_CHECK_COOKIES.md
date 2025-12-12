# How to Check Set-Cookie Headers - Step by Step

## Part 1: Check if Cookies Are Being Set During Sign-In

### Step 1: Open Chrome DevTools
1. Go to: https://osqazi.github.io/book_hackathon/signin
2. Press **F12** (or right-click → Inspect)
3. Click the **Network** tab at the top
4. Make sure the **red recording button** is on (if not, click it)
5. Check the box that says **"Preserve log"** (important!)

### Step 2: Clear Everything First
1. Click the **🚫 Clear** button in the Network tab (circle with a line through it)
2. Now you have a clean slate

### Step 3: Sign In and Watch
1. Enter your email and password
2. Click **Sign In**
3. Watch the Network tab - you'll see requests appearing

### Step 4: Find the Sign-In Request
1. In the Network tab, look for a request that says: **`email`** or **`sign-in`**
2. The full URL should be: `https://book-hackathon-alpha.vercel.app/api/auth/sign-in/email`
3. Click on that request to select it

### Step 5: Check Response Headers
1. With the request selected, you'll see tabs: **Headers**, Preview, Response, etc.
2. Click the **Headers** tab
3. Scroll down to find **"Response Headers"** section
4. **LOOK FOR: `set-cookie`** or **`Set-Cookie`**

### What to Report:

**Option A: You see Set-Cookie**
```
set-cookie: better-auth.session_token=eyJhb...; SameSite=None; Secure; HttpOnly
```
✅ **GOOD!** Cookies are being set. Continue to Part 2.

**Option B: You DON'T see Set-Cookie**
```
(No set-cookie in the list)
```
❌ **PROBLEM!** Environment variables might not be loaded. See "Fix for Option B" below.

---

## Part 2: Check if Cookies Are Stored in Browser

### Step 1: Open Application Tab
1. In DevTools, click the **Application** tab (next to Network)
2. In the left sidebar, look for **"Cookies"** and expand it
3. You should see:
   - `https://osqazi.github.io`
   - `https://book-hackathon-alpha.vercel.app`
4. Click on **`https://book-hackathon-alpha.vercel.app`**

### Step 2: Check for Session Cookie
Look for a cookie named: **`better-auth.session_token`**

**Option A: Cookie exists**
✅ Click on it and check:
- **SameSite** column should show: **None**
- **Secure** column should show: **✓** (checkmark)
- **HttpOnly** column should show: **✓** (checkmark)

**Option B: Cookie doesn't exist**
❌ Browser is blocking third-party cookies. See "Fix for Option B" below.

---

## Part 3: Check if Cookies Are Sent with Session Request

### Step 1: Look for Session Request
1. Back in the **Network** tab
2. Look for a request to: **`get-session`**
3. Full URL: `https://book-hackathon-alpha.vercel.app/api/auth/get-session`
4. Click on it

### Step 2: Check Request Headers
1. Click the **Headers** tab
2. Scroll down to find **"Request Headers"** section
3. **LOOK FOR: `cookie`** or **`Cookie`**

**Option A: You see Cookie header**
```
cookie: better-auth.session_token=eyJhb...
```
✅ **GOOD!** Cookies are being sent. Continue to Part 4.

**Option B: You DON'T see Cookie header**
```
(No cookie in the list)
```
❌ **PROBLEM!** Browser not sending cookies. See "Fix for Option B" below.

---

## Part 4: Check Response from Get-Session

### Still on the get-session request:
1. Click the **Response** or **Preview** tab
2. You should see JSON like:
```json
{
  "data": {
    "user": {...},
    "session": {...}
  },
  "error": null
}
```

**If data is null:**
```json
{
  "data": null,
  "error": null
}
```
❌ Session not valid. See troubleshooting below.

---

## Quick Fixes

### Fix for Part 1 (No Set-Cookie Headers):

**The server isn't setting cookies. This means:**

1. **Environment variables not loaded on Vercel**
   - Go to Vercel Dashboard
   - Check Environment Variables again
   - Make sure they're set for **"Production"** environment
   - **Redeploy from Deployments tab**

2. **Sign-in failing on server**
   - Check the Response tab of the sign-in request
   - Look for error messages
   - Report any errors you see

### Fix for Part 2 (Cookies Not Stored):

**Browser is blocking third-party cookies.**

**Chrome Fix:**
1. Go to: `chrome://settings/cookies`
2. Make sure **"Block third-party cookies"** is **OFF**
3. OR add exception:
   - Click **"Add"** under "Sites that can always use cookies"
   - Add: `https://book-hackathon-alpha.vercel.app`
4. Clear all cookies and try again

**Edge Fix:**
1. Go to: `edge://settings/content/cookies`
2. Make sure **"Block third-party cookies"** is **OFF**
3. Clear all cookies and try again

### Fix for Part 3 (Cookie Not Sent):

**Check browser privacy settings:**
1. Make sure you're not in "Incognito/Private" mode with strict settings
2. Check if Enhanced Tracking Protection is blocking cookies
3. Try regular mode with third-party cookies enabled

### Fix for Part 4 (Session Returns Null):

**Even though cookie is sent, session is invalid:**

1. **BETTER_AUTH_SECRET mismatch**
   - The secret on Vercel must EXACTLY match
   - Value: `S12aLXBp4w5ndFK8hACHYbCa7RKLkH13SyA36Kum4pM=`
   - Check for extra spaces or characters
   - Redeploy after fixing

2. **Database issue**
   - Check Vercel logs for database errors
   - Verify NEON_DATABASE_URL is correct

---

## What to Report Back

Please check all 4 parts above and tell me:

1. **Part 1:** Do you see `Set-Cookie` in sign-in response? (Yes/No)
2. **Part 2:** Do you see `better-auth.session_token` cookie stored? (Yes/No)
3. **Part 3:** Do you see `Cookie` header in get-session request? (Yes/No)
4. **Part 4:** What does the get-session response show? (Copy the JSON)

This will tell me exactly where the problem is!

---

## Video Alternative

If text instructions are hard to follow, I can guide you through live. Just tell me what you see in the Network tab when you click on the sign-in request.
