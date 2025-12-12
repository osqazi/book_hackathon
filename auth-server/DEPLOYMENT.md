# Auth Server Deployment Guide

This guide explains how to deploy the auth-server to production on Vercel while maintaining local development.

## Configuration Overview

The auth-server now supports both development and production environments:

### Development URLs
- Frontend: `http://localhost:3000` (Docusaurus)
- Auth Server: `http://localhost:3001`

### Production URLs
- Frontend: `https://osqazi.github.io/book_hackathon`
- Auth Server: `https://book-hackathon-alpha.vercel.app`

## Deployment Steps

### 1. Vercel Deployment

1. Deploy the `auth-server` directory to Vercel
2. Set the following environment variables in Vercel project settings:

```bash
NODE_ENV=production
AUTH_BASE_URL=https://book-hackathon-alpha.vercel.app
FRONTEND_URL=https://osqazi.github.io/book_hackathon
NEON_DATABASE_URL=<your-database-url>
```

3. Configure Vercel build settings:
   - **Framework Preset**: Other
   - **Build Command**: `npm install`
   - **Output Directory**: Leave empty
   - **Install Command**: `npm install`

### 2. Frontend Configuration

Update your frontend (Docusaurus) to use the correct auth server URL:

#### Development
```javascript
const authClient = createAuthClient({
  baseURL: "http://localhost:3001"
});
```

#### Production
```javascript
const authClient = createAuthClient({
  baseURL: "https://book-hackathon-alpha.vercel.app"
});
```

**Recommended approach**: Use environment variables in your frontend:

```javascript
const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_SERVER_URL || "http://localhost:3001"
});
```

Then set `REACT_APP_AUTH_SERVER_URL=https://book-hackathon-alpha.vercel.app` in your GitHub Pages deployment.

### 3. CORS Configuration

The auth-server is already configured to whitelist:
- `http://localhost:3000` (development)
- `http://localhost:3001` (development)
- `https://book-hackathon-alpha.vercel.app` (production backend)
- `https://osqazi.github.io` (production frontend)

### 4. Better-Auth Configuration

The Better-Auth instance is configured to:
- Use environment-based `baseURL` (automatically switches between dev and production)
- Trust all necessary origins (both dev and production)
- Enable secure cookies in production (`useSecureCookies: true` when `NODE_ENV=production`)
- Enable origin checks in production (`disableOriginCheck: false` when `NODE_ENV=production`)

## Testing

### Local Development
1. Keep `.env` with `NODE_ENV=development`
2. Run: `npm run dev`
3. Test authentication at `http://localhost:3000`

### Production Testing
1. After deploying to Vercel, test the health endpoint:
   ```bash
   curl https://book-hackathon-alpha.vercel.app/health
   ```

2. Test authentication from your production frontend at `https://osqazi.github.io/book_hackathon`

## Troubleshooting

### CORS Issues
If you see CORS errors:
1. Verify the frontend URL is in the CORS whitelist (server.js:51-58)
2. Verify the frontend is using the correct auth server URL
3. Check browser console for specific origin being rejected

### Cookie Issues
If authentication works but sessions don't persist:
1. Verify `credentials: true` is set in frontend fetch requests
2. Verify cookies are being sent with proper SameSite settings
3. Check that HTTPS is used in production (required for secure cookies)

### Origin Check Failures
If you see "INVALID_ORIGIN" errors:
1. Verify `trustedOrigins` includes the frontend URL (auth.js:23-30)
2. Check that `NODE_ENV` is set correctly in environment variables
3. Verify the `baseURL` matches your deployment URL

## Environment Variables Reference

### Development (.env)
```bash
NODE_ENV=development
AUTH_BASE_URL=http://localhost:3001
FRONTEND_URL=http://localhost:3000
NEON_DATABASE_URL=<your-database-url>
```

### Production (Vercel Environment Variables)
```bash
NODE_ENV=production
AUTH_BASE_URL=https://book-hackathon-alpha.vercel.app
FRONTEND_URL=https://osqazi.github.io/book_hackathon
NEON_DATABASE_URL=<your-database-url>
```

## Security Notes

1. **Secure Cookies**: Automatically enabled in production
2. **Origin Validation**: Automatically enabled in production
3. **HTTPS**: Required in production for cookie security
4. **Database Connection**: Uses SSL/TLS connection pooling via Neon

## Next Steps

After successful deployment:
1. Update frontend to use production auth server URL
2. Test signup/signin flow on production
3. Verify user background data is persisted correctly
4. Monitor Vercel logs for any errors
5. Set up monitoring/alerting for auth endpoints
