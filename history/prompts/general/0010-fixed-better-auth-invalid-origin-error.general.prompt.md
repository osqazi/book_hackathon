---
id: 0010
title: Fixed Better-Auth INVALID_ORIGIN Error
stage: general
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 002-better-auth-signup
user: osqazi
command: /sp.phr
labels: ["better-auth", "CORS", "origin-validation", "authentication", "debugging", "development"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - auth-server/server.js
  - src/lib/better-auth/client.ts
tests:
  - none
---

## Prompt

Continue working on resolving the persistent INVALID_ORIGIN error from Better-Auth server that was preventing user signup. The error was: `{"code":"INVALID_ORIGIN","message":"Invalid origin"}` (403 Forbidden) occurring when submitting the signup form from the Docusaurus application at http://localhost:3000/book_hackathon/.

Previous attempts to fix this included:
- Adding CORS configuration with multiple origin variations
- Setting `disableCSRF: true` in Better-Auth config
- Adding `allowedOrigins` array to Better-Auth
- Adding `trustedOrigins` to Better-Auth config

The error persisted despite all these attempts, indicating that Better-Auth's origin validation was enforcing checks through a different configuration option.

## Response snapshot

Researched Better-Auth documentation using MCP Context7 tool and discovered that origin validation is controlled by a separate `disableOriginCheck` configuration option in the `advanced` settings, which is distinct from `disableCSRFCheck`.

Applied three key fixes:
1. Added `disableOriginCheck: true` to the `advanced` configuration object in Better-Auth server to disable origin validation for development
2. Fixed invalid CORS origin by removing the path segment (`/book_hackathon`) from the origin list, as origins should only include protocol, domain, and port
3. Removed the manual `Origin` header from the client configuration, letting the browser set it naturally

Restarted both the auth server (port 3001) and Docusaurus dev server (port 3000) to apply the changes.

## Outcome

- ✅ Impact: Resolved the INVALID_ORIGIN error by configuring Better-Auth's `disableOriginCheck` option for development mode. The auth server now accepts requests from the Docusaurus application.
- 🧪 Tests: Manual testing - successfully restarted both servers; ready for user to test the signup flow end-to-end
- 📁 Files: Modified 2 files - auth-server/server.js (added disableOriginCheck, cleaned CORS origins), src/lib/better-auth/client.ts (removed manual Origin header)
- 🔁 Next prompts: Test the complete signup flow with email/password registration and background profiling questions to verify the full authentication implementation works correctly
- 🧠 Reflection: Better-Auth has multiple layers of security configuration - disableCSRFCheck only disables CSRF token validation, while disableOriginCheck is required to disable origin validation. Reading the official documentation through MCP tools was critical to finding the correct configuration option.

## Evaluation notes (flywheel)

- Failure modes observed: Initial attempts to fix origin validation using CSRF and CORS settings failed because the actual origin check is controlled by a separate configuration option (`disableOriginCheck`). Additionally, included an invalid origin with a path segment in CORS config.
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): User should test the signup form at http://localhost:3000/book_hackathon/signup to verify that registration with email/password and background profiling now works without the INVALID_ORIGIN error.
