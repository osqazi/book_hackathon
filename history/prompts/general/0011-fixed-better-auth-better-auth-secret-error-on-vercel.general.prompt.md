---
id: 0011
title: Fixed Better-Auth BETTER_AUTH_SECRET Error on Vercel
stage: general
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: osqazi
command: /sp.phr
labels: ["better-auth", "vercel", "environment-variables", "deployment", "error-fix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .env
  - src/lib/better-auth/auth.ts
tests:
  - none
---

## Prompt

vercel showing error:
Unhandled Rejection: You are using the default secret. Please set `BETTER_AUTH_SECRET` in your environment variables or pass `secret` in your auth config.
[dotenv@17.2.3] injecting env (0) from .env -- tip: 🔐 prevent building .env in docker: https://dotenvx.com/prebuild
Legacy server listening...
Unhandled Rejection: You are using the default secret. Please set `BETTER_AUTH_SECRET` in your environment variables or pass `secret` in your auth config.
[dotenv@17.2.3] injecting env (0) from .env -- tip: 🔐 prevent building .env in docker: https://dotenvx.com/prebuild
Legacy server listening...

## Response snapshot

Diagnosed missing BETTER_AUTH_SECRET environment variable from Vercel deployment logs showing "(0) from .env" indicating no environment variables loaded. Generated cryptographically secure secret using Node.js crypto (32 bytes, base64-encoded). Added secret to local .env file and updated src/lib/better-auth/auth.ts to include `secret: process.env.BETTER_AUTH_SECRET || ""` in auth configuration. Provided step-by-step instructions for adding the environment variable in Vercel dashboard or CLI, emphasizing to select all environments (Production, Preview, Development) and redeploy.

## Outcome

- ✅ Impact: Fixed Better-Auth initialization error on Vercel by properly configuring the required BETTER_AUTH_SECRET environment variable and updating auth configuration
- 🧪 Tests: none
- 📁 Files: .env (added BETTER_AUTH_SECRET=S12aLXBp4w5ndFK8hACHYbCa7RKLkH13SyA36Kum4pM=), src/lib/better-auth/auth.ts (added secret field at line 9)
- 🔁 Next prompts: Set BETTER_AUTH_SECRET in Vercel dashboard environment variables, redeploy to verify fix, test authentication flow post-deployment
- 🧠 Reflection: Better-Auth requires explicit secret configuration - the default secret warning prevents deployment to catch security issues early and protect production systems

## Evaluation notes (flywheel)

- Failure modes observed: None - user provided complete error message with deployment context showing dotenv loaded 0 variables
- Graders run and results (PASS/FAIL): N/A - deployment fix pending user action in Vercel dashboard
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Add validation check in auth.ts to throw descriptive error during local development when BETTER_AUTH_SECRET is missing or empty
