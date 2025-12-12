---
id: 0012
title: Fixed auth-client.js 404 Error on GitHub Pages
stage: general
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: osqazi
command: /sp.phr
labels: ["debugging", "deployment", "github-pages", "better-auth", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - plugins/docusaurus-auth/index.js
tests:
  - none
---

## Prompt

https://book-hackathon-alpha.vercel.app/health showing:
{"status":"ok","service":"Auth Server"}

but thru Signing in from https://osqazi.github.io/book_hackathon/signin/, it showing error:

GET https://osqazi.github.io/auth-client.js net::ERR_ABORTED 404 (Not Found)

## Response snapshot

Identified and fixed the root cause: the `plugins/docusaurus-auth/index.js` was injecting an unnecessary script tag for `/auth-client.js` that didn't exist and used an absolute path incompatible with the GitHub Pages baseUrl. Removed the `injectHtmlTags()` function since the auth client is already properly imported via React (`better-auth/react`). User needs to rebuild and redeploy to GitHub Pages to apply the fix.

## Outcome

- ✅ Impact: Fixed 404 error preventing signin functionality on GitHub Pages deployment
- 🧪 Tests: Manual testing required after rebuild and redeployment
- 📁 Files: plugins/docusaurus-auth/index.js (removed injectHtmlTags function)
- 🔁 Next prompts: "Run npm run build and deploy to GitHub Pages to verify the fix works"
- 🧠 Reflection: Unnecessary script injection caused path resolution issues with Docusaurus baseUrl; the auth client was already properly configured via React imports

## Evaluation notes (flywheel)

- Failure modes observed: None. Issue was quickly identified by examining the error message and tracing the script injection in the plugin.
- Graders run and results (PASS/FAIL): Not applicable - manual verification required after deployment
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): None needed - fix is straightforward removal of unnecessary code
