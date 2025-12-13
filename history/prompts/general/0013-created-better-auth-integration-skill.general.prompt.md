---
id: 0013
title: Created Better-Auth Integration Skill
stage: general
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: osqazi
command: none
labels: ["skill-creation", "better-auth", "knowledge-capture", "documentation", "best-practices"]
links:
  spec: specs/002-better-auth-signup/spec.md
  ticket: null
  adr: history/adr/0001-better-auth-architecture-decision.md
  pr: null
files:
  - .claude/skills/better-auth-integration.md
tests:
  - none
---

## Prompt

we have integrated Better-Auth Authentication in the docusaurus project using speckit-plus and manual correction. now the authentication is working perfectly. based on this experience, create a skill better-auth-integration

## Response snapshot

Created a comprehensive skill file at `.claude/skills/better-auth-integration.md` that encapsulates all knowledge and best practices from the successful Better-Auth integration in the Docusaurus Humanoid Robotics Book project.

The skill includes:

1. **Architecture Pattern**: Dual-server architecture (separate auth server + Docusaurus client)
2. **File Structure**: Complete directory layout with all components, hooks, services, and configuration files
3. **Core Implementation**: Detailed code examples for:
   - Auth server configuration with custom fields and cookie handling
   - Express server setup with CORS
   - Client configuration with environment detection
   - Authentication hooks and context providers
4. **Environment Variables**: Complete list of required env vars for both dev and production
5. **Common Issues and Solutions**: Documented fixes for:
   - INVALID_ORIGIN error (trustedOrigins configuration)
   - BETTER_AUTH_SECRET deployment error
   - auth-client.js 404 error (script injection removal)
   - Cross-origin cookie issues (defaultCookieAttributes API)
   - Database migration issues
6. **Development Workflow**: Step-by-step setup, development servers, and testing checklist
7. **Production Deployment**: Guides for Vercel, Hugging Face, and GitHub Pages deployment
8. **Best Practices**: Security, performance, code quality, and UX recommendations
9. **Extension Points**: Examples for adding social providers, custom endpoints, and content personalization

The skill serves as a complete reference for future Better-Auth integrations in Docusaurus projects.

## Outcome

- ✅ Impact: Created reusable skill that captures institutional knowledge from Better-Auth integration, enabling faster and more reliable future authentication implementations
- 🧪 Tests: none (documentation artifact)
- 📁 Files: .claude/skills/better-auth-integration.md (comprehensive skill file with 400+ lines)
- 🔁 Next prompts: Use this skill when implementing authentication in other Docusaurus projects or when troubleshooting auth issues
- 🧠 Reflection: Skill creation after successful implementation is crucial for knowledge retention and transfer. The integration involved solving multiple non-obvious issues (cookie attributes API, origin validation, script injection) that would be valuable for future projects.

## Evaluation notes (flywheel)

- Failure modes observed: None - reviewed all implementation artifacts (spec, plan, PHRs, ADR, source files) to ensure comprehensive coverage
- Graders run and results (PASS/FAIL): Not applicable - documentation artifact
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Test the skill by using it to guide a fresh Better-Auth integration in a new Docusaurus project
