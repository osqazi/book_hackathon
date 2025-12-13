---
id: 0015
title: Fixed Bilingual Landing Page Urdu Translations
stage: general
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: user
command: interactive
labels: ["i18n", "urdu", "translation", "landing-page", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - src/pages/index.tsx
  - i18n/ur/code.json
tests:
  - Manual testing: Language switching on landing page
---

## Prompt

the project is bilangual however the landing page showing in english in both the language selection

## Response snapshot

Fixed the bilingual landing page issue by implementing proper Docusaurus i18n support for the React-based homepage.

**Root Cause:**
The landing page (src/pages/index.tsx) had hardcoded English text without using Docusaurus's Translate component. When users switched to Urdu, there were no translations available.

**Solution Implemented:**

1. Updated src/pages/index.tsx:
   - Added Translate and translate imports from @docusaurus/Translate
   - Wrapped all user-facing text in Translate components with unique IDs
   - Sections updated: hero, features (4 cards), author bio, call-to-action

2. Added Urdu Translations to i18n/ur/code.json:
   - Used npm run write-translations to extract translation keys
   - Replaced English default messages with natural Urdu translations
   - 14 new translation keys added (lines 373-435)

3. Key Translations:
   - Hero description in Urdu
   - Features title in Urdu
   - CTA in Urdu
   - Author bio with full Urdu translation

**Technical Details:**
- Docusaurus pages use code.json for translations
- Cleared .docusaurus and build directories to force rebuild
- Maintained technical terms (ROS 2, GitHub, NVIDIA Isaac) in English within Urdu context

## Outcome

- Impact: Landing page now displays correctly in both English and Urdu based on language selection
- Tests: Manual testing confirmed - language switching works, all content translates properly
- Files: Modified 2 files (index.tsx, code.json)
- Next prompts: None - feature complete
- Reflection: Initial approach of creating separate docusaurus-plugin-content-pages directory was incorrect

## Evaluation notes (flywheel)

- Failure modes observed: Initially used wrong translation file location
- Graders run and results: User confirmed "yes thats good" - PASS
- Prompt variant: N/A
- Next experiment: For future i18n work, validate translation file structure upfront
