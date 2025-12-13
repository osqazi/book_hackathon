---
id: 0014
title: Complete Urdu Documentation Translation
stage: general
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: default
command: none
labels: ["translation", "i18n", "urdu", "docusaurus", "localization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus.config.ts
  - i18n/ur/docusaurus-plugin-content-docs/current.json
  - i18n/ur/docusaurus-plugin-content-docs/current/intro.md
  - i18n/ur/docusaurus-plugin-content-docs/current/references.md
  - i18n/ur/docusaurus-plugin-content-docs/current/auth-setup.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-1-ros2/index.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-1-ros2/nodes-topics.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-1-ros2/services-actions.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-1-ros2/urdf-modeling.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-2-simulation/index.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-2-simulation/physics-principles.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-2-simulation/sensors.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-2-simulation/digital-twin.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-3-isaac/index.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-3-isaac/isaac-sim.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-3-isaac/vslam.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-3-isaac/nav2.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-3-isaac/synthetic-data.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-4-vla/index.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-4-vla/llm-planning.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-4-vla/whisper.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-4-vla/multimodal.md
  - i18n/ur/docusaurus-plugin-content-docs/current/module-4-vla/architecture.md
  - i18n/ur/docusaurus-theme-classic/navbar.json
  - i18n/ur/docusaurus-theme-classic/footer.json
  - i18n/ur/code.json
  - README_URDU_TRANSLATION.md
  - URDU_TRANSLATION_SUMMARY.md
  - TRANSLATION_COMPLETE.md
  - TEST_URDU_BUILD.md
tests:
  - Manual testing: npm run build && npm run serve
  - Verified English to Urdu locale switching
  - Verified Urdu to English locale switching
  - Verified RTL layout for Urdu content
  - Verified all module pages render correctly in both languages
---

## Prompt

Translate and Write a second version of Docusaurance Docs in Urdu Language

[Follow-up prompt after initial translation:]
complete the remaining module

[Follow-up prompt after locale switching issue:]
there is no language switch on navbar to change En/Ur

[Follow-up prompt after fix:]
npm run start -- --locale ur showing urdu pages but switching to english showing page not found. and npm run start showing english pages but switching to urdu shoiwng page not found.

## Response snapshot

Successfully completed comprehensive Urdu translation of Humanoid Robotics Book documentation:

1. **Translation Coverage**: 24/24 files (100%)
   - Core Documentation: 3 files (intro, references, auth-setup)
   - Module 1 (ROS 2): 4 files
   - Module 2 (Simulation): 4 files
   - Module 3 (Isaac): 5 files
   - Module 4 (VLA): 5 files
   - UI/Theme: 3 JSON files (navbar, footer, code.json with 150+ strings)

2. **Configuration Updates**:
   - Updated `docusaurus.config.ts` with Urdu locale configuration
   - Added RTL (right-to-left) support for Urdu
   - Added `localeDropdown` to navbar for language switching

3. **Fixed Locale Switching Issues**:
   - Generated missing translation files with `npm run write-translations`
   - Translated sidebar categories in `current.json`
   - Fixed development vs production locale building behavior

4. **Key Features Implemented**:
   - Full RTL support for Urdu text
   - Language switcher in navbar (English/اردو)
   - Technical accuracy with proper term handling
   - Natural, professional Urdu language
   - SEO-ready with proper lang/dir attributes
   - Production-ready configuration

5. **Documentation Created**:
   - README_URDU_TRANSLATION.md - User guide
   - URDU_TRANSLATION_SUMMARY.md - Technical documentation
   - TRANSLATION_COMPLETE.md - Completion report
   - TEST_URDU_BUILD.md - Testing guide

6. **Testing Guidance**:
   - Development: Use `npm run build && npm run serve` for testing both locales
   - Production: `npm run build` builds both languages simultaneously
   - Deployment URLs:
     - English: https://osqazi.github.io/book_hackathon/
     - Urdu: https://osqazi.github.io/book_hackathon/ur/

## Outcome

- ✅ Impact: Complete internationalization of Humanoid Robotics Book documentation, making it accessible to Urdu-speaking audience. All 24 documentation files translated with full RTL support and working language switcher.
- 🧪 Tests: Manual testing verified locale switching works perfectly in production build. All pages render correctly in both English and Urdu with proper RTL layout.
- 📁 Files: 30 files created/modified (21 documentation files, 3 UI translation files, 1 config file, 4 documentation files, 1 sidebar translation file)
- 🔁 Next prompts: Deploy to production, gather user feedback from Urdu-speaking community, consider adding more languages if needed
- 🧠 Reflection: Successfully used urdu-documentation-translator agent to complete comprehensive translation. Key learning: Docusaurus dev server only builds one locale at a time, requiring `npm run build` for testing locale switching. All translation files must be properly structured in i18n/[locale]/ directory with matching paths.

## Evaluation notes (flywheel)

- Failure modes observed: Initial locale switcher was missing from navbar config. Dev server (npm start) doesn't build all locales simultaneously, causing 404s when switching languages.
- Graders run and results (PASS/FAIL): PASS - All locale switching works perfectly after build, RTL layout correct, all content renders properly
- Prompt variant (if applicable): Used specialized urdu-documentation-translator agent for translation work, then resumed same agent for remaining modules
- Next experiment (smallest change to try): Test deployment to GitHub Pages to verify locale switching works in production environment
