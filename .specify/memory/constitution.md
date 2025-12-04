# Docusaurus Book Deployment Constitution

<!--
Sync Impact Report:
Version: 1.0.0 → 1.0.0 (Initial ratification)
Modified principles: N/A (new constitution)
Added sections:
  - Core Principles (6 principles)
  - Development Standards
  - Deployment & Validation
  - Governance
Removed sections: None
Templates requiring updates:
  ✅ plan-template.md - Constitution Check section validated for compatibility
  ✅ spec-template.md - Requirements align with accuracy, reproducibility, and clarity principles
  ✅ tasks-template.md - Task categorization compatible with book generation workflow
Follow-up TODOs: None - all placeholders resolved
-->

## Core Principles

### I. Spec-Driven Generation

Every book component MUST be generated through explicit, documented spec-driven prompts. No content may be manually created outside the Spec-Kit Plus workflow. All structural decisions, content organization, and technical implementations MUST be traceable to a specification document.

**Rationale**: Ensures reproducibility, maintainability, and adherence to the AI/Spec-Driven Development methodology that is the subject matter of the book itself.

### II. Technical Accuracy

Every technical claim, command, code snippet, and deployment step MUST be verified against official Docusaurus documentation and tested in a real environment. No assumed or inferred technical details are permitted. When using Docusaurus features, the Context7 MCP Server MUST be consulted for up-to-date documentation.

**Rationale**: Readers depend on accurate, working instructions. Unverified claims undermine trust and create frustration when steps fail during reproduction.

### III. Reproducibility

All instructions and workflows MUST be reproducible by following the book sequentially from start to finish. Each step MUST include explicit commands, expected outputs, and clear success criteria. Prerequisites MUST be stated upfront with verification steps.

**Rationale**: A deployment guide is only valuable if readers can successfully execute every step without prior knowledge or additional research.

### IV. Clarity Over Brevity

Technical explanations MUST prioritize comprehension over conciseness. Each concept MUST include context, rationale, and practical examples. Code snippets MUST be complete and runnable, not abbreviated. Complex topics MUST follow a logical progression from fundamentals to advanced usage.

**Rationale**: Developers learning Docusaurus deployment need understanding, not just commands to copy. Clear explanations enable problem-solving when unexpected issues arise.

### V. Docusaurus MCP Server Integration

All Docusaurus-specific information, features, configurations, and best practices MUST be sourced from the Docusaurus MCP Server via Context7. Direct assumptions about Docusaurus functionality without verification are prohibited.

**Rationale**: Docusaurus evolves rapidly. The MCP Server ensures current, accurate documentation rather than outdated assumptions.

### VI. GitHub Pages Deployment Focus

The book MUST culminate in a successful GitHub Pages deployment. All setup, configuration, and build steps MUST be validated against GitHub Pages hosting requirements. Alternative deployment targets are out of scope unless explicitly specified as extensions.

**Rationale**: GitHub Pages is the defined deployment target. Focus ensures depth and completeness for this specific platform rather than superficial coverage of many options.

## Development Standards

### Markdown Quality

- All content MUST be valid Markdown compatible with Docusaurus (MDX support where needed)
- Code blocks MUST specify language for syntax highlighting
- Internal links MUST use relative paths and be validated
- Images and assets MUST be properly referenced and optimized
- Frontmatter MUST follow Docusaurus conventions

### Code Examples

- All code snippets MUST be tested and verified working
- Configuration examples MUST match current Docusaurus best practices
- Shell commands MUST be tested on the target platform (Windows/Linux/macOS specifics documented)
- Error cases and troubleshooting MUST be included where relevant

### Structure and Organization

- Content MUST follow a logical learning progression: setup → writing → configuration → build → deployment
- Each chapter MUST have clear learning objectives and outcomes
- Cross-references between sections MUST be explicit and navigable
- Table of contents MUST accurately reflect content hierarchy

### Testing Requirements

- The complete workflow (setup through deployment) MUST be tested end-to-end
- Each deployment step MUST be verified against a real GitHub repository
- Build commands MUST be executed and output validated
- Deployed site MUST be accessible and functional on GitHub Pages

## Deployment & Validation

### Pre-Deployment Checklist

- [ ] All Docusaurus features used are documented from MCP Server
- [ ] All commands tested and verified working
- [ ] All links (internal and external) validated
- [ ] Build succeeds without errors or warnings
- [ ] Site deploys successfully to GitHub Pages
- [ ] Deployed site renders correctly in major browsers
- [ ] All code examples tested and working
- [ ] Search functionality operational (if enabled)

### Success Validation

A successful deployment MUST satisfy:

1. **Build Success**: `npm run build` completes without errors
2. **Deployment Success**: Site is accessible at the expected GitHub Pages URL
3. **Content Integrity**: All pages render correctly with proper formatting
4. **Navigation**: All navigation elements function correctly
5. **Reproducibility**: Following the book instructions produces an identical working deployment

## Governance

This constitution supersedes all other development practices and preferences for this project. Any deviation requires explicit justification documented as an Architecture Decision Record (ADR).

### Amendment Process

1. Proposed amendments MUST be documented with rationale
2. Impact analysis MUST assess affected templates and workflows
3. Version MUST be incremented using semantic versioning:
   - **MAJOR**: Backward-incompatible principle changes or removals
   - **MINOR**: New principles or substantial expansions
   - **PATCH**: Clarifications, wording improvements, non-semantic fixes
4. All dependent templates MUST be updated for consistency

### Compliance and Review

- All specifications MUST reference and comply with these principles
- All pull requests MUST be reviewed for constitutional compliance
- Complexity additions MUST be justified in implementation plans
- Constitution violations MUST be documented with explicit rationale

### Runtime Guidance

For detailed development workflow and agent-specific instructions, refer to `CLAUDE.md` (agent runtime guidance) and command files in `.specify/templates/commands/`.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
