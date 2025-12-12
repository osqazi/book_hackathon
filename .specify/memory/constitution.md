<!-- Sync Impact Report:
Version: 1.0.0 → 2.0.0 (Major update: Complete rewrite of principles for Better-Auth authentication focus)
Modified principles: All principles replaced with Better-Auth authentication and background profiling focus
Added sections:
  - Core Principles (6 principles for authentication system)
  - Development Standards (authentication-specific standards)
  - Security & Privacy Requirements
  - Governance (updated for new focus)
Removed sections: Previous Docusaurus deployment principles
Templates requiring updates:
  ✅ plan-template.md - Updated for authentication system focus
  ✅ spec-template.md - Requirements aligned with authentication system
  ✅ tasks-template.md - Task categorization updated for authentication workflow
  ⚠️ commands may need review for authentication-specific guidance
Follow-up TODOs: None - all placeholders resolved
-->

# Humanoid Robotics Book Authentication Constitution

## Core Principles

### I. Strict Better-Auth Adherence

All authentication implementations MUST strictly follow Better-Auth official documentation and examples. When implementing Better-Auth features, the Context7 MCP Server MUST be consulted for up-to-date documentation and best practices. No custom authentication logic may be implemented outside of Better-Auth's established patterns.

**Rationale**: Ensures compatibility with the official library, maintainability, and follows security best practices established by the Better-Auth maintainers.

### II. Seamless Docusaurus Integration

Authentication features MUST integrate seamlessly with Docusaurus architecture without disrupting existing functionality. All authentication components MUST be compatible with Docusaurus' client-side rendering and static generation patterns. The authentication system MUST work harmoniously with Docusaurus' plugin ecosystem.

**Rationale**: Maintains the integrity of the existing Docusaurus-based book while adding authentication functionality without causing conflicts or performance issues.

### III. User Privacy & Data Minimization

All background profiling questions and user data collection MUST adhere to privacy-first principles with minimal data collection. Only essential background information MAY be collected during signup. All user data collection MUST be transparent, with clear opt-in mechanisms and explicit purpose statements.

**Rationale**: Protects user privacy while enabling personalized content delivery based on user background in software/hardware fields relevant to humanoid robotics.

### IV. Full TypeScript Type Safety

All authentication code MUST be fully typed with TypeScript. Custom field definitions for background profiling MUST include proper TypeScript interfaces. Client-side session management code MUST include comprehensive type definitions for all user data structures.

**Rationale**: Ensures code maintainability, reduces runtime errors, and provides better developer experience for future contributors to the authentication system.

### V. Secure Storage & Retrieval

User background data MUST be stored securely using Better-Auth's recommended practices. All data transmission MUST use encrypted channels. Session management MUST follow security best practices with proper token handling and validation. Background profiling data MUST be protected with appropriate access controls.

**Rationale**: Protects sensitive user information and maintains trust in the authentication system while enabling the personalization features.

### VI. Maintainable & Extensible Code

Authentication implementation MUST follow clean code principles with clear separation of concerns. Components MUST be modular and reusable. Future content personalization features based on user background MUST be accommodated through extensible architecture patterns.

**Rationale**: Enables future enhancements to the personalization system while maintaining code quality and reducing technical debt.

## Development Standards

### Authentication Implementation
- All Better-Auth configuration MUST follow official documentation patterns
- Custom fields for background questions MUST be properly defined and validated
- Error handling MUST be comprehensive with user-friendly feedback
- Session management MUST be consistent across all pages

### TypeScript Standards
- All user data interfaces MUST be explicitly defined
- Authentication state MUST be properly typed
- API responses MUST have strict typing
- Component props MUST include proper authentication-related types

### Code Organization
- Authentication logic MUST be separated from presentation logic
- Reusable authentication components MUST be placed in shared locations
- Custom hooks for authentication state management MUST follow React best practices
- Error boundaries MUST be implemented around authentication components

### Testing Requirements
- Authentication flows MUST be tested end-to-end
- Background profiling data collection MUST be verified
- Session persistence MUST be validated across page refreshes
- Error states MUST be tested and handled appropriately

## Security & Privacy Requirements

### Data Collection Standards
- Background profiling questions MUST be limited to software/hardware expertise
- All collected data MUST have explicit user consent
- Data retention policies MUST be clearly defined
- Users MUST be able to view and update their background information

### Access Control
- Authentication state MUST be validated before accessing protected content
- Background data access MUST be restricted to authorized personnel
- API endpoints MUST implement proper authentication checks
- Session timeouts MUST be configured appropriately

### Privacy Compliance
- Personalization based on background data MUST be opt-in
- Users MUST be informed about how their data will be used
- Data deletion requests MUST be honored promptly
- All data handling MUST comply with applicable privacy regulations

### Pre-Implementation Checklist
- [ ] Better-Auth configuration follows official documentation
- [ ] Custom background fields properly defined and validated
- [ ] TypeScript interfaces cover all user data structures
- [ ] Security measures implemented for data storage and transmission
- [ ] Session management works across all application routes
- [ ] Privacy controls and user consent mechanisms in place
- [ ] Error handling provides clear feedback to users
- [ ] All authentication flows tested end-to-end

### Success Validation
A successful authentication implementation MUST satisfy:

1. **Authentication Success**: Email/password signup and login work correctly
2. **Background Profiling**: Custom fields capture user software/hardware background
3. **Session Management**: Sessions persist appropriately across page visits
4. **Security**: All data transmission occurs over encrypted channels
5. **Privacy**: User consent obtained for data collection and usage
6. **Personalization**: Background data enables content personalization features
7. **Type Safety**: All code passes TypeScript compilation without errors
8. **User Experience**: Authentication flows are intuitive and user-friendly

## Governance

This constitution supersedes all other authentication development practices and preferences for this project. Any deviation requires explicit justification documented as an Architecture Decision Record (ADR).

### Amendment Process

1. Proposed amendments MUST be documented with rationale
2. Impact analysis MUST assess effects on authentication security and user privacy
3. Version MUST be incremented using semantic versioning:
   - **MAJOR**: Backward-incompatible authentication changes or security model shifts
   - **MINOR**: New authentication features or privacy enhancements
   - **PATCH**: Clarifications, wording improvements, non-semantic fixes
4. All dependent templates MUST be updated for consistency

### Compliance and Review

- All authentication specifications MUST reference and comply with these principles
- All pull requests affecting authentication MUST be reviewed for constitutional compliance
- Security additions MUST be justified in implementation plans
- Constitution violations MUST be documented with explicit rationale

### Runtime Guidance

For detailed authentication development workflow and agent-specific instructions, refer to `CLAUDE.md` (agent runtime guidance) and command files in `.specify/commands/`.

**Version**: 2.0.0 | **Ratified**: 2025-12-11 | **Last Amended**: 2025-12-11