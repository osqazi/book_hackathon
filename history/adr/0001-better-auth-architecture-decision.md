# ADR-0001: Better-Auth Architecture Decision

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** 002-better-auth-signup
- **Context:** Need to implement authentication with background profiling for personalized content in Docusaurus-based Humanoid Robotics Book. The solution must integrate with existing Python backend while providing user experience personalization based on software/hardware expertise.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Authentication Framework: Better-Auth v1.4.6 (comprehensive TypeScript auth solution)
- Architecture Pattern: Separate Auth Server (Node.js/Express) with Docusaurus integration
- Database: SQLite for development, PostgreSQL for production via Drizzle ORM
- User Profiling: Custom fields in Better-Auth for software/hardware background data
- Personalization: Client-side content adaptation based on user background
- Session Management: 7-day persistent sessions as specified

## Consequences

### Positive

- Comprehensive authentication features out of the box (email/password, sessions, security)
- TypeScript-first approach with strong typing for auth and user data
- Clean separation of concerns between auth and main application
- Easy to extend with custom fields for background profiling
- Good integration with existing React/Docusaurus frontend
- Proper security practices handled by the framework
- Content personalization based on user expertise levels

### Negative

- Additional infrastructure complexity with separate auth server
- Potential network latency between Docusaurus and auth server
- Extra dependency management for auth-specific components
- Need to maintain separate auth server deployment
- Possible CORS configuration requirements
- Additional database connections and management

## Alternatives Considered

Alternative A: Custom authentication implementation with Python backend
- Components: Flask/FastAPI auth endpoints, custom session management, manual security measures
- Why rejected: Would require significant development time, potential security vulnerabilities, reinventing authentication standards

Alternative B: Third-party authentication services (Auth0, Firebase Auth)
- Components: External auth provider, configuration via API keys, limited customization
- Why rejected: Less control over user data collection for background profiling, potential vendor lock-in, additional costs, limited ability to collect custom background data

Alternative C: Self-hosted auth solution (NextAuth.js)
- Components: NextAuth.js, separate Next.js app for auth, API routes
- Why rejected: Would require changing from Docusaurus to Next.js, major architectural shift, loss of existing documentation site benefits

## References

- Feature Spec: specs/002-better-auth-signup/spec.md
- Implementation Plan: specs/002-better-auth-signup/plan.md
- Related ADRs: none
- Evaluator Evidence: history/prompts/general/0008-better-auth-signup-and-signin-implementation.general.prompt.md
